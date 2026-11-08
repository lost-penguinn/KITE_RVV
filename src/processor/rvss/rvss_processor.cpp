/**
 * @file rvss_vm.cpp
 * @brief RVSS VM implementation
 * @author Vishank Singh, https://github.com/VishankSingh
 */

#include "processor/rvss/rvss_processor.h"
#include "common/globals.h"
#include "common/instructions.h"
#include "common/vector_defs.h"
#include "config/config.h"
#include "ui/processor_tab/processor_designs/rvss_processor_circuit_scene.h"
#include "utils/utils.h"
#include <QDebug>
#include <algorithm>
#include <atomic>
#include <cctype>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <queue>
#include <stack>
#include <thread>
#include <tuple>

namespace Kites
{
RVSSProcessor::RVSSProcessor() : ProcessorBase()
{
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
#ifndef DISABLE_GUI
    circuit_scene_ = std::make_unique<Kites::RVSSCircuitScene>();
    connect(this, &ProcessorBase::updateCircuitStateSignal, circuit_scene_.get(),
            &Kites::RVSSCircuitScene::updateCircuitState, Qt::QueuedConnection);
#endif

    active_wires_.append("IM_to_PC_pc");
    active_wires_.append("PC_to_IM_instruction");
    active_wires_.append("IM_to_RF_rs1_whole");
    active_wires_.append("RF_to_ALUMUX");
    active_wires_.append("PCMux_to_PC");
    active_wires_.append("Imm_to_ALUMUX");
    active_wires_.append("BranchALU_to_PCMUX");
    active_wires_.append("PCALU_to_PCMUX");
    active_wires_.append("Control_ALUOp");
    active_wires_.append("PC_to_InstrMem");
    active_wires_.append("ALUOp_to_ALU_alusignal");
    active_wires_.append("ALU_to_DM");

    always_active_wires_count_ = active_wires_.size();
}

RVSSProcessor::~RVSSProcessor() = default;

void RVSSProcessor::SetActiveWireNames()
{
    // first clear the current list
    // TODO * we dont have to clear wire that will always be active
    //  so make a list of those wires and only clear the rest
    //  for now this will
    // active_wires_.clear();

    // Map control signals to canonical wire names used in the scene JSON.
    // maybe sending integers would be better?
    // but for now, this is fine.
    // also the preformace boost well get of that will that really improve the user experience that
    // much?

    // first add the always active wires
    // this is not ideal but for now it will do

    // if(branch_flag_)
    // {

    // }

    if (control_unit_.GetMemWrite())
    {
        active_wires_.append("Control_to_MemWrite");
    }
    if (control_unit_.GetMemRead())
    {
        active_wires_.append("Control_to_MemRead");
    }
    if (control_unit_.GetRegWrite())
    {
        active_wires_.append("Control_to_Regwrite");
        active_wires_.append("WBMux_to_RF_lowest");
    }
    if (control_unit_.GetBranch())
    {
        active_wires_.append("Control_to_BranchAND");
    }
    if (control_unit_.GetAluSrc())
    {
        active_wires_.append("Control_to_ALUSrc_control");
    }
    if (control_unit_.GetMemToReg())
    {
        active_wires_.append("Control_to_MemtoRegMux");
    }
}

void RVSSProcessor::setProcessorState()
{
    processor_state_.programCounters = {program_counter_, INVALID_PC, INVALID_PC, INVALID_PC,
                                        INVALID_PC};
    processor_state_.lastExecutedPC = last_executed_pc_;
}

void RVSSProcessor::Run()
{
    ClearStop();
    while (!stop_requested_ && program_counter_ < program_size_)
    {
        if (last_breakpoint_pc_ != UINT64_MAX && program_counter_ != last_breakpoint_pc_)
        {
            last_breakpoint_pc_ = UINT64_MAX;
        }

        if (std::find(breakpoints_.begin(), breakpoints_.end(), program_counter_) !=
                breakpoints_.end() &&
            program_counter_ != last_breakpoint_pc_)
        {
            pause_requested_ = true;
            last_breakpoint_pc_ = program_counter_;
            emit processorPausedAtBreakpointSignal();
        }
        {
            QMutexLocker locker(&pause_mutex_);
            while (pause_requested_ && !stop_requested_)
            {
                pause_wait_condition_.wait(&pause_mutex_);
            }
            if (stop_requested_)
            {
                break;
            }
        }
        StepAndNotify();
        // handling the delay
        {
            QMutexLocker locker(&pause_mutex_);
            if (stop_requested_ || pause_requested_)
                continue;
            pause_wait_condition_.wait(&pause_mutex_, step_delay_);
            // we wait for step_delay_ milliseconds or until notified to wake up
        }
    }
    if (stop_requested_)
    {
        emit processorClockedSignal(processor_state_);
    }
    if (program_counter_ >= program_size_)
    {
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RVSSProcessor::Fetch()
{
    current_instruction_ = memory_controller_.readInstruction(program_counter_);
    UpdateProgramCounter(4);
}

void RVSSProcessor::Decode()
{
    control_unit_.SetControlSignals(current_instruction_);
}

void RVSSProcessor::Execute()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;

    if (opcode == 0b1110011 && funct3 == 0b000)
    {
        HandleSyscall();
        return;
    }

    if (instruction_set::isFInstruction(current_instruction_))
    { // RV64 F
        ExecuteFloat();
        return;
    }
    else if (instruction_set::isDInstruction(current_instruction_))
    {
        ExecuteDouble();
        return;
    }
    else if (opcode == 0b1110011)
    {
        ExecuteCsr();
        return;
    }
    else if (opcode == 0b1010111)
    {
        uint8_t funct3 = (current_instruction_ >> 12) & 0x7;

        if (funct3 == 0b111)
            ExecuteVsetvli();
        else
            ExecuteVector();

        return;
    }
    uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;

    int32_t imm = ImmGenerator(current_instruction_);

    uint64_t reg1_value = registers_.ReadGpr(rs1);
    uint64_t reg2_value = registers_.ReadGpr(rs2);

    bool overflow = false;

    if (control_unit_.GetAluSrc())
    {
        reg2_value = static_cast<uint64_t>(static_cast<int64_t>(imm));
    }

    alu::AluOp aluOperation =
        control_unit_.GetAluSignal(current_instruction_, control_unit_.GetAluOp());
    std::tie(execution_result_, overflow) = alu_.execute(aluOperation, reg1_value, reg2_value);

    if (control_unit_.GetBranch())
    {
        if (opcode == 0b1100111 || opcode == 0b1101111)
        {                                                      // JALR or JAL
            next_pc_ = static_cast<int64_t>(program_counter_); // PC was already updated in Fetch()
            UpdateProgramCounter(-4);
            return_address_ = program_counter_ + 4;
            if (opcode == 0b1100111)
            { // JALR
                UpdateProgramCounter(-program_counter_ + (execution_result_));
            }
            else
            { // JAL
                UpdateProgramCounter(imm);
            }
        }
        else if (opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kbeq)
                               .opcode ||
                 opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kbne)
                               .opcode ||
                 opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kblt)
                               .opcode ||
                 opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kbge)
                               .opcode ||
                 opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kbltu)
                               .opcode ||
                 opcode == instruction_set::instruction_encoding_map
                               .at(instruction_set::Instruction::kbgeu)
                               .opcode)
        {
            switch (funct3)
            {
            case 0b000:
            { // BEQ
                branch_flag_ = (execution_result_ == 0);
                break;
            }
            case 0b001:
            { // BNE
                branch_flag_ = (execution_result_ != 0);
                break;
            }
            case 0b100:
            { // BLT
                branch_flag_ = (execution_result_ == 1);
                break;
            }
            case 0b101:
            { // BGE
                branch_flag_ = (execution_result_ == 0);
                break;
            }
            case 0b110:
            { // BLTU
                branch_flag_ = (execution_result_ == 1);
                break;
            }
            case 0b111:
            { // BGEU
                branch_flag_ = (execution_result_ == 0);
                break;
            }
            }
        }
    }

    if (branch_flag_ &&
        (opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kbeq)
                       .opcode ||
         opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kbne)
                       .opcode ||
         opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kblt)
                       .opcode ||
         opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kbge)
                       .opcode ||
         opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kbltu)
                       .opcode ||
         opcode == instruction_set::instruction_encoding_map.at(instruction_set::Instruction::kbgeu)
                       .opcode))
    {
        UpdateProgramCounter(-4);
        UpdateProgramCounter(imm);
        // we have jumped so the branch alu wire will send signal to the pc mux
        active_wires_.append("ALUzero_to_ANDGATElower");
        active_wires_.append("BranchAND_to_PCMUX");
    }

    if (opcode == 0b0010111)
    { // AUIPC
        execution_result_ = static_cast<int64_t>(program_counter_) - 4 + (imm << 12);
    }
}

void RVSSProcessor::ExecuteFloat()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
    uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
    uint8_t rm = funct3;
    uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;
    uint8_t rs3 = (current_instruction_ >> 27) & 0b11111;

    uint8_t fcsr_status = 0;

    int32_t imm = ImmGenerator(current_instruction_);

    if (rm == 0b111)
    {
        rm = registers_.ReadCsr(0x002);
    }

    uint64_t reg1_value = registers_.ReadFpr(rs1);
    uint64_t reg2_value = registers_.ReadFpr(rs2);
    uint64_t reg3_value = registers_.ReadFpr(rs3);

    if (funct7 == 0b1101000 || funct7 == 0b1111000 || opcode == 0b0000111 || opcode == 0b0100111)
    {
        reg1_value = registers_.ReadGpr(rs1);
    }

    if (control_unit_.GetAluSrc())
    {
        reg2_value = static_cast<uint64_t>(static_cast<int64_t>(imm));
    }

    alu::AluOp aluOperation =
        control_unit_.GetAluSignal(current_instruction_, control_unit_.GetAluOp());
    std::tie(execution_result_, fcsr_status) =
        alu::Alu::fpexecute(aluOperation, reg1_value, reg2_value, reg3_value, rm);

    registers_.WriteCsr(0x003, fcsr_status);
}

void RVSSProcessor::ExecuteDouble()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
    uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
    uint8_t rm = funct3;
    uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;
    uint8_t rs3 = (current_instruction_ >> 27) & 0b11111;

    uint8_t fcsr_status = 0;

    int32_t imm = ImmGenerator(current_instruction_);

    uint64_t reg1_value = registers_.ReadFpr(rs1);
    uint64_t reg2_value = registers_.ReadFpr(rs2);
    uint64_t reg3_value = registers_.ReadFpr(rs3);

    if (funct7 == 0b1101001 || funct7 == 0b1111001 || opcode == 0b0000111 || opcode == 0b0100111)
    {
        reg1_value = registers_.ReadGpr(rs1);
    }

    if (control_unit_.GetAluSrc())
    {
        reg2_value = static_cast<uint64_t>(static_cast<int64_t>(imm));
    }

    alu::AluOp aluOperation =
        control_unit_.GetAluSignal(current_instruction_, control_unit_.GetAluOp());
    std::tie(execution_result_, fcsr_status) =
        alu::Alu::dfpexecute(aluOperation, reg1_value, reg2_value, reg3_value, rm);
}

void RVSSProcessor::ExecuteCsr()
{
    uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
    uint16_t csr = (current_instruction_ >> 20) & 0xFFF;
    uint64_t csr_val = registers_.ReadCsr(csr);

    csr_target_address_ = csr;
    csr_old_value_ = csr_val;
    csr_write_val_ = registers_.ReadGpr(rs1);
    csr_uimm_ = rs1;
}

// TODO: implement writeback for syscalls
void RVSSProcessor::HandleSyscall()
{
    uint64_t syscall_number = registers_.ReadGpr(17);
    switch (syscall_number)
    {
    case SYSCALL_PRINT_INT:
    {
        if (!globals::vm_as_backend)
        {
            std::cout << "[Syscall output: ";
        }
        else
        {
            std::cout << "VM_STDOUT_START";
        }
        std::cout << static_cast<int64_t>(registers_.ReadGpr(10)); // Print signed integer
        if (!globals::vm_as_backend)
        {
            std::cout << "]" << std::endl;
        }
        else
        {
            std::cout << "VM_STDOUT_END" << std::endl;
        }
        break;
    }
    case SYSCALL_PRINT_FLOAT:
    { // print float
        if (!globals::vm_as_backend)
        {
            std::cout << "[Syscall output: ";
        }
        else
        {
            std::cout << "VM_STDOUT_START";
        }
        float float_value;
        uint64_t raw = registers_.ReadGpr(10);
        std::memcpy(&float_value, &raw, sizeof(float_value));
        std::cout << std::setprecision(std::numeric_limits<float>::max_digits10) << float_value;
        if (!globals::vm_as_backend)
        {
            std::cout << "]" << std::endl;
        }
        else
        {
            std::cout << "VM_STDOUT_END" << std::endl;
        }
        break;
    }
    case SYSCALL_PRINT_DOUBLE:
    { // print double
        if (!globals::vm_as_backend)
        {
            std::cout << "[Syscall output: ";
        }
        else
        {
            std::cout << "VM_STDOUT_START";
        }
        double double_value;
        uint64_t raw = registers_.ReadGpr(10);
        std::memcpy(&double_value, &raw, sizeof(double_value));
        std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << double_value;
        if (!globals::vm_as_backend)
        {
            std::cout << "]" << std::endl;
        }
        else
        {
            std::cout << "VM_STDOUT_END" << std::endl;
        }
        break;
    }
    case SYSCALL_PRINT_STRING:
    {
        if (!globals::vm_as_backend)
        {
            std::cout << "[Syscall output: ";
        }
        PrintString(registers_.ReadGpr(10)); // Print string
        if (!globals::vm_as_backend)
        {
            std::cout << "]" << std::endl;
        }
        break;
    }
    case SYSCALL_EXIT:
    {
        stop_requested_ = true; // Stop the VM
        if (!globals::vm_as_backend)
        {
            std::cout << "VM_EXIT" << std::endl;
        }
        output_status_ = "VM_EXIT";
        std::cout << "Exited with exit code: " << registers_.ReadGpr(10) << std::endl;
        exit(0); // Exit the program
        break;
    }
    case SYSCALL_READ:
    { // Read
        uint64_t file_descriptor = registers_.ReadGpr(10);
        uint64_t buffer_address = registers_.ReadGpr(11);
        uint64_t length = registers_.ReadGpr(12);

        if (file_descriptor == 0)
        {
            // Read from stdin
            std::string input;
            {
                std::cout << "VM_STDIN_START" << std::endl;
                output_status_ = "VM_STDIN_START";
                std::unique_lock<std::mutex> lock(input_mutex_);
                input_cv_.wait(lock, [this]() { return !input_queue_.empty(); });
                output_status_ = "VM_STDIN_END";
                std::cout << "VM_STDIN_END" << std::endl;

                input = input_queue_.front();
                input_queue_.pop();
            }

            std::vector<uint8_t> old_bytes_vec(length, 0);
            std::vector<uint8_t> new_bytes_vec(length, 0);

            for (size_t i = 0; i < length; ++i)
            {
                old_bytes_vec[i] = memory_controller_.peekByte(buffer_address + i);
            }

            for (size_t i = 0; i < input.size() && i < length; ++i)
            {
                memory_controller_.writeByte(buffer_address + i, static_cast<uint8_t>(input[i]));
            }
            if (input.size() < length)
            {
                memory_controller_.writeByte(buffer_address + input.size(), '\0');
            }

            for (size_t i = 0; i < length; ++i)
            {
                new_bytes_vec[i] = memory_controller_.peekByte(buffer_address + i);
            }

            current_delta_.memory_changes.push_back({buffer_address, old_bytes_vec, new_bytes_vec});

            uint64_t old_reg = registers_.ReadGpr(10);
            unsigned int reg_index = 10;
            unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR
            uint64_t new_reg =
                std::min(static_cast<uint64_t>(length), static_cast<uint64_t>(input.size()));
            registers_.WriteGpr(10, new_reg);
            if (old_reg != new_reg)
            {
                current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
            }
        }
        else
        {
            std::cerr << "Unsupported file descriptor: " << file_descriptor << std::endl;
        }
        break;
    }
    case SYSCALL_WRITE:
    { // Write
        uint64_t file_descriptor = registers_.ReadGpr(10);
        uint64_t buffer_address = registers_.ReadGpr(11);
        uint64_t length = registers_.ReadGpr(12);

        if (file_descriptor == 1)
        { // stdout
            std::cout << "VM_STDOUT_START";
            output_status_ = "VM_STDOUT_START";
            uint64_t bytes_printed = 0;
            for (uint64_t i = 0; i < length; ++i)
            {
                char c = memory_controller_.readByte(buffer_address + i);
                // if (c == '\0') {
                //     break;
                // }
                std::cout << c;
                bytes_printed++;
            }
            std::cout << std::flush;
            output_status_ = "VM_STDOUT_END";
            std::cout << "VM_STDOUT_END" << std::endl;

            uint64_t old_reg = registers_.ReadGpr(10);
            unsigned int reg_index = 10;
            unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR
            uint64_t new_reg = std::min(static_cast<uint64_t>(length), bytes_printed);
            registers_.WriteGpr(10, new_reg);
            if (old_reg != new_reg)
            {
                current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
            }
        }
        else
        {
            std::cerr << "Unsupported file descriptor: " << file_descriptor << std::endl;
        }
        break;
    }
    default:
    {
        std::cerr << "Unknown syscall number: " << syscall_number << std::endl;
        break;
    }
    }
}

void RVSSProcessor::WriteVectorMemory()
{
    const uint32_t inst = current_instruction_;

    const uint32_t opcode = inst & 0x7F;
    const uint32_t vreg = (inst >> 7) & 0x1F;
    const uint32_t width = (inst >> 12) & 0x7;
    const uint32_t rs1 = (inst >> 15) & 0x1F;
    const uint32_t lumop = (inst >> 20) & 0x1F;
    const uint32_t vm = (inst >> 25) & 0x1;
    const uint32_t mop = (inst >> 26) & 0x3;
    const uint32_t mew = (inst >> 28) & 0x1;
    const uint32_t nf = (inst >> 29) & 0x7;

    const bool is_load = (opcode == 0x07);
    const bool is_store = (opcode == 0x27);

    if (!is_load && !is_store)
        return;

    if (width != 0b110 || lumop != 0 || vm != 1 || mop != 0 || mew != 0 || nf != 0)
    {
        std::cerr << "Unsupported vector memory addressing mode\n";
        return;
    }

    const uint64_t raw_vtype = registers_.ReadCsr(vconst::kCsrVtype);

    if (!vec::isLegalVtype(raw_vtype))
    {
        std::cerr << "Illegal vector memory instruction: invalid vtype\n";
        return;
    }

    const vec::VType vtype = vec::decodeVtype(raw_vtype);

    if (vtype.vlmul != 0 || vtype.vsew != 2)
    {
        std::cerr << "Vector memory instructions currently require SEW=32, LMUL=1\n";
        return;
    }

    if (registers_.ReadCsr(vconst::kCsrVstart) != 0)
    {
        std::cerr << "Nonzero vstart is not supported yet\n";
        return;
    }

    const uint64_t vl =
        std::min<uint64_t>(registers_.ReadCsr(vconst::kCsrVl), vec::vlmax(raw_vtype));

    const uint64_t base = registers_.ReadGpr(rs1);
    std::cerr << "\n[VEC-MEM]"
          << " opcode=0x" << std::hex << opcode
          << " vreg=v" << std::dec << vreg
          << " base=0x" << std::hex << base
          << " vl=" << std::dec << vl << '\n';
        
    if (is_load)
    {
        const auto old_vd = registers_.ReadVreg(vreg);
        auto new_vd = old_vd;

       
        for (uint64_t i = 0; i < vl; ++i)
        {
            const uint32_t value =
                memory_controller_.readWord(base + i * 4);
            std::cerr << "[VLE32] element=" << i
                      << " address=0x" << std::hex << (base + i * 4)
                      << " value=" << value << std::dec << '\n';

            for (size_t byte = 0; byte < 4; ++byte)
            {
                new_vd[i * 4 + byte] = static_cast<uint8_t>((value >> (byte * 8)) & 0xFF);
            }
        }

        registers_.WriteVreg(vreg, new_vd);
        

        if (old_vd != new_vd)
        {
            current_delta_.register_changes.push_back({vreg, 3, 0, 0, old_vd, new_vd});
        }
    }
    else
    {
        const auto source = registers_.ReadVreg(vreg);

        std::vector<uint8_t> old_bytes_vec;
        std::vector<uint8_t> new_bytes_vec;

        for (uint64_t i = 0; i < vl; ++i)
        {
            const uint64_t addr = base + i * 4;

            uint32_t value = 0;

            for (size_t byte = 0; byte < 4; ++byte)
            {
                value |= static_cast<uint32_t>(source[i * 4 + byte]) << (byte * 8);

                old_bytes_vec.push_back(memory_controller_.peekByte(addr + byte));
            }

            memory_controller_.writeWord(addr, value);
            std::cerr << "[VSE32] element=" << i
          << " address=0x" << std::hex << addr
          << " value=" << value << std::dec << '\n';

            for (size_t byte = 0; byte < 4; ++byte)
            {
                new_bytes_vec.push_back(memory_controller_.peekByte(addr + byte));
            }
        }

        if (old_bytes_vec != new_bytes_vec)
        {
            current_delta_.memory_changes.push_back({base, old_bytes_vec, new_bytes_vec});
        }
    }
}

void RVSSProcessor::WriteMemory()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    if (opcode == 0x07 || opcode == 0x27)
    {
        WriteVectorMemory();
        return;
    }
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;

    if (opcode == 0b1110011 && funct3 == 0b000)
    {
        return;
    }

    if (instruction_set::isFInstruction(current_instruction_))
    { // RV64 F
        WriteMemoryFloat();
        return;
    }
    else if (instruction_set::isDInstruction(current_instruction_))
    {
        WriteMemoryDouble();
        return;
    }

    if (control_unit_.GetMemRead())
    {
        switch (funct3)
        {
        case 0b000:
        { // LB
            memory_result_ = static_cast<int8_t>(memory_controller_.readByte(execution_result_));
            break;
        }
        case 0b001:
        { // LH
            memory_result_ =
                static_cast<int16_t>(memory_controller_.readHalfWord(execution_result_));
            break;
        }
        case 0b010:
        { // LW
            memory_result_ = static_cast<int32_t>(memory_controller_.readWord(execution_result_));
            break;
        }
        case 0b011:
        { // LD
            memory_result_ = memory_controller_.readDoubleWord(execution_result_);
            break;
        }
        case 0b100:
        { // LBU
            memory_result_ = static_cast<uint8_t>(memory_controller_.readByte(execution_result_));
            break;
        }
        case 0b101:
        { // LHU
            memory_result_ =
                static_cast<uint16_t>(memory_controller_.readHalfWord(execution_result_));
            break;
        }
        case 0b110:
        { // LWU
            memory_result_ = static_cast<uint32_t>(memory_controller_.readWord(execution_result_));
            break;
        }
        }
    }

    uint64_t addr = 0;
    std::vector<uint8_t> old_bytes_vec;
    std::vector<uint8_t> new_bytes_vec;

    // Undo bookkeeping reads use peekByte so they do not show up as cache accesses.

    if (control_unit_.GetMemWrite())
    {
        switch (funct3)
        {
        case 0b000:
        { // SB
            addr = execution_result_;
            old_bytes_vec.push_back(memory_controller_.peekByte(addr));
            memory_controller_.writeByte(execution_result_, registers_.ReadGpr(rs2) & 0xFF);
            new_bytes_vec.push_back(memory_controller_.peekByte(addr));
            break;
        }
        case 0b001:
        { // SH
            addr = execution_result_;
            for (size_t i = 0; i < 2; ++i)
            {
                old_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            memory_controller_.writeHalfWord(execution_result_, registers_.ReadGpr(rs2) & 0xFFFF);
            for (size_t i = 0; i < 2; ++i)
            {
                new_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            break;
        }
        case 0b010:
        { // SW
            addr = execution_result_;
            for (size_t i = 0; i < 4; ++i)
            {
                old_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            memory_controller_.writeWord(execution_result_, registers_.ReadGpr(rs2) & 0xFFFFFFFF);
            for (size_t i = 0; i < 4; ++i)
            {
                new_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            break;
        }
        case 0b011:
        { // SD
            addr = execution_result_;
            for (size_t i = 0; i < 8; ++i)
            {
                old_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            memory_controller_.writeDoubleWord(execution_result_,
                                               registers_.ReadGpr(rs2) & 0xFFFFFFFFFFFFFFFF);
            for (size_t i = 0; i < 8; ++i)
            {
                new_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
            }
            break;
        }
        }
    }

    if (old_bytes_vec != new_bytes_vec)
    {
        current_delta_.memory_changes.push_back({addr, old_bytes_vec, new_bytes_vec});
    }
}

void RVSSProcessor::WriteMemoryFloat()
{
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;

    if (control_unit_.GetMemRead())
    { // FLW
        memory_result_ = memory_controller_.readWord(execution_result_);
    }

    uint64_t addr = 0;
    std::vector<uint8_t> old_bytes_vec;
    std::vector<uint8_t> new_bytes_vec;

    if (control_unit_.GetMemWrite())
    { // FSW
        addr = execution_result_;
        for (size_t i = 0; i < 4; ++i)
        {
            old_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
        }
        uint32_t val = registers_.ReadFpr(rs2) & 0xFFFFFFFF;
        memory_controller_.writeWord(execution_result_, val);
        // new_bytes_vec.push_back(memory_controller_.ReadByte(addr));
        for (size_t i = 0; i < 4; ++i)
        {
            new_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
        }
    }

    if (old_bytes_vec != new_bytes_vec)
    {
        current_delta_.memory_changes.push_back({addr, old_bytes_vec, new_bytes_vec});
    }
}

void RVSSProcessor::WriteMemoryDouble()
{
    uint8_t rs2 = (current_instruction_ >> 20) & 0b11111;

    if (control_unit_.GetMemRead())
    { // FLD
        memory_result_ = memory_controller_.readDoubleWord(execution_result_);
    }

    uint64_t addr = 0;
    std::vector<uint8_t> old_bytes_vec;
    std::vector<uint8_t> new_bytes_vec;

    if (control_unit_.GetMemWrite())
    { // FSD
        addr = execution_result_;
        for (size_t i = 0; i < 8; ++i)
        {
            old_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
        }
        memory_controller_.writeDoubleWord(execution_result_, registers_.ReadFpr(rs2));
        for (size_t i = 0; i < 8; ++i)
        {
            new_bytes_vec.push_back(memory_controller_.peekByte(addr + i));
        }
    }

    if (old_bytes_vec != new_bytes_vec)
    {
        current_delta_.memory_changes.push_back({addr, old_bytes_vec, new_bytes_vec});
    }
}

void RVSSProcessor::WriteBack()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;
    uint8_t rd = (current_instruction_ >> 7) & 0b11111;
    int32_t imm = ImmGenerator(current_instruction_);

    if (opcode == 0b1110011 && funct3 == 0b000)
    {
        return;
    }

    if (instruction_set::isFInstruction(current_instruction_))
    { // RV64 F
        WriteBackFloat();
        return;
    }
    else if (instruction_set::isDInstruction(current_instruction_))
    {
        WriteBackDouble();
        return;
    }
    else if (opcode == 0b1110011)
    { // CSR opcode
        WriteBackCsr();
        return;
    }

    uint64_t old_reg = registers_.ReadGpr(rd);
    unsigned int reg_index = rd;
    unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR

    if (control_unit_.GetRegWrite())
    {
        switch (opcode)
        {
        case 0b0110011: // R-Type
        case 0b0010011: // I-Type
        case 0b0010111: // AUIPC
        case 0b0111011: // R4-Type / OP-32 (ADDW, SUBW, MULW, DIVW, DIVUW, REMW, REMUW, SLLW, SRLW,
                        // SRAW)
        case 0b0011011: // I-type OP-32 (ADDIW, SLLIW, SRLIW, SRAIW)
        {
            registers_.WriteGpr(rd, execution_result_);
            break;
        }
        case 0b0000011:
        { // Load
            registers_.WriteGpr(rd, memory_result_);
            break;
        }
        case 0b1100111: // JALR
        case 0b1101111:
        { // JAL
            registers_.WriteGpr(rd, next_pc_);
            break;
        }
        case 0b0110111:
        { // LUI
            registers_.WriteGpr(rd, (imm << 12));
            break;
        }
        default:
            break;
        }
    }

    if (opcode == 0b1101111)
    { // JAL
      // Updated in Execute()
    }
    if (opcode == 0b1100111)
    { // JALR
      // registers_.WriteGpr(rd, return_address_); // Write back to rs1
      // Updated in Execute()
    }

    uint64_t new_reg = registers_.ReadGpr(rd);
    if (old_reg != new_reg)
    {
        current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
    }
}
void RVSSProcessor::ExecuteVsetvli()
{
    const uint8_t rd = (current_instruction_ >> 7) & 0x1F;
    const uint8_t rs1 = (current_instruction_ >> 15) & 0x1F;
    const uint64_t zimm = (current_instruction_ >> 20) & 0x7FF;

    const uint64_t old_vtype = registers_.ReadCsr(vconst::kCsrVtype);
    const uint64_t old_vl = registers_.ReadCsr(vconst::kCsrVl);
    const uint64_t old_rd = registers_.ReadGpr(rd);

    uint64_t new_vtype = zimm;
    uint64_t new_vl = 0;

    if (!vec::isLegalVtype(zimm))
    {
        new_vtype = vconst::kVtypeVill; // unsupported request: vill set, vl = 0
        new_vl = 0;
    }
    else if (rs1 != 0)
    {
        new_vl = std::min<uint64_t>(registers_.ReadGpr(rs1), vec::vlmax(zimm));
    }
    else if (rd != 0)
    {
        new_vl = vec::vlmax(zimm); // rs1 = x0, rd != x0: request VLMAX
    }
    else if (vec::isLegalVtype(old_vtype) && vec::vlmax(old_vtype) == vec::vlmax(zimm))
    {
        new_vl = old_vl; // rd = rs1 = x0: keep vl, change type only
    }
    else
    {
        new_vtype = vconst::kVtypeVill; // that would change VLMAX: not allowed
        new_vl = 0;
    }

    registers_.WriteCsr(vconst::kCsrVtype, new_vtype);
    registers_.WriteCsr(vconst::kCsrVl, new_vl);

    if (old_vtype != new_vtype)
        current_delta_.register_changes.push_back({vconst::kCsrVtype, 1, old_vtype, new_vtype});
    if (old_vl != new_vl)
        current_delta_.register_changes.push_back({vconst::kCsrVl, 1, old_vl, new_vl});

    if (rd != 0)
    {
        registers_.WriteGpr(rd, new_vl);
        if (old_rd != new_vl)
            current_delta_.register_changes.push_back({rd, 0, old_rd, new_vl});
    }
}
void RVSSProcessor::ExecuteVector()
{
    const uint32_t inst = current_instruction_;
    const uint8_t opcode = inst & 0x7F;
    const uint8_t vd = (inst >> 7) & 0x1F;
    const uint8_t funct3 = (inst >> 12) & 0x7;
    const uint8_t vs1 = (inst >> 15) & 0x1F; // also rs1 (OPIVX) or simm5 (OPIVI)
    const uint8_t vs2 = (inst >> 20) & 0x1F;
    const uint8_t vm = (inst >> 25) & 0x1;
    const uint8_t funct6 = (inst >> 26) & 0x3F;

    enum class Op
    {
        VaddVV, VsubVV, VmulVV,
        VandVV, VorVV, VxorVV,

        VminVV, VmaxVV, VminuVV, VmaxuVV,

        VmseqVV, VmsneVV, VmsltVV, VmsltuVV,
        VmsleVV, VmsleuVV,

        VfaddVV, VfsubVV, VfmulVV, VfdivVV,

        VredsumVS, VredandVS, VredorVS, VredxorVS,
        VredminVS, VredmaxVS, VredminuVS, VredmaxuVS,

        VaddVX, VsubVX, VrsubVX, VmulVX,
        VandVX, VorVX, VxorVX,

        VmvVX, VmvVI,

        Unsupported
    } op = Op::Unsupported;

    if (opcode == 0x57 && funct3 == 0b000)
    {
        if      (funct6 == 0b000000) op = Op::VaddVV;
        else if (funct6 == 0b000010) op = Op::VsubVV;
        else if (funct6 == 0b001001) op = Op::VandVV;
        else if (funct6 == 0b001010) op = Op::VorVV;
        else if (funct6 == 0b001011) op = Op::VxorVV;

        else if (funct6 == 0b000101) op = Op::VminVV;
        else if (funct6 == 0b000111) op = Op::VmaxVV;
        else if (funct6 == 0b000100) op = Op::VminuVV;
        else if (funct6 == 0b000110) op = Op::VmaxuVV;

        else if (funct6 == 0b011000) op = Op::VmseqVV;
        else if (funct6 == 0b011001) op = Op::VmsneVV;
        else if (funct6 == 0b011010) op = Op::VmsltuVV;
        else if (funct6 == 0b011011) op = Op::VmsltVV;
        else if (funct6 == 0b011100) op = Op::VmsleuVV;
        else if (funct6 == 0b011101) op = Op::VmsleVV;
    }
    else if (opcode == 0x57 && funct3 == 0b001)
    {
        if      (funct6 == 0b000000) op = Op::VfaddVV;
        else if (funct6 == 0b000010) op = Op::VfsubVV;
        else if (funct6 == 0b100100) op = Op::VfmulVV;
        else if (funct6 == 0b100000) op = Op::VfdivVV;
    }
    else if (opcode == 0x57 && funct3 == 0b010)
    {
        if      (funct6 == 0b100101) op = Op::VmulVV;
        else if (funct6 == 0b000000) op = Op::VredsumVS;
        else if (funct6 == 0b000001) op = Op::VredandVS;
        else if (funct6 == 0b000010) op = Op::VredorVS;
        else if (funct6 == 0b000011) op = Op::VredxorVS;
        else if (funct6 == 0b000100) op = Op::VredminuVS;
        else if (funct6 == 0b000101) op = Op::VredminVS;
        else if (funct6 == 0b000110) op = Op::VredmaxuVS;
        else if (funct6 == 0b000111) op = Op::VredmaxVS;
    }
    else if (opcode == 0x57 && funct3 == 0b100)
    {
        if      (funct6 == 0b010111 && vs2 == 0) op = Op::VmvVX;
        else if (funct6 == 0b000000) op = Op::VaddVX;
        else if (funct6 == 0b000010) op = Op::VsubVX;
        else if (funct6 == 0b000011) op = Op::VrsubVX;
        else if (funct6 == 0b100101) op = Op::VmulVX;
        else if (funct6 == 0b001001) op = Op::VandVX;
        else if (funct6 == 0b001010) op = Op::VorVX;
        else if (funct6 == 0b001011) op = Op::VxorVX;
    }
    else if (opcode == 0x57 && funct6 == 0b010111 &&
             vs2 == 0 && funct3 == 0b011)
    {
        op = Op::VmvVI;
    }

    if (op == Op::Unsupported)
    {
        std::cerr << "Unsupported vector instruction\n";
        return;
    }
    if (vm != 1) // vm = 0 is masking (or vmerge for funct6 010111)
    {
        std::cerr << "Masked vector instructions are not supported yet\n";
        return;
    }

    const uint64_t raw_vtype = registers_.ReadCsr(vconst::kCsrVtype);
    if (!vec::isLegalVtype(raw_vtype)) // also true when vill is set
    {
        std::cerr << "Illegal instruction: vtype is invalid (vill)\n";
        return;
    }
    const vec::VType vtype = vec::decodeVtype(raw_vtype);
    if (vtype.vlmul != 0)
    {
        std::cerr << "Only LMUL=1 is supported yet\n";
        return;
    }
    if ((op == Op::VfaddVV || op == Op::VfsubVV || op == Op::VfmulVV || op == Op::VfdivVV ) &&
    vec::sewBits(vtype.vsew) != 32)
    {
        std::cerr << "vfadd.vv and vfsub.vv currently support only SEW=32 (FP32)\n";
        return;
    }

    const size_t sew_bytes = vec::sewBits(vtype.vsew) / 8;
    const uint64_t vl =
        std::min<uint64_t>(registers_.ReadCsr(vconst::kCsrVl), vec::vlmax(raw_vtype));

    uint64_t splat = 0; // value for the vmv forms; low SEW bits are stored
    if (op == Op::VmvVX)
        splat = registers_.ReadGpr(vs1);
    else if (op == Op::VmvVI)
        splat = static_cast<uint64_t>(static_cast<int64_t>(vs1 ^ 0x10) - 0x10); // sign-extend simm5

    const auto old_vd = registers_.ReadVreg(vd);
        auto new_vd = old_vd;

    const bool isReduction =
        op == Op::VredsumVS ||
        op == Op::VredandVS ||
        op == Op::VredorVS ||
        op == Op::VredxorVS ||
        op == Op::VredminVS ||
        op == Op::VredmaxVS ||
        op == Op::VredminuVS ||
        op == Op::VredmaxuVS;

    const bool isComparison =
        op == Op::VmseqVV ||
        op == Op::VmsneVV ||
        op == Op::VmsltVV ||
        op == Op::VmsltuVV ||
        op == Op::VmsleVV ||
        op == Op::VmsleuVV;

    if (isReduction)
    {
        if (registers_.ReadCsr(vconst::kCsrVstart) != 0)
        {
            std::cerr << "Nonzero vstart is not supported for reductions\n";
            return;
        }

        uint64_t value =
            registers_.ReadVectorElement(vs1, sew_bytes, 0);

        for (uint64_t i = 0; i < vl; ++i)
        {
            const uint64_t element =
                registers_.ReadVectorElement(vs2, sew_bytes, i);

            switch (op)
            {
            case Op::VredsumVS:
                value += element;
                break;

            case Op::VredandVS:
                value &= element;
                break;

            case Op::VredorVS:
                value |= element;
                break;

            case Op::VredxorVS:
                value ^= element;
                break;

            case Op::VredminVS:
            case Op::VredmaxVS:
            {
                const unsigned sew_bits = sew_bytes * 8;
                const uint64_t sign_bit = uint64_t{1} << (sew_bits - 1);

                const int64_t signed_value =
                    static_cast<int64_t>((value ^ sign_bit) - sign_bit);
                const int64_t signed_element =
                    static_cast<int64_t>((element ^ sign_bit) - sign_bit);

                if (op == Op::VredminVS && signed_value > signed_element)
                    value = element;

                if (op == Op::VredmaxVS && signed_value < signed_element)
                    value = element;

                break;
            }

            case Op::VredminuVS:
                value = std::min(value, element);
                break;

            case Op::VredmaxuVS:
                value = std::max(value, element);
                break;

            default:
                break;
            }
        }

        for (size_t k = 0; k < sew_bytes; ++k)
            new_vd[k] = static_cast<uint8_t>((value >> (8 * k)) & 0xFF);
    }
    else if (isComparison)
    {
        const uint64_t sign_bit =
            uint64_t{1} << (sew_bytes * 8 - 1);

        for (uint64_t i = 0; i < vl; ++i)
        {
            const uint64_t lhs =
                registers_.ReadVectorElement(vs2, sew_bytes, i);
            const uint64_t rhs =
                registers_.ReadVectorElement(vs1, sew_bytes, i);

            bool result = false;

            switch (op)
            {
            case Op::VmseqVV:
                result = (lhs == rhs);
                break;

            case Op::VmsneVV:
                result = (lhs != rhs);
                break;

            case Op::VmsltVV:
                result = ((lhs ^ sign_bit) < (rhs ^ sign_bit));
                break;

            case Op::VmsltuVV:
                result = (lhs < rhs);
                break;

            case Op::VmsleVV:
                result = ((lhs ^ sign_bit) <= (rhs ^ sign_bit));
                break;

            case Op::VmsleuVV:
                result = (lhs <= rhs);
                break;

            default:
                break;
            }

            const size_t byte_index = i / 8;
            const uint8_t bit_mask = uint8_t{1} << (i % 8);

            if (result)
                new_vd[byte_index] |= bit_mask;
            else
                new_vd[byte_index] &= static_cast<uint8_t>(~bit_mask);
        }
    }
    else if (op == Op::VfaddVV || op == Op::VfsubVV || op == Op::VfmulVV || op == Op::VfdivVV)
    {
        uint8_t rm = registers_.ReadCsr(0x002); // frm

        for (uint64_t i = 0; i < vl; ++i)
        {
            const uint32_t lhs = static_cast<uint32_t>(
                registers_.ReadVectorElement(vs2, 4, i));
            const uint32_t rhs = static_cast<uint32_t>(
                registers_.ReadVectorElement(vs1, 4, i));

            const auto [result, fflags] =
                alu::Alu::fpexecute(
                    op == Op::VfaddVV ? alu::AluOp::FADD_S :
                    op == Op::VfsubVV ? alu::AluOp::FSUB_S :
                    op == Op::VfmulVV ? alu::AluOp::FMUL_S :
                    alu::AluOp::FDIV_S,
                    lhs, rhs, 0, rm);

            const uint32_t value = static_cast<uint32_t>(result);

            for (size_t k = 0; k < 4; ++k)
                new_vd[i * 4 + k] =
                    static_cast<uint8_t>((value >> (8 * k)) & 0xFF);

            // Accumulate FP exception flags (fflags, CSR 0x001).
            const uint8_t old_fflags = registers_.ReadCsr(0x001);
            registers_.WriteCsr(0x001, old_fflags | fflags);
        }
    }
    else
    {
        for (uint64_t i = 0; i < vl; ++i)
        {
            uint64_t value = 0;

            if (op == Op::VmvVX || op == Op::VmvVI)
            {
                value = splat;
            }
            else
            {
                const uint64_t lhs =
                    registers_.ReadVectorElement(vs2, sew_bytes, i);

                if (op == Op::VaddVX || op == Op::VsubVX ||
                    op == Op::VrsubVX || op == Op::VmulVX ||
                    op == Op::VandVX || op == Op::VorVX ||
                    op == Op::VxorVX)
                {
                    const uint64_t scalar = registers_.ReadGpr(vs1);

                    switch (op)
                    {
                    case Op::VaddVX:  value = lhs + scalar; break;
                    case Op::VsubVX:  value = lhs - scalar; break;
                    case Op::VrsubVX: value = scalar - lhs; break;
                    case Op::VmulVX:  value = lhs * scalar; break;
                    case Op::VandVX:  value = lhs & scalar; break;
                    case Op::VorVX:   value = lhs | scalar; break;
                    case Op::VxorVX:  value = lhs ^ scalar; break;
                    default: break;
                    }
                }
                else
                {
                    const uint64_t rhs =
                        registers_.ReadVectorElement(vs1, sew_bytes, i);

                    switch (op)
                    {
                    case Op::VaddVV: value = lhs + rhs; break;
                    case Op::VsubVV: value = lhs - rhs; break;
                    case Op::VmulVV: value = lhs * rhs; break;
                    case Op::VandVV: value = lhs & rhs; break;
                    case Op::VorVV:  value = lhs | rhs; break;
                    case Op::VxorVV: value = lhs ^ rhs; break;

                    case Op::VminVV:
                    {
                        const uint64_t sign_bit =
                            uint64_t{1} << (sew_bytes * 8 - 1);
                        value = ((lhs ^ sign_bit) < (rhs ^ sign_bit))
                                    ? lhs : rhs;
                        break;
                    }

                    case Op::VmaxVV:
                    {
                        const uint64_t sign_bit =
                            uint64_t{1} << (sew_bytes * 8 - 1);
                        value = ((lhs ^ sign_bit) > (rhs ^ sign_bit))
                                    ? lhs : rhs;
                        break;
                    }

                    case Op::VminuVV:
                        value = (lhs < rhs) ? lhs : rhs;
                        break;

                    case Op::VmaxuVV:
                        value = (lhs > rhs) ? lhs : rhs;
                        break;

                    default:
                        std::cerr << "Invalid vector arithmetic operation\n";
                        return;
                    }
                }
            }

            for (size_t k = 0; k < sew_bytes; ++k)
                new_vd[i * sew_bytes + k] =
                    static_cast<uint8_t>((value >> (8 * k)) & 0xFF);
        }
    }

    if (old_vd != new_vd)
    {
        registers_.WriteVreg(vd, new_vd); // single write, single UI refresh
        current_delta_.register_changes.push_back({vd, 3, 0, 0, old_vd, new_vd});
    }
}
void RVSSProcessor::WriteBackFloat()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
    uint8_t rd = (current_instruction_ >> 7) & 0b11111;

    uint64_t old_reg = 0;
    unsigned int reg_index = rd;
    unsigned int reg_type = 2; // 0 for GPR, 1 for CSR, 2 for FPR
    uint64_t new_reg = 0;

    if (control_unit_.GetRegWrite())
    {
        // write to GPR
        if (funct7 == 0b1010000 || funct7 == 0b1100000 || funct7 == 0b1110000)
        { // f(eq|lt|le).s, fcvt.(w|wu|l|lu).s
            old_reg = registers_.ReadGpr(rd);
            registers_.WriteGpr(rd, execution_result_);
            new_reg = execution_result_;
            reg_type = 0; // GPR
        }
        // write to FPR
        else if (opcode == 0b0000111)
        {
            old_reg = registers_.ReadFpr(rd);
            registers_.WriteFpr(rd, memory_result_);
            new_reg = memory_result_;
            reg_type = 2; // FPR
        }
        else
        {
            old_reg = registers_.ReadFpr(rd);
            registers_.WriteFpr(rd, execution_result_);
            new_reg = execution_result_;
            reg_type = 2; // FPR
        }
    }

    if (old_reg != new_reg)
    {
        current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
    }
}

void RVSSProcessor::WriteBackDouble()
{
    uint8_t opcode = current_instruction_ & 0b1111111;
    uint8_t funct7 = (current_instruction_ >> 25) & 0b1111111;
    uint8_t rd = (current_instruction_ >> 7) & 0b11111;

    uint64_t old_reg = 0;
    unsigned int reg_index = rd;
    unsigned int reg_type = 2; // 0 for GPR, 1 for CSR, 2 for FPR
    uint64_t new_reg = 0;

    if (control_unit_.GetRegWrite())
    {
        // write to GPR
        if (funct7 == 0b1010001 || funct7 == 0b1100001 || funct7 == 0b1110001)
        { // f(eq|lt|le).d, fcvt.(w|wu|l|lu).d
            old_reg = registers_.ReadGpr(rd);
            registers_.WriteGpr(rd, execution_result_);
            new_reg = execution_result_;
            reg_type = 0; // GPR
        }
        // write to FPR
        else if (opcode == 0b0000111)
        {
            old_reg = registers_.ReadFpr(rd);
            registers_.WriteFpr(rd, memory_result_);
            new_reg = memory_result_;
            reg_type = 2; // FPR
        }
        else
        {
            old_reg = registers_.ReadFpr(rd);
            registers_.WriteFpr(rd, execution_result_);
            new_reg = execution_result_;
            reg_type = 2; // FPR
        }
    }

    if (old_reg != new_reg)
    {
        current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
    }

    return;
}

void RVSSProcessor::WriteBackCsr()
{
    uint8_t rd = (current_instruction_ >> 7) & 0b11111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;

    uint64_t old_gpr = registers_.ReadGpr(rd);
    uint64_t old_csr = registers_.ReadCsr(csr_target_address_);

    switch (funct3)
    {
    case 0b001:
    { // CSRRW
        registers_.WriteGpr(rd, csr_old_value_);
        registers_.WriteCsr(csr_target_address_, csr_write_val_);
        break;
    }
    case 0b010:
    { // CSRRS
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_write_val_ != 0)
        {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ | csr_write_val_);
        }
        break;
    }
    case 0b011:
    { // CSRRC
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_write_val_ != 0)
        {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ & ~csr_write_val_);
        }
        break;
    }
    case 0b101:
    { // CSRRWI
        registers_.WriteGpr(rd, csr_old_value_);
        registers_.WriteCsr(csr_target_address_, csr_uimm_);
        break;
    }
    case 0b110:
    { // CSRRSI
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_uimm_ != 0)
        {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ | csr_uimm_);
        }
        break;
    }
    case 0b111:
    { // CSRRCI
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_uimm_ != 0)
        {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ & ~csr_uimm_);
        }
        break;
    }
    }

    uint64_t new_gpr = registers_.ReadGpr(rd);
    uint64_t new_csr = registers_.ReadCsr(csr_target_address_);

    if (old_gpr != new_gpr)
    {
        current_delta_.register_changes.push_back({rd, 0, old_gpr, new_gpr});
    }
    if (old_csr != new_csr)
    {
        current_delta_.register_changes.push_back({csr_target_address_, 1, old_csr, new_csr});
    }
}

void RVSSProcessor::DebugRun()
{
    ClearStop();
    while (!stop_requested_ && program_counter_ < program_size_)
    {
        current_delta_.old_pc = program_counter_;
        if (std::find(breakpoints_.begin(), breakpoints_.end(), program_counter_) ==
            breakpoints_.end())
        {
            Fetch();
            Decode();
            Execute();
            WriteMemory();
            WriteBack();
            instructions_retired_++;
            cycle_s_++;
            std::cout << "Program Counter: " << program_counter_ << std::endl;

            current_delta_.new_pc = program_counter_;
            // history_.push(current_delta_);
            undo_stack_.push(current_delta_);
            while (!redo_stack_.empty())
            {
                redo_stack_.pop();
            }
            current_delta_ = RVSingleStageStepDelta();
            if (program_counter_ < program_size_)
            {
                std::cout << "VM_STEP_COMPLETED" << std::endl;
                output_status_ = "VM_STEP_COMPLETED";
            }
            else if (program_counter_ >= program_size_)
            {
                std::cout << "VM_LAST_INSTRUCTION_STEPPED" << std::endl;
                output_status_ = "VM_LAST_INSTRUCTION_STEPPED";
            }
            DumpRegisters(globals::registers_dump_file_path, registers_);
            DumpState(globals::vm_state_dump_file_path);
            // update circuit UI after the debug step
            SetActiveWireNames();
            emit updateCircuitStateSignal(active_wires_);

            // unsigned int delay_ms = vm_config::config.getRunStepDelay();
            //  std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
        else
        {
            std::cout << "VM_BREAKPOINT_HIT " << program_counter_ << std::endl;
            output_status_ = "VM_BREAKPOINT_HIT";
            break;
        }
    }
    if (program_counter_ >= program_size_)
    {
        std::cout << "VM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RVSSProcessor::Step()
{
    current_delta_.old_pc = program_counter_;
    if (program_counter_ < program_size_)
    {
        Fetch();
        Decode();
        Execute();
        WriteMemory();
        WriteBack();
        instructions_retired_++;
        cycle_s_++;
        std::cout << "Program Counter: " << std::hex << program_counter_ << std::dec << std::endl;

        current_delta_.new_pc = program_counter_;

        // history_.push(current_delta_);

        undo_stack_.push(current_delta_);
        while (!redo_stack_.empty())
        {
            redo_stack_.pop();
        }

        current_delta_ = RVSingleStageStepDelta();

        if (program_counter_ < program_size_)
        {
            std::cout << "VM_STEP_COMPLETED" << std::endl;
            output_status_ = "VM_STEP_COMPLETED";
        }
        else if (program_counter_ >= program_size_)
        {
            std::cout << "VM_LAST_INSTRUCTION_STEPPED" << std::endl;
            output_status_ = "VM_LAST_INSTRUCTION_STEPPED";
        }
    }
    else if (program_counter_ >= program_size_)
    {
        std::cout << "VM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

bool RVSSProcessor::StepAndNotify()
{
    QMutexLocker locker(&step_mutex_);
    if (program_counter_ >= program_size_)
    {
        return false;
    }
    last_executed_pc_ = program_counter_;
    Step();
    SetActiveWireNames();
    setProcessorState();
    emit processorClockedSignal(processor_state_);
    emit updateCircuitStateSignal(active_wires_);
    active_wires_.erase(active_wires_.begin() + always_active_wires_count_, active_wires_.end());
    // clear active wires after emitting signal except always active wires
    return program_counter_ < program_size_;
}

void RVSSProcessor::Undo()
{
    qInfo() << "Attempting to undo last step in rvss";
    if (undo_stack_.empty())
    {
        std::cout << "VM_NO_MORE_UNDO" << std::endl;
        output_status_ = "VM_NO_MORE_UNDO";
        return;
    }

    StepDelta last = undo_stack_.top();
    undo_stack_.pop();

    for (const auto &change : last.register_changes)
    {
        switch (change.reg_type)
        {
        case 0:
        { // GPR
            registers_.WriteGpr(change.reg_index, change.old_value);
            break;
        }
        case 1:
        { // CSR
            registers_.WriteCsr(change.reg_index, change.old_value);
            break;
        }
        case 2:
        { // FPR
            registers_.WriteFpr(change.reg_index, change.old_value);
            break;
        }
        case 3:
        { // VECTOR
            registers_.WriteVreg(change.reg_index, change.old_vector);
            break;
        }
        default:
            std::cerr << "Invalid register type: " << change.reg_type << std::endl;
            break;
        }
    }

    for (const auto &change : last.memory_changes)
    {
        for (size_t i = 0; i < change.old_bytes_vec.size(); ++i)
        {
            memory_controller_.writeByte(change.address + i, change.old_bytes_vec[i]);
        }
    }

    program_counter_ = last.old_pc;
    instructions_retired_--;
    cycle_s_--;
    std::cout << "Program Counter: " << program_counter_ << std::endl;

    redo_stack_.push(last);

    output_status_ = "VM_UNDO_COMPLETED";
    std::cout << "VM_UNDO_COMPLETED" << std::endl;

    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);

    SetActiveWireNames();
    setProcessorState();
    emit updateCircuitStateSignal(active_wires_);
    emit processorClockedSignal(processor_state_);
    qInfo() << "Undo completed in rvss";
}

void RVSSProcessor::Redo()
{
    qInfo() << "Attempting to redo last undone step in rvss";
    if (redo_stack_.empty())
    {
        std::cout << "VM_NO_MORE_REDO" << std::endl;
        return;
    }

    StepDelta next = redo_stack_.top();
    redo_stack_.pop();

    // if (!history_.can_redo()) {
    //       std::cout << "Nothing to redo.\n";
    //       return;
    //   }

    //   StepDelta next = history_.redo();

    for (const auto &change : next.register_changes)
    {
        switch (change.reg_type)
        {
        case 0:
        { // GPR
            registers_.WriteGpr(change.reg_index, change.new_value);
            break;
        }
        case 1:
        { // CSR
            registers_.WriteCsr(change.reg_index, change.new_value);
            break;
        }
        case 2:
        { // FPR
            registers_.WriteFpr(change.reg_index, change.new_value);
            break;
        }
        case 3:
        { // VECTOR
            registers_.WriteVreg(change.reg_index, change.new_vector);
            break;
        }
        default:
            std::cerr << "Invalid register type: " << change.reg_type << std::endl;
            break;
        }
    }

    for (const auto &change : next.memory_changes)
    {
        for (size_t i = 0; i < change.new_bytes_vec.size(); ++i)
        {
            memory_controller_.writeByte(change.address + i, change.new_bytes_vec[i]);
        }
    }

    program_counter_ = next.new_pc;
    instructions_retired_++;
    cycle_s_++;
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
    std::cout << "Program Counter: " << program_counter_ << std::endl;
    undo_stack_.push(next);

    SetActiveWireNames();
    setProcessorState();
    emit updateCircuitStateSignal(active_wires_);
    emit processorClockedSignal(processor_state_);
}

void RVSSProcessor::Reset()
{
    program_counter_ = 0;
    instructions_retired_ = 0;
    cycle_s_ = 0;
    last_breakpoint_pc_ = UINT64_MAX; // Clear breakpoint tracking on reset
    registers_.Reset();
    memory_controller_.reset();
    control_unit_.Reset();
    branch_flag_ = false;
    next_pc_ = 0;
    execution_result_ = 0;
    memory_result_ = 0;

    return_address_ = 0;
    csr_target_address_ = 0;
    csr_old_value_ = 0;
    csr_write_val_ = 0;
    csr_uimm_ = 0;
    current_delta_.register_changes.clear();
    current_delta_.memory_changes.clear();
    current_delta_.old_pc = 0;
    current_delta_.new_pc = 0;
    undo_stack_ = std::stack<StepDelta>();
    redo_stack_ = std::stack<StepDelta>();
}
} // namespace Kites
