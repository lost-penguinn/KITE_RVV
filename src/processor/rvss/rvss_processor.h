/**
 * @file rvss_vm.h
 * @brief RVSS VM definition
 * @author Vishank Singh, https://github.com/VishankSingh
 */
#ifndef RVSS_VM_H
#define RVSS_VM_H

#include "processor/processor_base.h"
#include "processor/processor_manager.h"
#include "rvss_control_unit.h"

#include <cstdint>
#include <iostream>
#include <stack>
#include <vector>


// TODO: use a circular buffer instead of a stack for undo/redo
namespace Kites
{
struct RVSingleStageStepDelta : public StepDelta
{
    // uint64_t old_pc;
    // uint64_t new_pc;
    // std::vector<RegisterChange> register_changes;
    // std::vector<MemoryChange> memory_changes;
};

class RVSSProcessor : public ProcessorBase
{
  public:
    RVSSControlUnit control_unit_;
    // intermediate variables
    int64_t execution_result_{};
    int64_t memory_result_{};
    // int64_t memory_address_{};
    // int64_t memory_data_{};
    uint64_t return_address_{};

    bool branch_flag_ = false;
    int64_t next_pc_{}; // for jal, jalr,

    // CSR intermediate variables
    uint16_t csr_target_address_{};
    uint64_t csr_old_value_{};
    uint64_t csr_write_val_{};
    uint8_t csr_uimm_{};

    std::stack<StepDelta> undo_stack_;
    std::stack<StepDelta> redo_stack_;

    RVSingleStageStepDelta current_delta_;

    void Fetch();

    // RVV profiling counters
    uint64_t rvv_instructions_{};
    uint64_t rvv_arithmetic_instructions_{};
    uint64_t rvv_logical_instructions_{};
    uint64_t rvv_reduction_instructions_{};
    uint64_t rvv_move_instructions_{};
    uint64_t rvv_load_instructions_{};
    uint64_t rvv_store_instructions_{};

    uint64_t rvv_elements_processed_{};
    uint64_t rvv_memory_bytes_read_{};
    uint64_t rvv_memory_bytes_written_{};
    uint64_t rvv_execution_time_ns_{};
    uint64_t rvv_memory_time_ns_{};
    uint64_t rvv_peek_time_ns_{};
    uint64_t rvv_write_time_ns_{};
    uint64_t rvv_undo_time_ns_{};

        // RVV profiling getters
    uint64_t getRvvInstructions() const { return rvv_instructions_; }
    uint64_t getRvvArithmeticInstructions() const { return rvv_arithmetic_instructions_; }
    uint64_t getRvvLogicalInstructions() const { return rvv_logical_instructions_; }
    uint64_t getRvvReductionInstructions() const { return rvv_reduction_instructions_; }
    uint64_t getRvvMoveInstructions() const { return rvv_move_instructions_; }
    uint64_t getRvvLoadInstructions() const { return rvv_load_instructions_; }
    uint64_t getRvvStoreInstructions() const { return rvv_store_instructions_; }

    uint64_t getRvvElementsProcessed() const { return rvv_elements_processed_; }
    uint64_t getRvvMemoryBytesRead() const { return rvv_memory_bytes_read_; }
    uint64_t getRvvMemoryBytesWritten() const { return rvv_memory_bytes_written_; }
    uint64_t getRvvExecutionTimeNs() const  {    return rvv_execution_time_ns_; }
    uint64_t getRvvMemoryTimeNs() const    {    return rvv_memory_time_ns_; }
    uint64_t getRvvPeekTimeNs() const { return rvv_peek_time_ns_; }
    uint64_t getRvvWriteTimeNs() const { return rvv_write_time_ns_; }
    uint64_t getRvvUndoTimeNs() const { return rvv_undo_time_ns_; }
    uint64_t getScalarSdWriteTimeNs() const { return scalar_sd_write_time_ns_; }
    uint64_t getScalarSdTotalTimeNs() const { return scalar_sd_total_time_ns_; }
    uint64_t scalar_sd_write_time_ns_{};
    uint64_t scalar_sd_total_time_ns_{};

    void Decode();

    void Execute();
    void ExecuteFloat();
    void ExecuteDouble();
    void ExecuteCsr();
    void ExecuteVector();
    void ExecuteStrassenMatmul();
    void ExecuteVsetvli();
    void HandleSyscall();
    void WriteMemory();
    void WriteVectorMemory();
    void WriteMemoryFloat();
    void WriteMemoryDouble();

    void WriteBack();
    void WriteBackFloat();
    void WriteBackDouble();
    void WriteBackCsr();

    RVSSProcessor();
    ~RVSSProcessor();

    void Run() override;
    void DebugRun() override;
    void Step() override;
    bool StepAndNotify() override;
    void Undo() override;
    void Redo() override;
    void Reset() override;


    void SetActiveWireNames() override;
    void setProcessorState() override;

    void PrintType()
    {
        std::cout << "rvssvm" << std::endl;
    }
};
}//namespace Kites
#endif // RVSS_VM_H
