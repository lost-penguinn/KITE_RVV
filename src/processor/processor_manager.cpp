/**
 * @file vm_manager.cpp
 * @brief This file contains the implementation of the VMManager class
 * @version 0.1
 * @date 2025-10-23
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <iostream>
#include "processor/processor_factory.h"
#include "processor/rv5s/rv5s_processor_h_f.h"
#include "processor/rv5s/rv5s_processor_h_nf.h"
#include "processor/rv5s/rv5s_processor_nh_f.h"
#include "processor/rv5s/rv5s_processor_nh_nf.h"
#include "processor/rvss/rvss_processor.h"
#include "processor/processor_manager.h"
#include "common/globals.h"
#include "common/assembled_program.h"
#include "assembler/assembler.h"
#include "utils/utils.h"

namespace Kites
{
ProcessorManager::ProcessorManager(QObject *parent, ProcessorType vmType) : QObject(parent)
{
    // first we register all the VMs
    // m_profiler = std::make_unique<Profiler>();

    ProcessorFactory::RegisterVM<RVSSProcessor>(ProcessorType::RVSS);
    ProcessorFactory::RegisterVM<RV5StageProcessorNHNF>(ProcessorType::RV5Stage_NH_NF);
    ProcessorFactory::RegisterVM<RV5StageProcessorHNF>(ProcessorType::RV5Stage_H_NF);
    ProcessorFactory::RegisterVM<RV5StageProcessorNHF>(ProcessorType::RV5Stage_NH_F);
    ProcessorFactory::RegisterVM<RV5StageProcessorHF>(ProcessorType::RV5Stage_H_F);
    m_currentProcessorType = vmType;
    m_currentProcessor = ProcessorFactory::createVM(vmType);
    connect(m_currentProcessor.get(), &ProcessorBase::processorClockedSignal, this,
            &ProcessorManager::processorStateChangedSignal, Qt::DirectConnection);
    connect(m_currentProcessor.get(), &ProcessorBase::processorPausedAtBreakpointSignal, this,
            &ProcessorManager::processorPausedAtBreakpointSignal, Qt::DirectConnection);
    // here we connect the vm state changed signal to the vm manager signal
    // and this will be further connected to the mainwindow slot to update the ui
    connect(m_currentProcessor.get(), &ProcessorBase::processorClockedSignal, this,
            &ProcessorManager::processorClockedSlot, Qt::DirectConnection);
    connect(m_currentProcessor.get(), &ProcessorBase::processorClockedSignal, this,
            &ProcessorManager::processorClockedSignal, Qt::DirectConnection);

    connect(this, &ProcessorManager::processorClockedSignal,&m_profiler, 
            &Profiler::processorClockedSlot, Qt::DirectConnection);
}

void ProcessorManager::changeProcessor(ProcessorType vmType)
{
    m_currentProcessorType = vmType;
    m_currentProcessor = ProcessorFactory::createVM(vmType);
    m_currentProcessor->step_delay_ = m_stepDelayMs;
    connect(m_currentProcessor.get(), &ProcessorBase::processorClockedSignal, this,
            &ProcessorManager::processorStateChangedSignal, Qt::DirectConnection);
    connect(m_currentProcessor.get(), &ProcessorBase::processorClockedSignal, this,
            &ProcessorManager::processorClockedSlot, Qt::DirectConnection);        
    connect(m_currentProcessor.get(), &ProcessorBase::processorPausedAtBreakpointSignal, this,
            &ProcessorManager::processorPausedAtBreakpointSignal, Qt::DirectConnection);
    connect(m_currentProcessor.get(), &ProcessorBase::processorClockedSignal, this,
            &ProcessorManager::processorClockedSignal, Qt::DirectConnection);
    // create new connections for the new VM
}

void ProcessorManager::loadProgram(const AssembledProgram &program)
{
    m_currentProgram = program;
    m_currentProcessor->LoadProgram(program);
    m_profiler.setInstructionToLineMapping(program);
    m_profiler.setLineNumberToInstructionTypeMapping(program);
    updateEditorHighlight({0});
}

void ProcessorManager::loadProgram(const std::string &sourceText)
{
    std::ofstream out(globals::temporary_assembly_file_path);
    out << sourceText;
    out.close();
    m_currentProgram = assemble(globals::temporary_assembly_file_path.string());
    m_currentProcessor->LoadProgram(m_currentProgram);
    DumpDisasssembly(globals::disassembly_file_path, m_currentProgram);
    std::ifstream in(globals::disassembly_file_path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    in.close();
    updateEditorHighlight({0});
    emit updateDisassemblySignal(QString::fromStdString(buffer.str()));
}

void ProcessorManager::runSlot()
{
    run();

    std::cout << "\n========== RVV PROFILING ==========\n";

    std::cout << "Total RVV Instructions: "
              << getRvvInstructions() << '\n';

    std::cout << "Arithmetic Instructions: "
              << getRvvArithmeticInstructions() << '\n';

    std::cout << "Logical Instructions: "
              << getRvvLogicalInstructions() << '\n';

    std::cout << "Reduction Instructions: "
              << getRvvReductionInstructions() << '\n';

    std::cout << "Move Instructions: "
              << getRvvMoveInstructions() << '\n';

    std::cout << "Vector Loads: "
              << getRvvLoadInstructions() << '\n';

    std::cout << "Vector Stores: "
              << getRvvStoreInstructions() << '\n';

    std::cout << "Elements Processed: "
              << getRvvElementsProcessed() << '\n';

    std::cout << "Memory Bytes Read: "
              << getRvvMemoryBytesRead() << '\n';

    std::cout << "Memory Bytes Written: "
              << getRvvMemoryBytesWritten() << '\n';

    std::cout << "===================================\n";

    std::cout << "Vector execution time: "
          << getRvvExecutionTimeNs() << " ns\n";

    std::cout << "Vector memory time: "
            << getRvvMemoryTimeNs() << " ns\n";

    std::cout << "  Peek time: "
          << getRvvPeekTimeNs() << " ns\n";

    std::cout << "  Write time: "
            << getRvvWriteTimeNs() << " ns\n";

    std::cout << "  Undo bookkeeping time: "
            << getRvvUndoTimeNs() << " ns\n";
    emit runFinishedSignal();

    std::cout << "\n========== SCALAR SD PROFILING ==========\n";

std::cout << "SD write time: "
          << getScalarSdWriteTimeNs() << " ns\n";

std::cout << "SD total time: "
          << getScalarSdTotalTimeNs() << " ns\n";

std::cout << "=========================================\n";

}

void ProcessorManager::debugRunSlot()
{
    m_currentProcessor->start_paused_ = true;
    runSlot();
}

void ProcessorManager::processorClockedSlot(const ProcessorState &processorState)
{
    //first we get all the necesary info from the prcossor and then update the related
    //gui widgets
    updateEditorHighlight(processorState.programCounters);

}


void ProcessorManager::updateEditorHighlight(const std::vector<uint64_t>& programCounters)
{
    std::vector<std::pair<int,std::string>> editorLines;
    std::vector<std::pair<int,std::string>> disassemblyLines;
    const auto addHighlightIfMapped = [&](const std::map<unsigned int, unsigned int> &mapping,
                                          unsigned int instructionNumber,
                                          const std::string &stageLabel,
                                          std::vector<std::pair<int, std::string>> &lines)
    {
        const auto it = mapping.find(instructionNumber);
        if (it == mapping.end())
        {
            return false;
        }

        lines.emplace_back(static_cast<int>(it->second), stageLabel);
        return true;
    };

    static const std::array<std::string, 5> pcToStageLable = {"IF","ID", "EX", "MEM", "WB"};

    if(m_currentProcessorType == ProcessorType::RVSS)
    {
        auto programCounter = programCounters[0];
        const auto instructionNumber = static_cast<unsigned int>(programCounter / 4);
        addHighlightIfMapped(m_currentProgram.instruction_number_line_number_mapping,
                             instructionNumber,
                             ".",
                             editorLines);
        addHighlightIfMapped(m_currentProgram.instruction_number_disassembly_mapping,
                             instructionNumber,
                             ".",
                             disassemblyLines);
    }
    else
    {

        for(size_t i = 0; i < programCounters.size(); ++i)
        {
            auto stagePC = programCounters[i];
            if(stagePC == INVALID_PC) continue; // skip stages that are not active

            const auto instructionNumber = static_cast<unsigned int>(stagePC / 4);
            const auto stageLabel = pcToStageLable[i];

            addHighlightIfMapped(m_currentProgram.instruction_number_line_number_mapping,
                                 instructionNumber,
                                 stageLabel,
                                 editorLines);
            addHighlightIfMapped(m_currentProgram.instruction_number_disassembly_mapping,
                                 instructionNumber,
                                 stageLabel,
                                 disassemblyLines);
            
        }
        
    }
    emit updateEditorHighlightSignal(editorLines, disassemblyLines);
}

int ProcessorManager::resolveCurrentSourceLine() const
{
    try
    {
        const uint64_t programCounter = m_currentProcessor->program_counter_;
        return static_cast<int>(
            m_currentProgram.instruction_number_line_number_mapping.at(programCounter / 4));
    }
    catch (const std::out_of_range &)
    {
        return -1;
    }
}

void ProcessorManager::run()
{
    try
    {
        m_currentProcessor->Run();
    }
    catch (const std::exception &e)
    {
        emit runErrorSignal(QString::fromStdString(e.what()), resolveCurrentSourceLine());
    }
}

void ProcessorManager::step()
{
    try
    {
        if (!m_currentProcessor->StepAndNotify())
        {
            // program finished while stepping, let the run thread exit so the ui resets
            m_currentProcessor->RequestStop();
        }
    }
    catch (const std::exception &e)
    {
        emit runErrorSignal(QString::fromStdString(e.what()), resolveCurrentSourceLine());
    }
}

void ProcessorManager::debugRun()
{
    // Debug mode starts in paused/manual mode; user advances with Step.
    m_currentProcessor->ClearStop();
    m_currentProcessor->RequestPause();
}

void ProcessorManager::stop()
{
    m_currentProcessor->RequestStop();
}
void ProcessorManager::pause()
{
    m_currentProcessor->RequestPause();
}
void ProcessorManager::resume()
{
    m_currentProcessor->RequestResume();
}
void ProcessorManager::undo()
{
    m_currentProcessor->Undo();
}
void ProcessorManager::redo()
{
    m_currentProcessor->Redo();
}

void ProcessorManager::pushInput(const std::string &input)
{
    m_currentProcessor->PushInput(input);
}
void ProcessorManager::setStepDelay(unsigned int delay)
{
    m_stepDelayMs = delay;
    m_currentProcessor->step_delay_ = delay;
}

void ProcessorManager::setBreakpoints(const std::vector<uint64_t> &breakpoints)
{
    m_currentProcessor->SetBreakpoints(breakpoints);
}

RegisterFile *ProcessorManager::getRegisterFile() const
{
    return &m_currentProcessor->registers_;
}

MemoryController *ProcessorManager::getMemoryController() const
{
    return &m_currentProcessor->memory_controller_;
}

Kites::CircuitScene *ProcessorManager::getCircuitScene() const
{
    return m_currentProcessor->circuit_scene_.get();
}
void ProcessorManager::reset()
{
    m_currentProcessor->Reset();
}

ProcessorType ProcessorManager::getProcessorType()
{
    return m_currentProcessorType;
}

const Profiler* ProcessorManager::getProfiler() const
{
    return &m_profiler;
}

uint64_t ProcessorManager::getProgramCounter() const
{
    return m_currentProcessor->program_counter_;
}

float ProcessorManager::getCPI() const
{
    return m_currentProcessor->cpi_;
}

float ProcessorManager::getIPC() const
{
    return m_currentProcessor->ipc_;
}

unsigned int ProcessorManager::getBranchMispredictions() const
{
    return m_currentProcessor->branch_mispredictions_;
}

unsigned int ProcessorManager::getStallCycles() const
{
    return m_currentProcessor->stall_cycles_;
}

unsigned int ProcessorManager::getCycles() const
{
    return m_currentProcessor->cycle_s_;
}

unsigned int ProcessorManager::getInstructionsRetired() const
{
    return m_currentProcessor->instructions_retired_;
}

// RVV profiling getters

uint64_t ProcessorManager::getRvvInstructions() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvInstructions() : 0;
}

uint64_t ProcessorManager::getRvvArithmeticInstructions() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvArithmeticInstructions() : 0;
}

uint64_t ProcessorManager::getRvvLogicalInstructions() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvLogicalInstructions() : 0;
}

uint64_t ProcessorManager::getRvvReductionInstructions() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvReductionInstructions() : 0;
}

uint64_t ProcessorManager::getRvvMoveInstructions() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvMoveInstructions() : 0;
}

uint64_t ProcessorManager::getRvvLoadInstructions() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvLoadInstructions() : 0;
}

uint64_t ProcessorManager::getRvvStoreInstructions() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvStoreInstructions() : 0;
}

uint64_t ProcessorManager::getRvvElementsProcessed() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvElementsProcessed() : 0;
}

uint64_t ProcessorManager::getRvvMemoryBytesRead() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvMemoryBytesRead() : 0;
}

uint64_t ProcessorManager::getRvvMemoryBytesWritten() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvMemoryBytesWritten() : 0;
}
uint64_t ProcessorManager::getRvvExecutionTimeNs() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvExecutionTimeNs() : 0;
}

uint64_t ProcessorManager::getRvvMemoryTimeNs() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvMemoryTimeNs() : 0;
}
uint64_t ProcessorManager::getRvvPeekTimeNs() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvPeekTimeNs() : 0;
}

uint64_t ProcessorManager::getRvvWriteTimeNs() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvWriteTimeNs() : 0;
}

uint64_t ProcessorManager::getRvvUndoTimeNs() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getRvvUndoTimeNs() : 0;
}
uint64_t ProcessorManager::getScalarSdWriteTimeNs() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getScalarSdWriteTimeNs() : 0;
}

uint64_t ProcessorManager::getScalarSdTotalTimeNs() const
{
    auto* rvss = dynamic_cast<RVSSProcessor*>(m_currentProcessor.get());
    return rvss ? rvss->getScalarSdTotalTimeNs() : 0;
}
}//namespace Kites
