#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <memory>

#include "assembler/assembler.h"
#include "processor/rvss/rvss_processor.h"
#include "utils/utils.h"

namespace
{

constexpr uint64_t kBaseA = 0x2000;
constexpr uint64_t kBaseB = 0x2100;
constexpr uint64_t kBaseC = 0x2200;

const std::array<uint32_t, 4> kInputA = {10, 20, 30, 40};
const std::array<uint32_t, 4> kInputB = {1, 2, 3, 4};
const std::array<uint32_t, 4> kExpected = {11, 22, 33, 44};

void seedMemory(Kites::MemoryController& memory)
{
    for (size_t i = 0; i < 4; ++i)
    {
        memory.writeWord_d(kBaseA + i * 4, kInputA[i]);
        memory.writeWord_d(kBaseB + i * 4, kInputB[i]);
        memory.writeWord_d(kBaseC + i * 4, 0);
    }
}

} // namespace

TEST(RVSSVectorMemory, LoadAddStoreUnitStrideE32)
{
    Kites::setupVmStateDirectory();

    auto vm = std::make_unique<Kites::RVSSProcessor>();

    seedMemory(vm->memory_controller_);

    Kites::AssembledProgram program =
        Kites::assemble("../examples/vector_memory_test.s");

    vm->LoadProgram(program);
    vm->DebugRun();

    // Verify vector loads.
    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_EQ(vm->registers_.ReadVectorElement(1, 4, i), kInputA[i])
            << "v1 element " << i;

        EXPECT_EQ(vm->registers_.ReadVectorElement(2, 4, i), kInputB[i])
            << "v2 element " << i;

        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i), kExpected[i])
            << "v3 element " << i;
    }

    // Verify vector store.
    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_EQ(
            vm->memory_controller_.readWord_d(kBaseC + i * 4),
            kExpected[i])
            << "memory element " << i;
    }
}
