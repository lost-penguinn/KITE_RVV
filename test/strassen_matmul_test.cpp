#include <gtest/gtest.h>
#include <filesystem>
#include <array>
#include <cstdint>
#include <vector>
#include "assembler/assembler.h"
#include "utils/utils.h"
#include "processor/rvss/rvss_processor.h"

using namespace Kites;

namespace {

constexpr uint64_t kBaseA = 0x2000;
constexpr uint64_t kBaseB = 0x2100;
constexpr uint64_t kBaseC = 0x2200;

constexpr size_t kMatrixWords = 9;

const std::array<uint32_t, kMatrixWords> kMatrixA = {
    1, 2, 3,
    4, 5, 6,
    7, 8, 9
};

const std::array<uint32_t, kMatrixWords> kMatrixB = {
    9, 8, 7,
    6, 5, 4,
    3, 2, 1
};

const std::array<uint32_t, kMatrixWords> kExpectedC = {
    30, 24, 18,
    84, 69, 54,
    138, 114, 90
};

void writeMatrix(MemoryController& memory,
                 uint64_t base,
                 const std::array<uint32_t, kMatrixWords>& values)
{
    for (size_t i = 0; i < values.size(); ++i)
    {
        memory.writeWord_d(base + i * 4, values[i]);
    }
}

} // namespace

TEST(StrassenMatmulTest, MultipliesThreeByThreeMatrices)
{
    setupVmStateDirectory();

    RVSSProcessor vm;

    writeMatrix(vm.memory_controller_, kBaseA, kMatrixA);
    writeMatrix(vm.memory_controller_, kBaseB, kMatrixB);

    for (size_t i = 0; i < kMatrixWords; ++i)
    {
        vm.memory_controller_.writeWord_d(kBaseC + i * 4, 0);
    }

   const auto repo_root =
    std::filesystem::path(__FILE__).parent_path().parent_path();

const auto asm_path =
    repo_root / "examples" / "strassen_matmul_3x3.s";

AssembledProgram program = assemble(asm_path.string());

    vm.LoadProgram(program);
    vm.DebugRun();

    for (size_t i = 0; i < kMatrixWords; ++i)
    {
        const uint32_t actual =
            vm.memory_controller_.readWord_d(kBaseC + i * 4);

        EXPECT_EQ(actual, kExpectedC[i]) << "matrix index=" << i;
    }
}
TEST(StrassenMatmulTest, MultipliesThirtyThreeByThirtyThreeMatrices)
{
    setupVmStateDirectory();

    constexpr size_t N = 33;
    constexpr uint64_t A_BASE = 0x2000;
    constexpr uint64_t B_BASE = 0x4000;
    constexpr uint64_t C_BASE = 0x6000;

    RVSSProcessor vm;

    // A = identity matrix.
    // B[i][j] = i * N + j + 1.
    // Therefore A * B = B.
    for (size_t i = 0; i < N; ++i)
    {
        for (size_t j = 0; j < N; ++j)
        {
            vm.memory_controller_.writeWord_d(
                A_BASE + (i * N + j) * 4,
                (i == j) ? 1u : 0u);

            vm.memory_controller_.writeWord_d(
                B_BASE + (i * N + j) * 4,
                static_cast<uint32_t>(i * N + j + 1));

            vm.memory_controller_.writeWord_d(
                C_BASE + (i * N + j) * 4, 0);
        }
    }

    const auto repo_root =
        std::filesystem::path(__FILE__).parent_path().parent_path();

    const auto asm_path =
        repo_root / "examples" / "strassen_matmul_33.s";

    AssembledProgram program = assemble(asm_path.string());
    vm.LoadProgram(program);
    vm.DebugRun();

    for (size_t i = 0; i < N; ++i)
    {
        for (size_t j = 0; j < N; ++j)
        {
            const uint32_t expected =
                static_cast<uint32_t>(i * N + j + 1);

            const uint32_t actual =
    vm.memory_controller_.readWord_d(C_BASE + (i * N + j) * 4);

            EXPECT_EQ(actual, expected)
                << "matrix position (" << i << ", " << j << ")";
        }
    }
}
TEST(StrassenMatmulTest, UndoRedoRestoresMemoryAndVectorRegisters)
{
    setupVmStateDirectory();

    RVSSProcessor vm;

    writeMatrix(vm.memory_controller_, kBaseA, kMatrixA);
    writeMatrix(vm.memory_controller_, kBaseB, kMatrixB);

    for (size_t i = 0; i < kMatrixWords; ++i)
        vm.memory_controller_.writeWord_d(kBaseC + i * 4, 0);

    const auto repo_root =
        std::filesystem::path(__FILE__).parent_path().parent_path();

    const auto asm_path =
        repo_root / "examples" / "strassen_matmul_3x3.s";

    AssembledProgram program = assemble(asm_path.string());
    vm.LoadProgram(program);

    // Execute the six setup instructions.
    for (int i = 0; i < 6; ++i)
        vm.Step();

    // Snapshot state immediately before the custom instruction.
    std::array<uint32_t, kMatrixWords> initial_output{};
    for (size_t i = 0; i < kMatrixWords; ++i)
        initial_output[i] =
            vm.memory_controller_.readWord_d(kBaseC + i * 4);

    const auto old_v1 = vm.registers_.ReadVreg(1);
    const auto old_v2 = vm.registers_.ReadVreg(2);
    const auto old_v3 = vm.registers_.ReadVreg(3);

    // Execute vmatmul.strassen.
    vm.Step();

    std::array<uint32_t, kMatrixWords> computed_output{};
    for (size_t i = 0; i < kMatrixWords; ++i)
    {
        computed_output[i] =
            vm.memory_controller_.readWord_d(kBaseC + i * 4);
    }

    EXPECT_EQ(computed_output, kExpectedC);

    const auto new_v1 = vm.registers_.ReadVreg(1);
    const auto new_v2 = vm.registers_.ReadVreg(2);
    const auto new_v3 = vm.registers_.ReadVreg(3);

    // Undo the custom instruction.
    vm.Undo();

    for (size_t i = 0; i < kMatrixWords; ++i)
    {
        EXPECT_EQ(
            vm.memory_controller_.readWord_d(kBaseC + i * 4),
            initial_output[i]);
    }

    EXPECT_EQ(vm.registers_.ReadVreg(1), old_v1);
    EXPECT_EQ(vm.registers_.ReadVreg(2), old_v2);
    EXPECT_EQ(vm.registers_.ReadVreg(3), old_v3);

    // Redo the custom instruction.
    vm.Redo();

    for (size_t i = 0; i < kMatrixWords; ++i)
    {
        EXPECT_EQ(
            vm.memory_controller_.readWord_d(kBaseC + i * 4),
            computed_output[i]);
    }

    EXPECT_EQ(vm.registers_.ReadVreg(1), new_v1);
    EXPECT_EQ(vm.registers_.ReadVreg(2), new_v2);
    EXPECT_EQ(vm.registers_.ReadVreg(3), new_v3);
}
TEST(StrassenMatmulTest, SupportsOutputAliasingInputA)
{
    setupVmStateDirectory();

    RVSSProcessor vm;

    writeMatrix(vm.memory_controller_, kBaseA, kMatrixA);
    writeMatrix(vm.memory_controller_, kBaseB, kMatrixB);

    const auto repo_root =
        std::filesystem::path(__FILE__).parent_path().parent_path();

    const auto asm_path =
        repo_root / "examples" / "strassen_matmul_3x3.s";

    AssembledProgram program = assemble(asm_path.string());

    // Change a2 (output address) to equal a0 (input A address).
    // Easier to use a dedicated assembly file for this test.
    vm.LoadProgram(program);

    // Execute the first three setup instructions.
    // Execute all six setup instructions.
for (int i = 0; i < 6; ++i)
{
    vm.Step();
}

// Now make output C alias input A.
vm.registers_.WriteGpr(
    12, vm.registers_.ReadGpr(10));

// Execute vmatmul.strassen.
vm.Step();

    // a2 = a0 = 0x2000.
    vm.registers_.WriteGpr(12, vm.registers_.ReadGpr(10));

    // Execute remaining setup instructions and custom instruction.
    vm.Step();
    vm.Step();
    vm.Step();
    vm.Step();

    for (size_t i = 0; i < kMatrixWords; ++i)
    {
        EXPECT_EQ(
            vm.memory_controller_.readWord_d(kBaseA + i * 4),
            kExpectedC[i])
            << "matrix index=" << i;
    }
}