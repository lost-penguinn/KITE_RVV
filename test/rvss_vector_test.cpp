
#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <sstream>

#include "assembler/assembler.h"
#include "processor/rvss/rvss_processor.h"
#include "utils/utils.h"

namespace
{

std::unique_ptr<Kites::RVSSProcessor> runVectorProgram(
    const std::string& source)
{
    Kites::setupVmStateDirectory();

    auto vm = std::make_unique<Kites::RVSSProcessor>();

    std::istringstream input(source);
    Kites::AssembledProgram program = Kites::assemble(input);

    vm->LoadProgram(program);
    vm->DebugRun();

    return vm;
}

TEST(RVSSVector, SetVectorLengthAndMoveImmediate)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
    )");


    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_EQ(vm->registers_.ReadVectorElement(1, 4, i), 5)
            << "v1 element " << i;
    }
}

TEST(RVSSVector, AddVectorElements)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
        vmv.v.i v2, 7
        vadd.vv v3, v1, v2
    )");

    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i), 12)
            << "v3 element " << i;
    }
}

TEST(RVSSVector, SubtractVectorElements)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 12
        vmv.v.i v2, 5
        vsub.vv v3, v1, v2
    )");

    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i), 7)
            << "v3 element " << i;
    }
}

TEST(RVSSVector, MultiplyVectorElements)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
        vmv.v.i v2, 7
        vmul.vv v3, v1, v2
    )");

    for (size_t i = 0; i < 4; ++i)
    {
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i), 35)
            << "v3 element " << i;
    }
}

TEST(RVSSVector, AndVectorElements)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 6
        vmv.v.i v2, 3
        vand.vv v3, v1, v2
    )");

    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i), 2);
}

TEST(RVSSVector, OrVectorElements)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 6
        vmv.v.i v2, 3
        vor.vv v3, v1, v2
    )");

    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i), 7);
}

TEST(RVSSVector, XorVectorElements)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 6
        vmv.v.i v2, 3
        vxor.vv v3, v1, v2
    )");

    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i), 5);
}

TEST(RVSSVector, AddScalarToVector)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
        addi t0, zero, 3
        vadd.vx v2, v1, t0
    )");

    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(2, 4, i), 8);
}

TEST(RVSSVector, SubtractScalarFromVector)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 10
        addi t0, zero, 3
        vsub.vx v2, v1, t0
    )");

    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(2, 4, i), 7);
}

TEST(RVSSVector, MultiplyVectorByScalar)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
        addi t0, zero, 3
        vmul.vx v2, v1, t0
    )");

    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(2, 4, i), 15);
}

TEST(RVSSVector, ReductionSum)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 2
        vmv.v.i v2, 0
        vredsum.vs v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, 0), 8);
}

TEST(RVSSVector, ReductionAnd)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 7
        vmv.v.i v2, 15
        vredand.vs v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, 0), 7);
}

TEST(RVSSVector, ReductionOr)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 2
        vmv.v.i v2, 0
        vredor.vs v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, 0), 2);
}

TEST(RVSSVector, ReductionXor)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 3
        vmv.v.i v2, 0
        vredxor.vs v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, 0), 0);
}

TEST(RVSSVector, ReductionMinimumSigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
        vmv.v.i v2, 10
        vredmin.vs v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, 0), 5);
}

TEST(RVSSVector, ReductionMaximumSigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
        vmv.v.i v2, 2
        vredmax.vs v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, 0), 5);
}

TEST(RVSSVector, ReductionMinimumUnsigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
        vmv.v.i v2, 10
        vredminu.vs v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, 0), 5);
}

TEST(RVSSVector, ReductionMaximumUnsigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
        vmv.v.i v2, 2
        vredmaxu.vs v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, 0), 5);
}
TEST(RVSSVector, MinimumSigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        li t0, -5
        vmv.v.x v1, t0
        li t0, 3
        vmv.v.x v2, t0
        vmin.vv v3, v1, v2
    )");

    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i),
                  0xFFFFFFFBu);
}

TEST(RVSSVector, MaximumSigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        li t0, -5
        vmv.v.x v1, t0
        li t0, 3
        vmv.v.x v2, t0
        vmax.vv v3, v1, v2
    )");

    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i), 3);
}

TEST(RVSSVector, MinimumUnsigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        li t0, -1
        vmv.v.x v1, t0
        li t0, 3
        vmv.v.x v2, t0
        vminu.vv v3, v1, v2
    )");

    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i), 3);
}

TEST(RVSSVector, MaximumUnsigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        li t0, -1
        vmv.v.x v1, t0
        li t0, 3
        vmv.v.x v2, t0
        vmaxu.vv v3, v1, v2
    )");

    for (size_t i = 0; i < 4; ++i)
        EXPECT_EQ(vm->registers_.ReadVectorElement(3, 4, i),
                  0xFFFFFFFFu);
}
TEST(RVSSVector, CompareEqual)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
        vmv.v.i v2, 5
        vmseq.vv v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 1, 0), 0x0F);
}

TEST(RVSSVector, CompareNotEqual)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        vmv.v.i v1, 5
        vmv.v.i v2, 3
        vmsne.vv v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 1, 0), 0x0F);
}

TEST(RVSSVector, CompareLessThanSigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        li t0, -1
        vmv.v.x v1, t0
        li t0, 1
        vmv.v.x v2, t0
        vmslt.vv v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 1, 0), 0x0F);
}

TEST(RVSSVector, CompareLessThanUnsigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        li t0, -1
        vmv.v.x v1, t0
        li t0, 1
        vmv.v.x v2, t0
        vmsltu.vv v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 1, 0), 0x00);
}

TEST(RVSSVector, CompareLessOrEqualSigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        li t0, -1
        vmv.v.x v1, t0
        li t0, 1
        vmv.v.x v2, t0
        vmsle.vv v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 1, 0), 0x0F);
}

TEST(RVSSVector, CompareLessOrEqualUnsigned)
{
    auto vm = runVectorProgram(R"(
        .text
        vsetvli t0, zero, 208
        li t0, 3
        vmv.v.x v1, t0
        li t0, 3
        vmv.v.x v2, t0
        vmsleu.vv v3, v1, v2
    )");

    EXPECT_EQ(vm->registers_.ReadVectorElement(3, 1, 0), 0x0F);
}
} // namespace
