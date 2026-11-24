# Test Case 1: Industrial Motor Vibration Analysis
# This program simulates motor vibration monitoring by processing accelerometer measurements.
# It removes baseline vibration, calculates vibration energy, normalizes the values,
# and computes a final severity score using FP32 vector operations.
# It verifies vfsub.vv, vfmul.vv, vfdiv.vv, and vfadd.vv by checking intermediate
# and final results against expected FP32 hexadecimal values.

main:
    # --------------------------------
    # 1. Initialize memory addresses
    # --------------------------------
    lui t0, 0x2
    addi t1, t0, 0x100
    addi t2, t0, 0x200
    addi t3, t0, 0x300
    addi t4, t0, 0x400

    # --------------------------------
    # 2. Initialize vibration samples X
    # X = [2.0, 3.5, 8.0, 4.0]
    # --------------------------------
    lui t5, 0x40000
    sw t5, 0(t0)

    lui t5, 0x40600
    sw t5, 4(t0)

    lui t5, 0x41000
    sw t5, 8(t0)

    lui t5, 0x40800
    sw t5, 12(t0)

    # --------------------------------
    # 3. Initialize baseline B
    # B = [1.0, 1.0, 1.0, 1.0]
    # --------------------------------
    lui t5, 0x3F800
    sw t5, 0(t1)
    sw t5, 4(t1)
    sw t5, 8(t1)
    sw t5, 12(t1)

    # --------------------------------
    # 4. Initialize calibration C
    # C = [2.0, 2.0, 4.0, 2.0]
    # --------------------------------
    lui t5, 0x40000
    sw t5, 0(t2)
    sw t5, 4(t2)

    lui t5, 0x40800
    sw t5, 8(t2)

    lui t5, 0x40000
    sw t5, 12(t2)

    # --------------------------------
    # 5. Configure vector registers
    # SEW = 32, LMUL = 1
    # --------------------------------
    vsetvli t6, zero, 208

    # Load X and B
    vle32.v v1, (t0)
    vle32.v v2, (t1)

    # --------------------------------
    # 6. D = X - B
    # Expected: [1, 2.5, 7, 3]
    # --------------------------------
    vfsub.vv v3, v1, v2

    # Store D at 0x2400
    vse32.v v3, (t3)

    # --------------------------------
    # 7. E = D * D
    # Expected: [1, 6.25, 49, 9]
    # --------------------------------
    vfmul.vv v4, v3, v3

    # Store E at 0x2500
    vse32.v v4, (t4)

    # --------------------------------
    # 8. N = E / C
    # Expected: [0.5, 3.125, 12.25, 4.5]
    # --------------------------------
    vle32.v v5, (t2)

    vfdiv.vv v6, v4, v5

    # Store N at 0x2600
    addi t3, t3, 0x200
    vse32.v v6, (t3)

    # --------------------------------
    # 9. Weighted score
    # S = 0.75*N + 0.25*D
    # --------------------------------

    # Initialize coefficient vectors
    addi t0, t0, 0x500

    lui t5, 0x3F400
    sw t5, 0(t0)
    sw t5, 4(t0)
    sw t5, 8(t0)
    sw t5, 12(t0)

    lui t5, 0x3E800
    sw t5, 16(t0)
    sw t5, 20(t0)
    sw t5, 24(t0)
    sw t5, 28(t0)

    # Load coefficients
    vle32.v v7, (t0)

    addi t0, t0, 16
    vle32.v v8, (t0)

    # 0.75 * N
    vfmul.vv v9, v6, v7

    # 0.25 * D
    vfmul.vv v10, v3, v8

    # S = 0.75*N + 0.25*D
    vfadd.vv v11, v9, v10

    # Store final severity scores
    addi t3, t3, 0x100
    vse32.v v11, (t3)

end:
    addi x0, x0, 0