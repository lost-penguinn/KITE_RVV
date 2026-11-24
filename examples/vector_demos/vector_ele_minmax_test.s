
# Test Case 5: Industrial Sensor Outlier Detection
# Simulates two industrial sensors measuring the same parameters.
# Calculates element-wise minimum and maximum values using signed
# and unsigned integer vector min/max instructions.

main:
    # --------------------------------
    # 1. Initialize memory addresses
    # --------------------------------
    lui t0, 0x2
    addi t1, t0, 0x100
    addi t2, t0, 0x200
    addi t3, t0, 0x300
    addi t4, t0, 0x400
    addi t5, t0, 0x500

    # --------------------------------
    # 2. Initialize Sensor A
    # A = [12, -45, 8, 67]
    # --------------------------------
    addi t6, zero, 12
    sw t6, 0(t0)

    addi t6, zero, -45
    sw t6, 4(t0)

    addi t6, zero, 8
    sw t6, 8(t0)

    addi t6, zero, 67
    sw t6, 12(t0)

    # --------------------------------
    # 3. Initialize Sensor B
    # B = [20, -30, 15, 50]
    # --------------------------------
    addi t6, zero, 20
    sw t6, 0(t1)

    addi t6, zero, -30
    sw t6, 4(t1)

    addi t6, zero, 15
    sw t6, 8(t1)

    addi t6, zero, 50
    sw t6, 12(t1)

    # --------------------------------
    # 4. Configure vector registers
    # SEW = 32, LMUL = 1
    # --------------------------------
    vsetvli t6, zero, 208

    vle32.v v1, (t0)
    vle32.v v2, (t1)

    # --------------------------------
    # 5. Signed minimum
    # --------------------------------
    vmin.vv v3, v1, v2
    vse32.v v3, (t2)

    # --------------------------------
    # 6. Signed maximum
    # --------------------------------
    vmax.vv v4, v1, v2
    vse32.v v4, (t3)

    # --------------------------------
    # 7. Unsigned minimum
    # --------------------------------
    vminu.vv v5, v1, v2
    vse32.v v5, (t4)

    # --------------------------------
    # 8. Unsigned maximum
    # --------------------------------
    vmaxu.vv v6, v1, v2
    vse32.v v6, (t5)

end:
    addi x0, x0, 0