
# Test Case 4: Industrial Sensor Range Filtering
# Compares measured sensor values against reference limits.
# Tests equality, inequality, signed and unsigned comparisons.

main:
    lui t0, 0x2

    # Sensor readings at 0x2000
    addi t1, t0, 0x100
    addi t2, t0, 0x200

    addi t4, zero, -5
    sw t4, 0(t0)
    addi t4, zero, 10
    sw t4, 4(t0)
    addi t4, zero, 20
    sw t4, 8(t0)
    addi t4, zero, -1
    sw t4, 12(t0)

    # Reference values at 0x2100
    addi t4, zero, 0
    sw t4, 0(t1)
    addi t4, zero, 10
    sw t4, 4(t1)
    addi t4, zero, 15
    sw t4, 8(t1)
    addi t4, zero, 1
    sw t4, 12(t1)

    # Load vectors
    vsetvli t6, zero, 208
    vle32.v v1, (t0)
    vle32.v v2, (t1)

    # Equality and inequality
    vmseq.vv v3, v1, v2
    vmsne.vv v4, v1, v2

    # Signed comparisons
    vmslt.vv v5, v1, v2
    vmsle.vv v6, v1, v2

    # Unsigned comparisons
    vmsltu.vv v7, v1, v2
    vmsleu.vv v8, v1, v2

end:
    addi x0, x0, 0