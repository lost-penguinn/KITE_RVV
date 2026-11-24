
# Test Case 3: Sensor Signal Polarity Correction
# Simulates sensor processing by changing signal signs using a
# reference polarity vector. Tests vfsgnj.vv, vfsgnjn.vv, vfsgnjx.vv.

main:
    # Base address
    lui t0, 0x2

    # Input signal array at 0x2000
    addi t1, t0, 0x100
    lui t5, 0x40400       # 3.0
    sw t5, 0(t0)
    lui t5, 0xC0A00       # -5.0
    sw t5, 4(t0)
    lui t5, 0x40E00       # 7.0
    sw t5, 8(t0)
    lui t5, 0xC1100       # -9.0
    sw t5, 12(t0)

    # Reference polarity array at 0x2100
    lui t5, 0xC0000       # -2.0
    sw t5, 0(t1)
    lui t5, 0x40800       # 4.0
    sw t5, 4(t1)
    lui t5, 0xC0C00       # -6.0
    sw t5, 8(t1)
    lui t5, 0x41000       # 8.0
    sw t5, 12(t1)

    # Load vectors
    addi t2, t0, 0x200
    addi t3, t0, 0x300
    addi t4, t0, 0x400

    vsetvli t6, zero, 208
    vle32.v v1, (t0)
    vle32.v v2, (t1)

    # Copy sign from reference
    vfsgnj.vv v3, v1, v2
    vse32.v v3, (t2)

    # Copy inverted sign from reference
    vfsgnjn.vv v4, v1, v2
    vse32.v v4, (t3)

    # XOR the signs
    vfsgnjx.vv v5, v1, v2
    vse32.v v5, (t4)

end:
    addi x0, x0, 0