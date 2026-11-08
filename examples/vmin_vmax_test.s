
main:
    # Base addresses
    lui t0, 0x2
    addi t1, t0, 0x100
    addi t2, t0, 0x200
    addi t3, t0, 0x300

    # Array A: [12, -18, 7, -5]
    addi t4, zero, 12
    sw t4, 0(t0)
    addi t4, zero, -18
    sw t4, 4(t0)
    addi t4, zero, 7
    sw t4, 8(t0)
    addi t4, zero, -5
    sw t4, 12(t0)

    # Array B: [3, -6, 20, -10]
    addi t4, zero, 3
    sw t4, 0(t1)
    addi t4, zero, -6
    sw t4, 4(t1)
    addi t4, zero, 20
    sw t4, 8(t1)
    addi t4, zero, -10
    sw t4, 12(t1)

    # Configure VL=4, SEW=32, LMUL=1
    vsetvli t5, zero, 208

    # Load arrays
    vle32.v v1, (t0)
    vle32.v v2, (t1)

    # Minimum
    vmin.vv v3, v1, v2
    vse32.v v3, (t2)

    # Maximum
    vmax.vv v4, v1, v2
    vse32.v v4, (t3)

end:
    addi x0, x0, 0
