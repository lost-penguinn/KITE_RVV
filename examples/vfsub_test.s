main:
    # Base addresses
    lui t0, 0x2
    addi t1, t0, 0x100
    addi t2, t0, 0x200

    # Array A: [55.5, 10.0, -8.0, 100.0]
    lui t4, 0x425e0
    sw t4, 0(t0)
    lui t4, 0x41200
    sw t4, 4(t0)
    lui t4, 0xc1000
    sw t4, 8(t0)
    lui t4, 0x42c80
    sw t4, 12(t0)

    # Array B: [45.9, 2.5, 3.0, 0.5]
    lui t4, 0x4237a
    addi t4, t4, -1638
    sw t4, 0(t1)
    lui t4, 0x40200
    sw t4, 4(t1)
    lui t4, 0x40400
    sw t4, 8(t1)
    lui t4, 0x3f000
    sw t4, 12(t1)

    # Configure VL=4, SEW=32, LMUL=1
    vsetvli t3, zero, 208

    # Load, subtract, and store
    vle32.v v1, (t0)
    vle32.v v2, (t1)
    vfsub.vv v3, v1, v2
    vse32.v v3, (t2)

end:
    addi x0, x0, 0