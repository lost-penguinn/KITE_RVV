main:
    # Configure 4 lanes, SEW=32, LMUL=1
    vsetvli t3, zero, 208

    # Base addresses
    lui t0, 0x2
    addi t1, t0, 0x100
    addi t2, t0, 0x200

    # Load both arrays
    vle32.v v1, (t0)
    vle32.v v2, (t1)

    # Add corresponding elements
    vadd.vv v3, v1, v2

    # Store result
    vse32.v v3, (t2)

end:
    addi x0, x0, 0
