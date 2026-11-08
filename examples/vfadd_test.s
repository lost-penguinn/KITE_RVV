main:
    vsetvli t3, zero, 208

    vfadd.vv v3, v1, v2

end:
    addi x0, x0, 0
