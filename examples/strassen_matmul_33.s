main:
    lui  a0, 0x2
    lui  a1, 0x4
    lui  a2, 0x6
    li   a3, 33

    vmatmul.strassen v3, v1, v2

end:
    addi x0, x0, 0
