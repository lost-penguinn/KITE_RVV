main:
    lui  a0, 0x2         # A = 0x2000
    lui  a1, 0x2
    addi a1, a1, 0x100   # B = 0x2100
    lui  a2, 0x2
    addi a2, a2, 0x200   # C = 0x2200
    li   a3, 3           # N = 3

    vmatmul.strassen v3, v1, v2

end:
    addi x0, x0, 0