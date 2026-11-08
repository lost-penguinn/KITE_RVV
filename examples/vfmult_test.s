main:
# Base addresses
lui t0, 0x2
addi t1, t0, 0x100
addi t2, t0, 0x200

# Array A: [12.5, -6.0, 1.5, 100.0]
lui t4, 0x41480
sw t4, 0(t0)
lui t4, 0xc0c00
sw t4, 4(t0)
lui t4, 0x3fc00
sw t4, 8(t0)
lui t4, 0x42c80
sw t4, 12(t0)

# Array B: [3.25, 2.0, 0.5, 25.0]
lui t4, 0x40500
sw t4, 0(t1)
lui t4, 0x40000
sw t4, 4(t1)
lui t4, 0x3f000
sw t4, 8(t1)
lui t4, 0x41c80
sw t4, 12(t1)

# Configure VL=4, SEW=32, LMUL=1
vsetvli t3, zero, 208

# Load, multiply, store
vle32.v v1, (t0)
vle32.v v2, (t1)
vfmul.vv v3, v1, v2
vse32.v v3, (t2)

end: