
.data

array_a:
    .word 12, -18, 7, -5

array_b:
    .word 12, -6, 20, -10

.text

    vsetvli t3, zero, 208

    # Load array A
    la t1, array_a
    vle32.v v1, (t1)

    # Load array B
    la t2, array_b
    vle32.v v2, (t2)

    # Equality
    vmseq.vv v4, v1, v2

    # Not equal
    vmsne.vv v5, v1, v2

    # Signed less than
    vmslt.vv v6, v1, v2

    # Unsigned less than
    vmsltu.vv v7, v1, v2

    ebreak
