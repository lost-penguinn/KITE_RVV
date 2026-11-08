.data

array_a:
    .word 12, -18, 7, -5

array_b:
    .word 12, -6, 20, -10

.text

    vsetvli t3, zero, 208

    la t1, array_a
    vle32.v v1, (t1)

    la t2, array_b
    vle32.v v2, (t2)

    vmsle.vv v8, v1, v2
    vmsleu.vv v9, v1, v2

    ebreak