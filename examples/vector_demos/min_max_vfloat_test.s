
# ==================================================
# Test Case 2: Image Pixel Intensity Clipping
#
# This program simulates image processing by
# restricting pixel intensity values to [0.0, 1.0].
#
# vfmax.vv: Applies the minimum intensity limit (0.0).
# vfmin.vv: Applies the maximum intensity limit (1.0).
#
# It verifies FP32 vector min/max operations using
# intermediate and final hexadecimal results.
# ==================================================

main:
    # --------------------------------
    # 1. Initialize memory addresses
    # --------------------------------
    lui t0, 0x2
    addi t1, t0, 0x100
    addi t2, t0, 0x200
    addi t3, t0, 0x300

    # --------------------------------
    # 2. Initialize pixel values
    # P = [0.2, 0.8, 1.4, -0.1]
    # --------------------------------

    # 0.2 = 0x3E4CCCCD
    lui t5, 0x3E4CD
    addi t5, t5, -819
    sw t5, 0(t0)

    # 0.8 = 0x3F4CCCCD
    lui t5, 0x3F4D
    addi t5, t5, -819
    sw t5, 4(t0)

    # 1.4 = 0x3FB33333
    lui t5, 0x3FB33
    addi t5, t5, 819
    sw t5, 8(t0)

    # -0.1 = 0xBDCCCCCD
    lui t5, 0xBDCCD
    addi t5, t5, -819
    sw t5, 12(t0)

    # --------------------------------
    # 3. Initialize minimum limit
    # Zero = [0.0, 0.0, 0.0, 0.0]
    # --------------------------------
    sw zero, 0(t1)
    sw zero, 4(t1)
    sw zero, 8(t1)
    sw zero, 12(t1)

    # --------------------------------
    # 4. Initialize maximum limit
    # One = [1.0, 1.0, 1.0, 1.0]
    # --------------------------------
    lui t5, 0x3F800
    sw t5, 0(t2)
    sw t5, 4(t2)
    sw t5, 8(t2)
    sw t5, 12(t2)

    # --------------------------------
    # 5. Configure vector registers
    # SEW = 32, LMUL = 1
    # --------------------------------
    vsetvli t6, zero, 208

    # Load pixel values and limits
    vle32.v v1, (t0)
    vle32.v v2, (t1)
    vle32.v v3, (t2)

    # --------------------------------
    # 6. Apply minimum intensity limit
    # T = max(P, 0.0)
    # Expected: [0.2, 0.8, 1.4, 0.0]
    # --------------------------------
    vfmax.vv v4, v1, v2

    # Store intermediate result
    vse32.v v4, (t3)

    # --------------------------------
    # 7. Apply maximum intensity limit
    # R = min(T, 1.0)
    # Expected: [0.2, 0.8, 1.0, 0.0]
    # --------------------------------
    vfmin.vv v5, v4, v3

    # Store final clipped pixels
    addi t3, t3, 0x100
    vse32.v v5, (t3)

end:
    addi x0, x0, 0