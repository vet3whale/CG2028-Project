.syntax unified
.cpu cortex-m4
.thumb

.global sqrtf_approx
.type sqrtf_approx, %function

sqrtf_approx:
    @ Input:  S0 (The number to find the square root of)
    @ Output: S0 (The approximated square root)

    VCMP.F32 S0, #0.0
    VMRS APSR_nzcv, FPSCR
    BLE end_sqrt

    VMOV.F32 S4, S0
    VMOV.F32 S1, #1.0
    VMOV.F32 S3, #0.5
    MOV R0, #10

loop_start:
    @ babylonian formula: x = 0.5 * (x + (S / x))
    VDIV.F32 S2, S4, S1
    VADD.F32 S1, S1, S2
    VMUL.F32 S1, S1, S3

    SUBS R0, R0, #1
    BNE loop_start

    VMOV.F32 S0, S1

end_sqrt:
    BX LR
