/*
 * mov_avg.s
 *
 * Created on: 2/2/2026
 * Author: Hitesh B, Hou Linxin
 */
.syntax unified
 .cpu cortex-m4
 .thumb

 .global mov_avg
 .equ N_MAX, 8
 .bss
 .align 4

 .text
 .align 2
@ CG2028 Assignment, Sem 2, AY 2025/26
@ (c) ECE NUS, 2025
@ Karthikkeyan Vetrivel (A0307459W)
@ Nicholas Lau Hongyi (A0308905B)
@ You could create a look-up table of registers here:
@ R0 = N (number of samples)
@ R1 = accel_buff (pointer pointer to int buffer)
@ Returns R0 = integer average of the N samples
@ write your program from here:
mov_avg:
    @ Save registers (matches template spirit) + keep stack 8-byte aligned
    push {r2-r11, lr}          @ 10 regs = 40 bytes -> aligned

    @ If N <= 0, return 0
    cmp  r0, #0
    ble  .ret_zero

    @ Optional clamp to N_MAX (keep if your lab uses max 8)
    cmp  r0, #N_MAX
    ble  .N_ok
    movs r0, #N_MAX
.N_ok:

    mov  r4, r0                @ r4 = N (loop counter)
    mov  r5, r1                @ r5 = accel_buff pointer
    movs r2, #0                @ r2 = sum = 0

.sum_loop:
    ldr  r3, [r5], #4          @ r3 = *r5; r5 += 4
    adds r2, r2, r3            @ sum += r3
    subs r4, r4, #1            @ counter--
    bne  .sum_loop             @ loop until counter == 0

    @ average = sum / N
    sdiv r0, r2, r0            @ return in r0

    pop  {r2-r11, pc}          @ restore regs and return

.ret_zero:
    movs r0, #0
    pop  {r2-r11, pc}
