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
@ Karthikeyan Vetrivel (A0307459W)
@ Nicholas Lau Hongyi (A0308905B)
@ You could create a look-up table of registers here:
@ R0 = N (number of samples)
@ R1 = accel_buff (pointer pointer to int buffer)
@ Returns R0 = integer average of the N samples
@ write your program from here:
mov_avg:
 PUSH {r2-r11, lr}

 MOV r2, #0
 MOV r4, #0

LOOP:
 CMP r0, #0
 BEQ DONE

 LDR r3, [r1], #4
 ADD r2, r3
 SUBS r0, #1
 ADD r4, #1
 B LOOP

DONE:
 CMP r4, #0
 BEQ ZERO
 SDIV r0, r2, r4
 B EXIT

ZERO:
 MOV r0, #0
EXIT:
 POP {r2-r11, pc}
