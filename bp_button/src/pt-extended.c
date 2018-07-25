/*
 * pt-extended.c - Extended Protothread library - For stm32f103c8
 */

// ms tick counter. Updated by Systick or TIM ISR
volatile unsigned int pt_msTicks = 0;
