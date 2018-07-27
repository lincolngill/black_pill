/*
 * pt-extended.c - Extended Protothread library - For stm32f103c8
 */
#include "pt-extended.h"
#include "stm32f10x.h"

// ms tick counter. Updated by Systick or TIM ISR
volatile unsigned int pt_msTicks = 0;

// SysTick set to increment pt_msTicks every ms
// The IRQ has to be declared as a C function otherwise it does not overwrite the weak version in exception_handlers.c
void SysTick_Handler(void)
{
  pt_msTicks++; // Protothread ms tick counter
}

void PT_SETUP(void) {
  // Setup SysTick as 1ms timer using CMSIS core routines. ISR Increments pt_milliSec
  if (SysTick_Config(SystemCoreClock / 1000)) {
    while (1) ; // Capture error
  }
}
