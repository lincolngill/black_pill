// Blackpill: STM32F103C8T6
/*
 * PB12 --Onboard LED - Flash/sec
 */
//#include <stdio.h>
//#include <stdlib.h>
//#include "diag/Trace.h"

#include <pt-extended.h>
#include "stm32f10x.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#define LED_TICK_TOGGLE GPIOB ->ODR ^= (1<<12)
#define LED_TICK_ON     GPIOB ->BSRR |= GPIO_BSRR_BS12
#define LED_TICK_OFF    GPIOB ->BRR |= GPIO_BRR_BR12

// SysTick set to increment msticks every ms
// The IRQ has to be declared as a C function otherwise it does not overwrite the weak version in exception_handlers.c
#ifdef __cplusplus
extern "C" {
#endif
// ms tick counter. Updated by Systick or TIM ISR
// Comment out pt_msTicks definition if using pt-extended.h
//volatile unsigned int pt_msTicks = 0;
void SysTick_Handler(void)
{
  pt_msTicks++; // Protothread ms tick counter
}
#ifdef __cplusplus
}
#endif

void initialise() {
  // Enable the GPIO clock for Port B
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  //PB12 - LED - Output
  // MODE12=11: 50 MHz
  GPIOB->CRH |= GPIO_CRH_MODE12;
  // CNF12=00: General purpose output push-pull
  GPIOB->CRH &= ~(GPIO_CRH_CNF12);

  // Setup SysTick as 1ms timer using CMSIS core routines. ISR Increments pt_milliSec
  if (SysTick_Config(SystemCoreClock / 1000)) {
    while (1)
      LED_TICK_ON; // Capture error
  }
}

void msdelay(uint32_t ms) {
  uint32_t wait_till = pt_msTicks + ms;
  while (wait_till > pt_msTicks) ;
}

#define MSDELAY 1000
int main(int argc, char* argv[]) {
  int mswait = MSDELAY;
  if (argc > 1)
    {
      // If defined, get the number of loops from the command line,
      // configurable via semihosting.
      //mswait = atoi (argv[1]);
    }
  initialise();
  while (1) {
      LED_TICK_TOGGLE ;
      msdelay(mswait);
  }
}

#pragma GCC diagnostic pop
