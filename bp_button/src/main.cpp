// Blackpill: STM32F103C8T6
/*
 * Toggle LED when button pressed.
 *                  |
 *                -----
 * PA7 --330ohm---o   o--3V3
 *              |
 *              --2Mohm--GND
 *
 * PB13 --100ohm--LED--GND
 *
 * PB12 --Onboard LED - Flash/sec
 */
#include <pt-extended.h>
#include "stm32f10x.h"

#define BUTTON_PRESSED  (GPIOA ->IDR & GPIO_IDR_IDR7)
#define LED_TOGGLE      GPIOB ->ODR ^= (1<<13)
#define LED_TICK_TOGGLE GPIOB ->ODR ^= (1<<12)
#define LED_TICK_ON     GPIOB ->BSRR |= GPIO_BSRR_BS12
#define LED_TICK_OFF    GPIOB ->BRR |= GPIO_BRR_BR12

// Protothread structures
static struct pt pt_tick, pt_button;

// SysTick set to increment msticks every ms
//static uint32_t msticks = 0;
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
// The IRQ has to be declared as a C function otherwise it does not overwrite the weak version in exception_handlers.c
#ifdef __cplusplus
extern "C" {
#endif
void SysTick_Handler(void)
{
  pt_msTicks++; // Protothread ms tick counter
}
#ifdef __cplusplus
}
#endif

void initialise() {
  // Enable the GPIO clock for Port B & A
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

  //PB13 - LED - Output
  // MODE13=11: 50 MHz
  GPIOB->CRH |= GPIO_CRH_MODE13;
  // CNF13=00: General purpose output push-pull
  GPIOB->CRH &= ~(GPIO_CRH_CNF13);

  //PB12 - LED - Output
  // MODE12=11: 50 MHz
  GPIOB->CRH |= GPIO_CRH_MODE12;
  // CNF12=00: General purpose output push-pull
  GPIOB->CRH &= ~(GPIO_CRH_CNF12);

  // PA7 - Switch Input
  // MODE7=00: Input mode
  GPIOA->CRL &= ~(GPIO_CRL_MODE7);
  // CNF7=01: Floating input
  GPIOA->CRL |= GPIO_CRL_CNF7_0;
  GPIOA->CRL &= ~(GPIO_CRL_CNF7_1);

  // Setup SysTick as 1ms timer using CMSIS core routines. ISR Increments pt_milliSec
  if (SysTick_Config(SystemCoreClock / 1000)) {
    while (1)
      LED_TICK_ON; // Capture error
  }
}

// push button state machine thread
#define BUTTON_DEBOUNCE_MS 5
static PT_THREAD (protothread_button(struct pt *pt)) {
  // Push button structure
  static struct button_t {
    enum state_t {
      RELEASED,
      MAYBE_PUSHED,
      PUSHED,
      MAYBE_RELEASED
    } state = RELEASED;
  } button;
  static unsigned int timeout;
  PT_BEGIN(pt);
  while (1) {
    // Cannot use PT calls inside a switch. Using if instead.
    if (button.state == button.RELEASED) {
      PT_WAIT_UNTIL(pt, BUTTON_PRESSED);  // Sit here until the button is pressed
      button.state = button.MAYBE_PUSHED;
    }
    if (button.state == button.MAYBE_PUSHED) {
      // Check the button remains pressed for a few ms
      timeout = PT_GET_TIME() + BUTTON_DEBOUNCE_MS;
      // Checks the condition every time the thread is scheduled.
      PT_WAIT_WHILE(pt, (BUTTON_PRESSED && PT_GET_TIME() < timeout));
      if (BUTTON_PRESSED) {
        button.state = button.PUSHED; // Timeout expired. Confirmed button push
        LED_TOGGLE;                   // Transition action
      } else {
        button.state = button.RELEASED; // Released before timeout. Still bouncing. Go back.
      }
    }
    if (button.state == button.PUSHED) {
      PT_WAIT_WHILE(pt, BUTTON_PRESSED); // Sit here until the button is released
      button.state = button.MAYBE_RELEASED;
    }
    if (button.state == button.MAYBE_RELEASED) {
      // Check the button remains released for a few ms
      timeout = PT_GET_TIME() + BUTTON_DEBOUNCE_MS;
      PT_WAIT_WHILE(pt, (!BUTTON_PRESSED && PT_GET_TIME() < timeout));
      if (BUTTON_PRESSED) {
        button.state = button.PUSHED; // Still pressed before timeout. Bouncing. Go back.
      } else {
        button.state = button.RELEASED; // Timeout expired. Confirmed button release.
      }
    }
    PT_YIELD(pt);
  }
  PT_END(pt);
}

// Thread to blink LED
static PT_THREAD (protothread_tick(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {
      LED_TICK_TOGGLE;
      PT_YIELD_TIME_msec(500);
    }
    PT_END(pt);
}

int main() {
    initialise();
    PT_INIT(&pt_tick);
    PT_INIT(&pt_button);
    while (1) {
        PT_SCHEDULE(protothread_tick(&pt_tick));
        PT_SCHEDULE(protothread_button(&pt_button));
    }
}
