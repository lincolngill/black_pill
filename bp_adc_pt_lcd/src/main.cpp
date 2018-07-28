// Blackpill: STM32F103C8T6
/*
 * PB12 --Onboard LED - Flashes
 * LCD Refer LCD_HD44780.h/c
 *   PA6  - E  - Enable
 *   PA7  - RS - Register Select
 *   PA8  - D4
 *   PA9  - D5
 *   PA10 - D6
 *   PB15 - D7
 */
//#include <stdio.h>
//#include <stdlib.h>
//#include "diag/Trace.h"
#include <Lcd.h>
#include "stm32f10x.h"
#include "pt-extended.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough="
#define LED_TICK_TOGGLE GPIOB ->ODR ^= (1<<12)
#define LED_TICK_ON     GPIOB ->BSRR |= GPIO_BSRR_BS12
#define LED_TICK_OFF    GPIOB ->BRR |= GPIO_BRR_BR12

// Protothread structures
static struct pt pt_tick, pt_led, pt_lcd;
static struct pt_sem time_update;
Lcd lcd;

void initialise() {
  // Enable the GPIO clock for Port B
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  //PB12 - LED - Output
  // MODE12=11: 50 MHz
  GPIOB->CRH |= GPIO_CRH_MODE12;
  // CNF12=00: General purpose output push-pull
  GPIOB->CRH &= ~(GPIO_CRH_CNF12);
}

// Thread to track seconds elapsed since boot
static uint32_t running_secs = 0;
static PT_THREAD (protothread_tick(struct pt *pt)) {
  PT_BEGIN(pt);
  while (1) {
    PT_YIELD_TIME_msec(1000);
    running_secs++;
    PT_SEM_SIGNAL(pt, &time_update);
  }
  PT_END(pt);
}

// Thread to blink LED
static PT_THREAD (protothread_led(struct pt *pt)) {
  PT_BEGIN(pt);
  while (1) {
    LED_TICK_TOGGLE;
    PT_YIELD_TIME_msec(150);
  }
  PT_END(pt);
}

// Thread to update LCD when running_secs changes
static PT_THREAD (protothread_lcd(struct pt *pt)) {
  PT_BEGIN(pt);
  lcd.init();
  PT_YIELD(pt);
  lcd.printStr("Boot Time");
  while(1) {
    lcd.gotoLine2();
    lcd.printf("%6d seconds",running_secs);
    PT_SEM_WAIT(pt, &time_update);
  }
  PT_END(pt);
}

int main(int argc, char* argv[]) {
  initialise();
  PT_SETUP();
  PT_INIT(&pt_tick);
  PT_INIT(&pt_lcd);
  PT_INIT(&pt_led);
  while (1) {
    PT_SCHEDULE(protothread_tick(&pt_tick));
    PT_SCHEDULE(Lcd::protothread(&pt_lcd, lcd));
    PT_SCHEDULE(protothread_led(&pt_led));
  }
}

#pragma GCC diagnostic pop
