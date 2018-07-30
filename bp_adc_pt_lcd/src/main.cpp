// Blackpill: STM32F103C8T6
/*
 * PB12 --Onboard LED - Flashes
 * LCD Refer LCD.h/c
 *   PA6  - E  - Enable
 *   PA7  - RS - Register Select
 *   PA8  - D4
 *   PA9  - D5
 *   PA10 - D6
 *   PB15 - D7
 */
#define VERSION 4

//#include "diag/Trace.h"
#include <LCD.h>
#include "stm32f10x.h"
#include "pt-extended.h"
#include <stdio.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough="

#define LED_TICK_TOGGLE GPIOB ->ODR ^= (1<<12)
#define LED_TICK_ON     GPIOB ->BSRR |= GPIO_BSRR_BS12
#define LED_TICK_OFF    GPIOB ->BRR |= GPIO_BRR_BR12

// Protothread structures
static struct pt pt_tick, pt_led, pt_lcd_driver;
static struct pt_sem time_update;

/**
 * Initialise Pins - PB12 LED pin
 */
void initialise() {
  // Enable the GPIO clock for Port B
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  //PB12 - LED - Output
  // MODE12=11: 50 MHz
  GPIOB->CRH |= GPIO_CRH_MODE12;
  // CNF12=00: General purpose output push-pull
  GPIOB->CRH &= ~(GPIO_CRH_CNF12);
}

/**
 * Seconds since boot
 */
static uint32_t running_secs = 0;
/**
 * Thread to increment running_secs every second
 * \param[in] pt protothread pointer
 */
static PT_THREAD (protothread_tick(struct pt *pt)) {
  PT_BEGIN(pt);
  while (1) {
    PT_YIELD_TIME_msec(pt, 1000);
    running_secs++;
    PT_SEM_SIGNAL(pt, &time_update);
  }
  PT_END(pt);
}

/**
 * Thread to toggle the LED
 * \param[in] pt protothread pointer
 */
static PT_THREAD (protothread_led(struct pt *pt)) {
  PT_BEGIN(pt);
  while (1) {
    LED_TICK_TOGGLE;
    PT_YIELD_TIME_msec(pt, 150);
  }
  PT_END(pt);
}

/**
 * Thread to driver LCD output
 * \param[in] pt protothread pointer
 *
 * Updates LCD with running_secs
 */
static PT_THREAD (protothread_lcd(struct pt *pt)) {
  PT_BEGIN(pt);
  static char text_buffer[32];
  static char date[] = __DATE__;
  PT_LCD_INIT(pt);
  //PT_LCD_PRINTSTR(pt,"v3.0 Boot Time:");
  //sprintf(text_buffer, "v%d BootTime:", VERSION);
  // Line 1 = Compile time
  date[6]=0;
  sprintf(text_buffer, "%s %s", date, __TIME__);
  PT_LCD_PRINTSTR(pt,text_buffer);
  while(1) {
    PT_LCD_2NDLINE(pt);
    sprintf(text_buffer, "%6d seconds",running_secs);
    PT_LCD_PRINTSTR(pt, text_buffer);
    PT_SEM_WAIT(pt, &time_update);
  }
  PT_END(pt);
}

int main(int argc, char* argv[]) {
  initialise();
  PT_SETUP();
  PT_INIT(&pt_tick);
  PT_INIT(&pt_lcd_driver);
  PT_INIT(&pt_led);
  // Schedule threads
  while (1) {
    PT_SCHEDULE(protothread_tick(&pt_tick));
    PT_SCHEDULE(protothread_lcd(&pt_lcd_driver));
    PT_SCHEDULE(protothread_led(&pt_led));
  }
}

#pragma GCC diagnostic pop
