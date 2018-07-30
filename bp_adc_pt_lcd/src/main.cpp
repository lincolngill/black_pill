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
#define VERSION 5

//#include "diag/Trace.h"
#include <LCD.h>
#include "stm32f10x.h"
#include "pt-extended.h"
#include <stdio.h>
#include <time.h>

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
 * Thread to increment running_secs every second and signal the LCD thread it needs to update
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
  static time_t seconds;
  static tm *p;
  PT_LCD_INIT(pt);
  //PT_LCD_PRINTSTR(pt,"v3.0 Boot Time:");
  //sprintf(text_buffer, "v%d BootTime:", VERSION);
  // Line 1 = Compile time
  sprintf(text_buffer, "v%d Compiled", VERSION);
  PT_LCD_PRINTSTR(pt,text_buffer);
  PT_LCD_2NDLINE(pt);
  date[6]=0; // truncate to "MMM DD"
  sprintf(text_buffer, "%s %s", date, __TIME__);
  PT_LCD_PRINTSTR(pt,text_buffer);
  PT_YIELD_TIME_msec(pt,2000);
  PT_LCD_CLEARDISPLAY(pt);
  PT_LCD_PRINTSTR(pt,"Running:");
  while(1) {
    PT_LCD_2NDLINE(pt);
    //sprintf(text_buffer, "%8d seconds",running_secs);
    seconds = running_secs;
    p = gmtime(&seconds);
    sprintf(text_buffer, "%03d %02d:%02d:%02d",p->tm_yday, p->tm_hour, p->tm_min, p->tm_sec);
    PT_LCD_PRINTSTR(pt, text_buffer);
    PT_SEM_WAIT(pt, &time_update);
  }
  PT_END(pt);
}

void init_adc() {
  // Setup ADC prescalar to ADCPRE[1:0]=10. PCLK2/6 = 72/6 = 12 MHz must be <= 14 Mhz
  RCC->CFGR &= ~(RCC_CFGR_ADCPRE_0);
  RCC->CFGR |= RCC_CFGR_ADCPRE_1;
  // Set Data alignment: ALIGN=0 Right (default)
  ADC1->CR2 &= ~(ADC_CR2_ALIGN);
  // Turn on ADC ADON=1
  ADC1->CR2 |= ADC_CR2_ADON;
  // Calibrate: Wait >= 2 ADC clock cycles after power on. Then set CAL=1 and wait until H/W resets it
  //   2/12000000 = 167ns. 72/12*2 = 12 cycles of HCLK (72MHz)
  uint32_t t = 50; // Should be more than 12 cycles
  while (t--) __asm__("nop");
  ADC1->CR2 |= ADC_CR2_CAL;
  LED_TICK_ON ; // LED will stay on if calibration never finishes
  while (ADC1->CR2 & ADC_CR2_CAL) __asm__("nop"); // Wait until ADC_CR2_CAL=0
  LED_TICK_OFF ;
  // Select regular channels to convert ADC_SQRx
  // Set number of regular channels ADC_SQR1->L[3:0]=1 (or 2 if want temperature ADCx_IN16 too)
  // Set channel sample times ADC_SMPRx->SMP[2:0]
  //   Tconv = (SampleTime + 12.5 cycles) * 1/ADCCLK. E.g. (1.5 + 12.5) * 1/14000000 = 1us

  // Single conversion: ADC_CR2->ADON=1 when CONT=0. When done ADC is stopped.
  // Continuous: ADC_CR2->ADON=1 when CONT=1
  // Trigger: EXT-TRIG=1,  EXT-SEL[2:0] selects regular channel trigger. E.g. TM1_CC1 or SWSTART
  //   ADC_CR2->SWSTART=1 triggers via software

  // Set EOCIE End-of-conversion-interrupt-enable to generate interrupt

  // Scan mode ADC_CR1->SCAN=1. Scans all channels in ADC_SQRx. DMA must be set
  // Discontinuous mode: ADC_CR1->DISCEN=1 and ADC_CR1->DISCNUM[2:0] n channels per trigger.

  // Temperature sensor: ADCx_IN16. Sample time = 17.1us. ADC_CR2->TSVREFE=1 (ADCx_IN17 Vrefint)
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
