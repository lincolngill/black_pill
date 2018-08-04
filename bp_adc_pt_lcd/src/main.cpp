
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
#include <Pin.h>
#include "stm32f10x.h"
#include "pt-extended.h"
#include <stdio.h>
#include <time.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough="

// Protothread structures
static struct pt pt_tick, pt_led, pt_lcd_driver;
static struct pt_sem time_update;
static uint16_t analog1 = 0;
static uint16_t analog_reading = 0;

/**
 * onboard LED PB12
 */
Pin led(GPIOB, 12, PIN_PUSHPULL_OUTPUT_50);

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
    led.toggle();
    PT_YIELD_TIME_msec(pt, 150);
  }
  PT_END(pt);
}

#define ADC_TOLERANCE 4
/**
 * Thread to driver LCD output
 * \param[in] pt protothread pointer
 *
 * Updates LCD with ADC reading & running_secs
 */
static PT_THREAD (protothread_lcd(struct pt *pt)) {
  PT_BEGIN(pt);
  static char text_buffer[32];
  static char date[] = __DATE__;
  static time_t seconds;
  static tm *p;
  PT_LCD_INIT(pt);
  // Display Compile time
  sprintf(text_buffer, "v%d Compiled", VERSION);
  PT_LCD_PRINTSTR(pt,text_buffer);
  PT_LCD_2NDLINE(pt);
  date[6]=0; // truncate to "MMM DD"
  sprintf(text_buffer, "%s %s", date, __TIME__);
  PT_LCD_PRINTSTR(pt,text_buffer);
  PT_YIELD_TIME_msec(pt,2000);
  // Display Running seconds
  PT_LCD_CLEARDISPLAY(pt);
  PT_LCD_PRINTSTR(pt,"ADC1:");
  while(1) {
    PT_LCD_2NDLINE(pt);
    //sprintf(text_buffer, "%8d seconds",running_secs);
    //seconds = running_secs;
    //p = gmtime(&seconds);
    //sprintf(text_buffer, "%03d %02d:%02d:%02d",p->tm_yday, p->tm_hour, p->tm_min, p->tm_sec);
    sprintf(text_buffer, "%4d %6ds", analog1, running_secs);
    PT_LCD_PRINTSTR(pt, text_buffer);
    //PT_SEM_WAIT(pt, &time_update);
    // Wait for reading out of band or update in time
    PT_WAIT_UNTIL(pt, analog_reading < analog1-ADC_TOLERANCE || analog_reading > analog1+ADC_TOLERANCE || PT_SEM_READ(&time_update));
    // Don't update analog display value if just a time update. So it doesn't flicker
    (PT_SEM_READ(&time_update)) ? PT_SEM_CLEAR(&time_update) : analog1 = analog_reading;
  }
  PT_END(pt);
}

/**
 * Initialise TMR1 - 10Hz (100ms period) PWM 50% duty. Using CC1 output
 */
void init_tmr() {
  // Enable timer clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
  // Reset CR1. And set count up mode DIR=0
  TIM1->CR1 = 0;
  //TIM1->CR1 |= TIM_CR1_DIR;
  // Set CC1 to output (default). CC1S=0
  TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S);
  // Set output compare 1 mode. OC1M=0b111 PWM 2 - C1 active for TMR1_CNT between 0-500
  //TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M);
  //TIM1->CCMR1 |= TIM_CCMR1_OC1M_0;
  TIM1->CCMR1 |= TIM_CCMR1_OC1M; // PWM mode 2
  // OC preload enable OC1PE=1
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
  // OC fast enable OC1FE=1
  TIM1->CCMR1 |= TIM_CCMR1_OC1FE;
  // CC1 output capture enable CC1E=1
  TIM1->CCER |= TIM_CCER_CC1E;
  // main output enable
  TIM1->BDTR |= TIM_BDTR_MOE;
  // Setup the clock source. CK_INT
  // Set prescaler TIM1->PSC
  // CK_CNT = F(ck_psc)/PSC[15:0]+1
  // For TIM1 APB2 prescaler = 1, so TIM1CLK=72MHz
  // PSC value for 100us period. PSC = 100x10e-6us * 72x10e6 Hz = 7200
  TIM1->PSC = 7199;
  // Set auto reload ARR=1000 = 100ms. So do a conversion 10x a second.
  TIM1->ARR = 1000;
  // Load Compare to generate CC1 trigger
  TIM1->CCR1 = 500;
  // Don't set repetition register TIM1->RCR
  // Disable the UEV disable bit TIM1->CR1 UDIS
  // Enable the counter TIM1->CR1 CEN=1
  TIM1->CR1 |= TIM_CR1_CEN;
}

/**
 * ISR for ADC1 & 2 - Capture EOC value
 * ISRs must be C functions otherwise the weak definition is not over written
 */
extern "C" {
  void ADC1_2_IRQHandler(void) {
    if (ADC1->SR & ADC_SR_EOC) {
      analog_reading = ADC1->DR & 0xFFFF;
    }
  }
}

/**
 * Initialise ADC1 to convert PA0
 */
void init_adc() {
  // Setup ADC prescalar to ADCPRE[1:0]=10. PCLK2/6 = 72/6 = 12 MHz must be <= 14 Mhz
  RCC->CFGR &= ~(RCC_CFGR_ADCPRE_0);
  RCC->CFGR |= RCC_CFGR_ADCPRE_1;
  // Enable ADC1 clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  // Set Data alignment: ALIGN=0 Right (default)
  ADC1->CR2 &= ~(ADC_CR2_ALIGN);
  // Turn on ADC ADON=1
  ADC1->CR2 |= ADC_CR2_ADON;
  // Calibrate: Wait >= 2 ADC clock cycles after power on. Then set CAL=1 and wait until H/W resets it
  //   2/12000000 = 167ns. 72/12*2 = 12 cycles of HCLK (72MHz)
  uint32_t t = 50; // Should be more than 12 cycles
  while (t--) __asm__("nop");
  ADC1->CR2 |= ADC_CR2_CAL;
  led.reset(); // LED will stay on if calibration never finishes
  while (ADC1->CR2 & ADC_CR2_CAL) __asm__("nop"); // Wait until ADC_CR2_CAL=0
  led.set();
  // Select regular channels to convert SQR3[4:0]=0 = ADC0
  ADC1->SQR3 = 0x0000;
  // Set number of regular channels SQR1 L[3:0]=0 = 1 conversion (or 2 if want temperature ADCx_IN16 too)
  ADC1->SQR1 &= ~(ADC_SQR1_L);
  // Set PA0 pin to analog input. CNF=00 (Analog), MODE=00 (Input mode)
  Pin::config(GPIOA, 0, PIN_ANALOG_INPUT);
  // Set channel sample times SMPR2 SMP0[2:0]=0b000 = 1.5 cycles
  //   Tconv = (SampleTime + 12.5 cycles) * 1/ADCCLK. E.g. (1.5 + 12.5) * 1/12000000 = 1.167us
  ADC1->SMPR2 &= ~(ADC_SMPR2_SMP0);

  // Single conversion: ADC_CR2->ADON=1 when CONT=0. When done ADC is stopped.
  // Continuous: ADC_CR2->ADON=1 when CONT=1

  // Trigger: EXTTRIG=1,  EXTSEL[2:0] selects regular channel trigger. E.g. TM1_CC1 or SWSTART
  //   ADC_CR2->SWSTART=1 triggers via software
  ADC1->CR2 |= ADC_CR2_EXTTRIG;
  // EXTSEL[2:0]=0b000 - Timer 1 CC1 event
  ADC1->CR2 &= ~(ADC_CR2_EXTSEL);
  //ADC1->CR2 |= ADC_CR2_EXTSEL; //SWSTART

  // Set EOCIE End-of-conversion-interrupt-enable to generate interrupt
  ADC1->CR1 |= ADC_CR1_EOCIE;

  // Enable the NVIC interrupt
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // 4-bit for preemption priority. 0-bits for subpriority
  NVIC_EnableIRQ(ADC1_2_IRQn);
  NVIC_SetPriority(ADC1_2_IRQn, 8U);

  // Scan mode ADC_CR1->SCAN=1. Scans all channels in ADC_SQRx. DMA must be set
  // Discontinuous mode: ADC_CR1->DISCEN=1 and ADC_CR1->DISCNUM[2:0] n channels per trigger.

  // Temperature sensor: ADCx_IN16. Sample time = 17.1us. ADC_CR2->TSVREFE=1 (ADCx_IN17 Vrefint)
}

int main(int argc, char* argv[]) {
  PT_SETUP();
  PT_INIT(&pt_tick);
  PT_INIT(&pt_lcd_driver);
  PT_INIT(&pt_led);
  init_adc();
  init_tmr();
  // Schedule threads
  while (1) {
    PT_SCHEDULE(protothread_tick(&pt_tick));
    PT_SCHEDULE(protothread_lcd(&pt_lcd_driver));
    PT_SCHEDULE(protothread_led(&pt_led));
  }
}

#pragma GCC diagnostic pop
