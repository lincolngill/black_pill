#ifndef __LCD_HD44780
#define __LCD_HD44780

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"

/* LCD HW settings */
// 4-bit data
// *** These must be 5v tolerant pins
#define LCD_D4_PIN  8
#define LCD_D4_PORT GPIOA
#define LCD_D5_PIN  9
#define LCD_D5_PORT GPIOA
#define LCD_D6_PIN  10
#define LCD_D6_PORT GPIOA
#define LCD_D7_PIN  15
#define LCD_D7_PORT GPIOB

#define LCD_ENABLE_PIN  6
#define LCD_ENABLE_PORT GPIOB
#define LCD_RS_PIN      7
#define LCD_RS_PORT     GPIOB

#define LCD_SET_PIN(GPIOx,Pinx)    (GPIOx->BSRR |= 1<<Pinx)
#define LCD_RESET_PIN(GPIOx,Pinx)  (GPIOx->BRR |= 1<<Pinx)

#define LCD_SET_RS       LCD_SET_PIN(LCD_RS_PORT,LCD_RS_PIN)
#define LCD_RESET_RS     LCD_RESET_PIN(LCD_RS_PORT,LCD_RS_PIN)
#define LCD_SET_ENABLE   LCD_SET_PIN(LCD_ENABLE_PORT,LCD_ENABLE_PIN)
#define LCD_RESET_ENABLE LCD_RESET_PIN(LCD_ENABLE_PORT,LCD_ENABLE_PIN)

#define LCD_CGRAM_ADDR 0x40

void _ConfigPin(GPIO_TypeDef * port, uint8_t pin, uint8_t cnf_and_mode);
void LCD_ConfigPins(void);
uint32_t LCD_Read4BitData(void);
void LCD_Write4BitData(uint8_t data);
void LCD_Delay(uint32_t ms);
void LCD_Enable (void);
void LCD_Cmd(uint8_t cmd_data);
void LCD_ClearDisplay(void);
void LCD_Init(void);
void LCD_GotoLine2(void);
void LCD_Home(void);
void LCD_LShift(void);
void LCD_RShift(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
void LCD_Locate(uint8_t row, uint8_t column);
void LCD_PrintChar(char ascode);
void LCD_PrintStr(char *text);
void LCD_Printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif //__LCD_HD44780
