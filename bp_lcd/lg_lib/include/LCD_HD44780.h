#ifndef __LCD_HD44780
#define __LCD_HD44780

/* LCD HW settings */
/* LCDPort contains 4-bit data D0 to D3*/
/* LCDPort must be assigned to the chosen port */
#define LCDPort GPIOB
/* Pins E and RS of LCD must be assigned to LCDControlPort*/
#define LCDControlPort GPIOC
/* Define port where LCD Power is connected */
#define LCDPwrPort GPIOC
/* LCD Power Supply pin is assigned to Px5 */
#define LCDPwrPin GPIO_PIN_5
/* LCD Enable pin is assigned to Px1 */
#define LCD_Enable GPIO_PIN_1
/* LCD RS pin is assigned to Px2 */
#define LCD_RS GPIO_PIN_2
/* HD44780 CGRAM address start */
#define CGRAM_address_start 0x40

uint16_t GPIO_ReadOutputData (GPIO_TypeDef *GPIOx);
void     GPIO_Write (GPIO_TypeDef *GPIOx, uint16_t PortVal);

void LCD_LOAD_CGRAM(char tab[], uint8_t charnum);
void LCD_ENABLE (void);
void LCD_CMD(unsigned char cmd_data);
void LCD_CLEAR_DISPLAY(void);
void LCD_INIT(void);
void LCD_2ndROW(void);
void LCD_HOME(void);
void LCD_LSHIFT(void);
void LCD_RSHIFT(void);
void LCD_DISP_ON(void);
void LCD_DISP_OFF(void);
void LCD_LOCATE(uint8_t row, uint8_t column);

void LCD_printstring(char *text);
void LCD_printchar(unsigned char ascode);
void LCD_printf(const char *fmt, ...);

#endif
