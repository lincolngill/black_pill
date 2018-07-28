/* HD44780 LCD Driver
 *
 * */
#include <stdarg.h>
#include <stdio.h>
#include <LCD_HD44780.h>
#include "pt-extended.h"

/**
 * @brief Configure a GPIO pin's MODE and CNF and enable the port clock
 * @param GPIO port. E.g. GPIOA
 * @param Pin number: 0-15
 * @param CNF[1:0] and MODE[1:0] as 4-bit number. E.g. 0b0011 (CNF=00 MODE=11)
 */
void _ConfigPin(GPIO_TypeDef * port, uint8_t pin, uint8_t cnf_and_mode) {
  uint32_t pin_mask;
  volatile uint32_t * cr_reg;

  // Enable the port clock
  if (port == GPIOA) RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  if (port == GPIOB) RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  // Determine if we need CRL or CRH reg and calculate the location mask for the pins MODE[0] bit
  if (pin < 8) {
      cr_reg = &(port->CRL);                  // Need to use CRL
      pin_mask = (uint32_t) (1<<(pin*4));     // CRL MODE[0] mask for pin
  } else {
      cr_reg = &(port->CRH);                  // Need to use CRH
      pin_mask = (uint32_t) (1<<((pin-8)*4)); // CRH MODE[0] mask for pin
  }

  // Set or Reset CNF[1:0]MODE[1:0]
  for (int i=0; i<4; i++) {
      (cnf_and_mode & 1<<i) ? (*cr_reg |= pin_mask<<i) : (*cr_reg &= ~(pin_mask<<i)) ;
  }
}

/**
 * @brief Configure the LCD Pins for output
 */
void LCD_ConfigPins(void) {
  // Config pins
  // CNF[1:0]=01  - General purpose output Open-drain
  // MODE[1:0]=11 - 50 MHz
  _ConfigPin(LCD_D4_PORT,     LCD_D4_PIN,     0b0111);
  _ConfigPin(LCD_D5_PORT,     LCD_D5_PIN,     0b0111);
  _ConfigPin(LCD_D6_PORT,     LCD_D6_PIN,     0b0111);
  _ConfigPin(LCD_D7_PORT,     LCD_D7_PIN,     0b0111);
  // CNF[1:0]=00  - General purpose output Push-pull
  _ConfigPin(LCD_ENABLE_PORT, LCD_ENABLE_PIN, 0b0011);
  _ConfigPin(LCD_RS_PORT,     LCD_RS_PIN,     0b0011);
}

/**
 * @brief Reads the LCDs 4 data pins and stitches together a 4-bit data value
 * @return 4-bit data value
 */
uint32_t LCD_Read4BitData(void) {
  return
     (LCD_D4_PORT ->ODR & 1<<LCD_D4_PIN) +
    ((LCD_D5_PORT ->ODR & 1<<LCD_D5_PIN)<<1) +
    ((LCD_D6_PORT ->ODR & 1<<LCD_D6_PIN)<<2) +
    ((LCD_D7_PORT ->ODR & 1<<LCD_D7_PIN)<<3);
}

/**
 * @brief Writes the LCDs 4-bit data
 * @param 4-bit data value
 */
void LCD_Write4BitData(uint8_t data) {
  (data & 0b0001) ? LCD_SET_PIN(LCD_D4_PORT,LCD_D4_PIN) : LCD_RESET_PIN(LCD_D4_PORT,LCD_D4_PIN);
  (data & 0b0010) ? LCD_SET_PIN(LCD_D5_PORT,LCD_D5_PIN) : LCD_RESET_PIN(LCD_D5_PORT,LCD_D5_PIN);
  (data & 0b0100) ? LCD_SET_PIN(LCD_D6_PORT,LCD_D6_PIN) : LCD_RESET_PIN(LCD_D6_PORT,LCD_D6_PIN);
  (data & 0b1000) ? LCD_SET_PIN(LCD_D7_PORT,LCD_D7_PIN) : LCD_RESET_PIN(LCD_D7_PORT,LCD_D7_PIN);
}

// TODO: Change this to a more accurate delay
void LCD_Delay(uint32_t ms) {
  uint32_t wait_till = pt_msTicks + ms;
  while (wait_till > pt_msTicks) ;
}

/**
  * @brief  Activate Enable Pin from LCD module
  */
void LCD_Enable (void)
{
  LCD_SET_ENABLE ;
  LCD_Delay(2);
  LCD_RESET_ENABLE ;
}

/**
  * @brief  Command data sent to LCD module
  * @param  command value to be sent
  */
void LCD_Cmd(uint8_t cmd_data)
{
  LCD_RESET_RS ;
  LCD_Write4BitData( (cmd_data>>4) & 0x0F );
  LCD_Enable();
  LCD_Write4BitData( cmd_data & 0x0F );
  LCD_Enable();
  LCD_Delay(2);
}

/**
  * @brief  Clear LCD module display
  */
void LCD_ClearDisplay(void)
{
  LCD_Cmd(0x01);
  LCD_Delay(2);
}

void LCD_Init(void)
{
  LCD_ConfigPins();

  LCD_RESET_ENABLE;
  LCD_RESET_RS;
  //Initialization of HD44780-based LCD (4-bit HW)
  // 0011 0011 Function set: D[4..5]=11 twice. DL=1 8-bit mode
  LCD_Cmd(0x33);
  LCD_Delay(4);
  // 0011 0010 Function set: D[4..5]=11 DL=1, then D[4..5]=10 DL=0 4-bit mode
  LCD_Cmd(0x32);
  LCD_Delay(4);
  //Function Set 4-bit mode
  // 0010 1000 Function set DL=0 4-bit, N=1 2-line, F=0 5x8 dots
  LCD_Cmd(0x28);
  //Display On/Off Control
  // 0000 1100 Display ON/OFF D=1 Display ON, C=0 Cursor OFF, B=0 Cursor blink OFF
  LCD_Cmd(0x0C);
  //Entry mode set
  // 0000 0110 Entry Mode
  // I/D=1 Cursor/blink moves right & DDRAM addr inc
  // SH=0 Display shift OFF
  LCD_Cmd(0x06);
  LCD_ClearDisplay();
  //Minimum delay to wait before driving LCD module
  LCD_Delay(200);
}

/**
  * @brief  Set Cursor on second row 1st digit
  */
void LCD_GotoLine2(void)
{
  // 1100 0000 Set DDRAM Addr AC6=1 AC[5..0]=0. Addr=0100 000 = 0x40
  // When N=1 2-line
  //   Line1: 0x00 - 0x27
  //   Line2: 0x40 - 0x67
  LCD_Cmd(0xC0);
}

/**
  * @brief  Set Cursor to Home position
  */
void LCD_Home(void)
{
  // Return Home
  LCD_Cmd(0x02);
  LCD_Delay(2);
}

/**
  * @brief  Shift display to left
  */
void LCD_LShift(void)
{
  // 0001 1000 Cursor or display shift
  // S/C=1 Display & Cursor (When S/C=0 only cursor is shifted)
  // R/L=0 Left
  LCD_Cmd(0x18);
}

/**
  * @brief  Shift display to right
  * @param  Text to be displayed
  */
void LCD_RShift(void)
{
  // 0001 1100 Cursor or display shift
  // S/C=1 Display & Cursor (When S/C=0 only cursor is shifted)
  // R/L=1 Right
  LCD_Cmd(0x1C);
}

/**
  * @brief  Set Display On
  */
void LCD_DisplayOn(void)
{
  // 0000 1100 Display ON/OFF
  // D=1 Display ON
  // C=0 Cursor OFF
  // B=0 Cursor Blinking OFF
  LCD_Cmd(0x0C);
}

/**
  * @brief  Set Display Off
  */
void LCD_DisplayOff(void)
{
  // 0000 1000 Display ON/OFF
  // D=0 Display OFF
  // C=0 Cursor OFF
  // B=0 Cursor Blinking OFF
  LCD_Cmd(0x08);
}

/**
  * @brief  Set Cursor to a specified location given by row and column information
  * @param  Row Number (1 to 2)
  * @param  Column Number (1 to 16) Assuming a 2 X 16 characters display
  */
void LCD_Locate(uint8_t row, uint8_t column)
{
  column--;
  if (row == 1) {
      // 1nnn nnnn Set DDRAM Addr
      LCD_Cmd(column |= 0x80);
  }
  if (row == 2) {
      // 11nn nnnn Line 2
      LCD_Cmd(column |= 0x40 | 0x80);
  }
}

/**
  * @brief  Print Character on LCD module
  * @param  Ascii value of character
  */
void LCD_PrintChar(char ascode)
{
  LCD_SET_RS ;
  LCD_Write4BitData( (ascode>>4) & 0x0F );
  LCD_Enable();
  LCD_Write4BitData( ascode & 0x0F );
  LCD_Enable();
  LCD_Delay(2);
}

/**
  * @brief  Display of a characters string
  * @param  Text to be displayed
  */
void LCD_PrintStr(char *text)
{
  do
  {
    LCD_PrintChar(*text++);
  }
  while (*text != '\n');
}

/**
  * @brief  lcd printf function
  * @param  string with standard defined formats
  */
void LCD_Printf(const char *fmt, ...)
{
  //uint32_t i;
  int i, text_size;
  char letter;
  static char text_buffer[32];
  va_list args;

  va_start(args, fmt);
  text_size = vsprintf(text_buffer, fmt, args);

  // Process the string
  for (i = 0; i < text_size; i++)
  {
    letter = text_buffer[i];

    if (letter == 10)
      break;
    else
    {
      //if ((letter > 0x1F) && (letter < 0x80))
      if ((letter > 0x1F))
        LCD_PrintChar(letter);
    }
  }
}

