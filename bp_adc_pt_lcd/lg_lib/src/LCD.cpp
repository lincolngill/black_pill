/* HD44780 LCD Driver
 *
 * */
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough="

#include <LCD.h>

/**
 * \namespace lcd
 * \brief LCD protothread library
 */
namespace lcd {

  /**
   * Configure a GPIO pin's MODE and CNF and enable the port clock
   * \param[in] port         GPIO port. E.g. GPIOA
   * \param[in] pin          Pin number: 0-15
   * \param[in] cnf_and_mode CNF[1:0] and MODE[1:0] as 4-bit number. E.g. 0b0011 (CNF=00 MODE=11)
   */
  void
  configPin (GPIO_TypeDef * port, uint8_t pin, uint8_t cnf_and_mode) {
    uint32_t pin_mask;
    volatile uint32_t * cr_reg;

    // Enable the port clock
    if (port == GPIOA)
      RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    if (port == GPIOB)
      RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    // Determine if we need CRL or CRH reg and calculate the location mask for the pins MODE[0] bit
    if (pin < 8) {
      cr_reg = &(port->CRL);                  // Need to use CRL
      pin_mask = (uint32_t) (1 << (pin * 4)); // CRL MODE[0] mask for pin
    } else {
      cr_reg = &(port->CRH);                        // Need to use CRH
      pin_mask = (uint32_t) (1 << ((pin - 8) * 4)); // CRH MODE[0] mask for pin
    }

    // Set or Reset CNF[1:0]MODE[1:0]
    for (int i = 0; i < 4; i++) {
      (cnf_and_mode & 1 << i) ?
          (*cr_reg |= pin_mask << i) : (*cr_reg &= ~(pin_mask << i));
    }
  }

  /**
   * Configure the LCD Pins for output
   */
  void
  configPins (void) {
    // Config pins
    // CNF[1:0]=01  - General purpose output Open-drain
    // MODE[1:0]=11 - 50 MHz
    configPin (LCD_D4_PORT, LCD_D4_PIN, 0b0111);
    configPin (LCD_D5_PORT, LCD_D5_PIN, 0b0111);
    configPin (LCD_D6_PORT, LCD_D6_PIN, 0b0111);
    configPin (LCD_D7_PORT, LCD_D7_PIN, 0b0111);
    // CNF[1:0]=00  - General purpose output Push-pull
    configPin (LCD_ENABLE_PORT, LCD_ENABLE_PIN, 0b0011);
    configPin (LCD_RS_PORT, LCD_RS_PIN, 0b0011);
  }

  /**
   * Reads the output register bits for pins D[4:7] and stitches together a 4-bit data value
   * \return 4-bit data value
   */
  uint32_t
  read4BitData (void) {
    return (LCD_D4_PORT->ODR & 1 << LCD_D4_PIN)
        + ((LCD_D5_PORT->ODR & 1 << LCD_D5_PIN) << 1)
        + ((LCD_D6_PORT->ODR & 1 << LCD_D6_PIN) << 2)
        + ((LCD_D7_PORT->ODR & 1 << LCD_D7_PIN) << 3);
  }

  /**
   * Writes the LCDs 4-bit data to D[4:7] pins
   * \param[in] data 4-bit data value
   */
  void
  write4BitData (uint8_t data) {
    (data & 0b0001) ?
        LCD_SET_PIN(LCD_D4_PORT, LCD_D4_PIN) :
        LCD_RESET_PIN(LCD_D4_PORT, LCD_D4_PIN);
    (data & 0b0010) ?
        LCD_SET_PIN(LCD_D5_PORT, LCD_D5_PIN) :
        LCD_RESET_PIN(LCD_D5_PORT, LCD_D5_PIN);
    (data & 0b0100) ?
        LCD_SET_PIN(LCD_D6_PORT, LCD_D6_PIN) :
        LCD_RESET_PIN(LCD_D6_PORT, LCD_D6_PIN);
    (data & 0b1000) ?
        LCD_SET_PIN(LCD_D7_PORT, LCD_D7_PIN) :
        LCD_RESET_PIN(LCD_D7_PORT, LCD_D7_PIN);
  }

  /**
   * Thread to Send a byte to the LCD in either command mode or data mode
   * \param[in] pt           protothread pointer
   * \param[in] byte_to_send byte to send to the LCD
   * \param[in] rs           Register select: LCD_RS_CMD or LCD_RS_DATA
   * \param[in] post_delay   Extra delay (ms) at end of Enable
   */
  PT_THREAD (send(struct pt *pt, uint8_t byte_to_send, uint8_t rs, uint8_t post_delay)) {
    PT_BEGIN(pt);
    // Register Select
    (rs == LCD_RS_CMD) ? LCD_RESET_RS : LCD_SET_RS ;
    // Send high order 4-bits
    write4BitData ((byte_to_send >> 4) & 0x0F);
    // Enable sequence
    LCD_SET_ENABLE;
    PT_YIELD_TIME_msec(pt, 2);
    LCD_RESET_ENABLE;
    // Send low order 4-bits
    write4BitData (byte_to_send & 0x0F);
    // Enable sequence
    LCD_SET_ENABLE;
    PT_YIELD_TIME_msec(pt, 2);
    LCD_RESET_ENABLE;
    // Post enable wait: min 2ms + post_delay (ms)
    PT_YIELD_TIME_msec(pt, 2+post_delay);
    PT_END(pt);
  }

  /**
   * Thread to Initialise the LCD - Configure the pins and send the initialise commands
   * \param[in] pt protothread pointer
   */
  PT_THREAD (init(struct pt *pt)) {
    PT_BEGIN(pt);
    configPins ();
    LCD_RESET_ENABLE;
    LCD_RESET_RS;
    //Initialization of HD44780-based LCD (4-bit HW)
    // 0011 0011 Function set: D[4..5]=11 twice. DL=1 8-bit mode
    PT_SPAWN(pt, &pt_send, send(&pt_send,0x33,LCD_RS_CMD,4));
    // 0011 0010 Function set: D[4..5]=11 DL=1, then D[4..5]=10 DL=0 4-bit mode
    PT_SPAWN(pt, &pt_send, send(&pt_send,0x32,LCD_RS_CMD,4));
    //Function Set 4-bit mode
    // 0010 1000 Function set DL=0 4-bit, N=1 2-line, F=0 5x8 dots
    PT_SPAWN(pt, &pt_send, send(&pt_send,0x28,LCD_RS_CMD,0));
    //Display On/Off Control
    // 0000 1100 Display ON/OFF D=1 Display ON, C=0 Cursor OFF, B=0 Cursor blink OFF
    PT_SPAWN(pt, &pt_send, send(&pt_send,0x0C,LCD_RS_CMD,0));
    //Entry mode set
    // 0000 0110 Entry Mode
    // I/D=1 Cursor/blink moves right & DDRAM addr inc
    // SH=0 Display shift OFF
    PT_SPAWN(pt, &pt_send, send(&pt_send,0x06,LCD_RS_CMD,0));
    // Clear display
    // Minimum delay to wait before driving LCD module
    PT_SPAWN(pt, &pt_send, send(&pt_send,0x01,LCD_RS_CMD,200));
    PT_END(pt);
  }

  /**
   * Thread to Display a character string on LCD
   * \param[in] pt   protothread pointer
   * \param[in] text string to send to LCD
   */
  PT_THREAD (printStr (struct pt *pt, const char *text)) {
    PT_BEGIN(pt);
    static int i;
    i = 0;
    while (text[i]) {
      PT_SPAWN(pt, &pt_send, send(&pt_send,text[i],LCD_RS_DATA,0));
      i++;
    }
    PT_END(pt);
  }

  /**
   * Set Cursor to a specified location given by row and column information
   * \param[in] row    Number (1 to 2)
   * \param[in] column Number (1 to 16) Assumes a 2 X 16 character display
   */
  PT_THREAD (locate (struct pt *pt, uint8_t row, uint8_t column)) {
    PT_BEGIN(pt);
    // Use column parameter as DDRAM Addr command
    column--;
    if (row == 1) {
      // 1nnn nnnn Set DDRAM Addr
      column |= 0x80;
    }
    if (row == 2) {
      // 11nn nnnn Line 2
      column |= 0x40 | 0x80;
    }
    PT_SPAWN(pt, &pt_send, send(&pt_send,column,LCD_RS_CMD,0));
    PT_END(pt);
  }
}
