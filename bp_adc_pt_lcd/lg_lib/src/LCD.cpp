/* HD44780 LCD Driver
 *
 * */
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough="

#include <LCD.h>
#include <Pin.h>

/**
 * \namespace lcd
 * \brief LCD protothread library
 */
namespace lcd {

  Pin data_pins[4] = {
    Pin(LCD_D4_PORT, LCD_D4_PIN, PIN_OPENDRAIN_OUTPUT_50),
    Pin(LCD_D5_PORT, LCD_D5_PIN, PIN_OPENDRAIN_OUTPUT_50),
    Pin(LCD_D6_PORT, LCD_D6_PIN, PIN_OPENDRAIN_OUTPUT_50),
    Pin(LCD_D7_PORT, LCD_D7_PIN, PIN_OPENDRAIN_OUTPUT_50)
  };
  Pin enable_pin(LCD_ENABLE_PORT, LCD_ENABLE_PIN, PIN_PUSHPULL_OUTPUT_50);
  Pin rs_pin(LCD_RS_PORT, LCD_RS_PIN, PIN_PUSHPULL_OUTPUT_50);

  /**
   * Writes the LCDs 4-bit data to D[4:7] pins
   * \param[in] data 4-bit data value
   */
  void
  write4BitData (uint8_t data) {
    for (uint8_t i=0; i<4; i++) {
      (data & 1<<i) ? data_pins[i].set() : data_pins[i].reset();
    }
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
    (rs == LCD_RS_CMD) ? rs_pin.reset() : rs_pin.set() ;
    // Send high order 4-bits
    write4BitData ((byte_to_send >> 4) & 0x0F);
    // Enable sequence
    enable_pin.set();
    PT_YIELD_TIME_msec(pt, 2);
    enable_pin.reset();
    // Send low order 4-bits
    write4BitData (byte_to_send & 0x0F);
    // Enable sequence
    enable_pin.set();
    PT_YIELD_TIME_msec(pt, 2);
    enable_pin.reset();
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
    enable_pin.reset();
    rs_pin.reset();
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
