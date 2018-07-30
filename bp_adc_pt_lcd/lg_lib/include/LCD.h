#ifndef __LCD_HD44780
#define __LCD_HD44780

//#ifdef __cplusplus
// extern "C" {
//#endif

#include "stm32f10x.h"
#include "pt-extended.h"

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

/**
 * LCD Register Select RS=0 - Command mode
 */
#define LCD_RS_CMD 0
/**
 * LCD Register Select RS=1 - Data mode
 */
#define LCD_RS_DATA 1

// PT Macros for library users

/**
 * Initialise the LCD
 * \param[in] pt protothread pointer
 */
#define PT_LCD_INIT(pt)          PT_SPAWN(pt, &lcd::pt_init, lcd::init(&lcd::pt_init))
/**
 * Print a string on LCD
 * \param[in] pt parent protothread pointer
 * \param[in] str String to display
 */
#define PT_LCD_PRINTSTR(pt, str) PT_SPAWN(pt, &lcd::pt_printstr, lcd::printStr(&lcd::pt_printstr, str))
/**
 * Cmd: 0000 0001 Clear LCD display
 * \param[in] pt parent protothread pointer
 */
#define PT_LCD_CLEARDISPLAY(pt)  PT_SPAWN(pt, &lcd::pt_send, lcd::send(&lcd::pt_send,0x01,LCD_RS_CMD,2))
/**
 * Cmd: 1100 0000 Set LCD cursor on the second line (when N=1 2-line mode)
 * \param[in] pt parent protothread pointer
 *
 * Set DDRAM Addr AC6=1 AC[5..0]=0. Addr=0100 000 = 0x40
 * When N=1 2-line
 *   Line1: 0x00 - 0x27
 *   Line2: 0x40 - 0x67
 */
#define PT_LCD_2NDLINE(pt)       PT_SPAWN(pt, &lcd::pt_send, lcd::send(&lcd::pt_send,0xC0,LCD_RS_CMD,0))
/**
 * Cmd: 0000 0010 Set LCD cursor to Home position [0,0]
 */
#define PT_LCD_HOME(pt)          PT_SPAWN(pt, &lcd::pt_send, lcd::send(&lcd::pt_send,0x02,LCD_RS_CMD,2))
/**
 * Cmd: 0001 1000 Cursor or display shift left
 * \param[in] pt parent protothread pointer
 *
 * S/C=1 Display & Cursor (When S/C=0 only cursor is shifted)
 * R/L=0 Left
 */
#define PT_LCD_LSHIFT(pt)        PT_SPAWN(pt, &lcd::pt_send, lcd::send(&lcd::pt_send,0x18,LCD_RS_CMD,0))
/**
 * Cmd: 0001 1100 Cursor or display shift right
 * \param[in] pt parent protothread pointer
 *
 * S/C=1 Display & Cursor (When S/C=0 only cursor is shifted)
 * R/L=1 Right
 */
#define PT_LCD_RSHIFT(pt)        PT_SPAWN(pt, &lcd::pt_send, lcd::send(&lcd::pt_send,0x1C,LCD_RS_CMD,0))
/**
 * Cmd: 0000 1100 Turn LCD display on
 * \param[in] pt parent protothread pointer
 *
 * D=1 Display ON
 * C=0 Cursor OFF
 * B=0 Cursor Blinking OFF
 */
#define PT_LCD_DISPLAYON(pt)     PT_SPAWN(pt, &lcd::pt_send, lcd::send(&lcd::pt_send,0x0C,LCD_RS_CMD,0))
/**
 * Cmd: 0000 1100 Turn LCD display off
 * \param[in] pt parent protothread pointer
 *
 * D=0 Display OFF
 * C=0 Cursor OFF
 * B=0 Cursor Blinking OFF
 */
#define PT_LCD_DISPLAYOFF(pt)    PT_SPAWN(pt, &lcd::pt_send, lcd::send(&lcd::pt_send,0x08,LCD_RS_CMD,0))
/**
 * Set Cursor to a specified location given by row and column information
 * \param[in] pt protothread pointer
 * \param[in] row Number (1 to 2)
 * \param[in] column Number (1 to 16) Assumes a 2 X 16 character display
 */
#define PT_LCD_LOCATE(pt, row, column)   PT_SPAWN(pt, &lcd::pt_locate, lcd::locate(&lcd::pt_locate, row, column))
/**
 * Print character on LCD
 * \param[in] pt protothread pointer
 */
#define PT_LCD_PRINTCHAR(pt, character)  PT_SPAWN(pt, &lcd::pt_send, lcd::send(&lcd::pt_send,character,LCD_RS_DATA,0))

namespace lcd {
  static struct pt pt_init, pt_send, pt_printstr, pt_locate;
  void configPin(GPIO_TypeDef * port, uint8_t pin, uint8_t cnf_and_mode);
  void configPins(void);
  uint32_t read4BitData(void);
  void write4BitData(uint8_t data);
  PT_THREAD (send(struct pt *pt, uint8_t byte_to_send, uint8_t rs, uint8_t post_delay));
  PT_THREAD (init(struct pt *pt));
  PT_THREAD (printStr (struct pt *pt, const char *text));
  PT_THREAD (locate (struct pt *pt, uint8_t row, uint8_t column));
}

//#ifdef __cplusplus
//}
//#endif

#endif //__LCD_HD44780
