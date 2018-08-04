/*
 * Pin.h
 *
 *  Created on: 2/08/2018
 *      Author: links
 */

#ifndef INCLUDE_PIN_H_
#define INCLUDE_PIN_H_

#include "stm32f10x.h"

#define PIN_ANALOG_INPUT        0b0000
#define PIN_OPENDRAIN_OUTPUT_50 0b0111
#define PIN_PUSHPULL_OUTPUT_50  0b0011

class Pin {
    GPIO_TypeDef * port;
    uint8_t pin;
  public:
    static void config(GPIO_TypeDef * port, uint8_t pin, uint8_t cnf_and_mode);
    Pin (GPIO_TypeDef * port, uint8_t pin, uint8_t cnf_and_mode);
    void toggle();
    void set();
    void reset();
};

#endif /* INCLUDE_PIN_H_ */
