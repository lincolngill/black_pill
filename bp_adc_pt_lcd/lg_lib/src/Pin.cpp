/*
 * Pin.cpp
 *
 *  Created on: 2/08/2018
 *      Author: links
 */

#include <Pin.h>

void Pin::config(GPIO_TypeDef * port, uint8_t pin, uint8_t cnf_and_mode) {
  uint8_t lshift;
  volatile uint32_t * cr_reg;

  // Enable the port clock
  if (port == GPIOA)
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  if (port == GPIOB)
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

  // Determine if we need CRL or CRH reg and calculate the location mask for the pins MODE[0] bit
  if (pin < 8) {
    cr_reg = &(port->CRL);                  // Need to use CRL
    lshift = (uint8_t) (pin * 4);
  } else {
    cr_reg = &(port->CRH);                  // Need to use CRH
    lshift = (uint8_t) ((pin - 8) * 4);
  }
  // Reset CNF=00 and MODE=00
  *cr_reg &= ~(0b1111 << lshift);
  // Set CNF and MODE
  *cr_reg |= (uint32_t) (cnf_and_mode << lshift);
}

Pin::Pin (GPIO_TypeDef * GPIO_port, uint8_t pin_num, uint8_t cnf_and_mode_bits) {
  port = GPIO_port;
  pin = pin_num;
  config(port, pin, cnf_and_mode_bits);
}

void Pin::toggle() {
  port->ODR ^= (1<<pin);
}

void Pin::set() {
  port->BSRR |= (1<<pin);
}

void Pin::reset() {
  port->BRR |= (1<<pin);
}

