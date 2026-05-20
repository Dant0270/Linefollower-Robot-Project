#ifndef UART_H
#define UART_H

#include "config.h"
#include <stdio.h>
#include <string.h>

class HardwareSerial {
  public: void begin(uint32_t baudrate) {
    RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA -> MODER &= ~((3 << (9 * 2)) | (3 << (10 * 2)));
    GPIOA -> MODER |= (2 << (9 * 2)) | (2 << (10 * 2));
    GPIOA -> AFR[1] |= (7 << ((9 - 8) * 4)) | (7 << ((10 - 8) * 4));

    USART1 -> CR1 = 0;
    USART1 -> BRR = 84000000 / baudrate;
    USART1 -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
  }

  void print(char ch) {
    while (!(USART1 -> SR & USART_SR_TXE));
    USART1 -> DR = ch;
  }
  void print(const char * str) {
    while ( * str) print( * str++);
  }
  void println(const char * str) {
    print(str);
    print("\r\n");
  }
  void print(int val) {
    char buf[16];
    sprintf(buf, "%d", val);
    print(buf);
  }
  void println(int val) {
    print(val);
    print("\r\n");
  }

  void print(float val, int precision = 2) {
    if (val < 0.0f) {
      print('-');
      val = -val;
    }
    int int_part = (int) val;
    float remainder = val - (float) int_part;
    print(int_part);
    if (precision > 0) print('.');
    for (int i = 0; i < precision; i++) {
      remainder *= 10.0f;
      int digit = (int) remainder;
      print((char)('0' + digit));
      remainder -= digit;
    }
  }
  void println(float val, int precision = 2) {
    print(val, precision);
    print("\r\n");
  }

  bool available() {
    return (USART1 -> SR & USART_SR_RXNE) != 0;
  }
  char read() {
    while (!available());
    return (char)(USART1 -> DR & 0xFF);
  }
};

extern HardwareSerial Serial;

#endif