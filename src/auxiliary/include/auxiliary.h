#ifndef AUXILIARY_H
#define AUXILIARY_H
#include <stdint.h>

int _write(int file, char *ptr, int len);
uint8_t init_auxiliary(const uint32_t delay_timer, const uint32_t usart_console);
void delay_us(const uint32_t us);

#endif