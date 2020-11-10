#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <stdint.h>
#include <libopencm3/stm32/timer.h>

uint8_t init_motor_control(const uint32_t timer_motor_pwm);
uint8_t set_speed(const float speed);
uint8_t set_motor_enable_pin(const uint32_t port, const uint16_t pin);
uint8_t set_motor_direction_pin(const uint32_t port, const uint16_t pin);
uint8_t set_motor_half_step_pin(const uint32_t port, const uint16_t pin);

#endif