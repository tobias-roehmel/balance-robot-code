#include "motor_control.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <stdio.h>
#include <math.h>

static uint32_t TIMER_MOTOR_PWM = 0x0;

static uint32_t MOTOR_ENABLE_PORT = 0x0;
static uint16_t MOTOR_ENABLE_PIN = 0x0;

static uint32_t MOTOR_DIRECTION_PORT = 0x0;
static uint16_t MOTOR_DIRECTION_PIN = 0x0;

static uint32_t MOTOR_HALF_STEP_PORT = 0x0;
static uint16_t MOTOR_HALF_STEP_PIN = 0x0;

static void timer_setup(void) {

    /* Clock division and mode */
    TIM_CR1(TIMER_MOTOR_PWM) = TIM_CR1_CKD_CK_INT | TIM_CR1_CMS_EDGE;
	/* Prescaler */
	TIM_PSC(TIMER_MOTOR_PWM) = 6;//rcc_apb1_frequency/12000000;

	//printf("apb: %lu\n prsc:%lu\n", rcc_apb1_frequency, TIM_PSC(TIMER_MOTOR_PWM));
    // timer counts to this value
	TIM_ARR(TIMER_MOTOR_PWM) = 15000;

	TIM_EGR(TIMER_MOTOR_PWM) = TIM_EGR_UG;

	/* Output compare 1 mode and preload */
	TIM_CCMR1(TIMER_MOTOR_PWM) |= TIM_CCMR1_OC1M_PWM1;//| TIM_CCMR1_OC1PE;

	/* Polarity and state of output wave*/
	TIM_CCER(TIMER_MOTOR_PWM) |= TIM_CCER_CC1NP | TIM_CCER_CC1E;
	//TIM2_CCER |= TIM_CCER_CC1E;

	/* duty cycle */
	TIM_CCR1(TIMER_MOTOR_PWM) = 1000;

	/* ARR reload enable */
	TIM_CR1(TIMER_MOTOR_PWM) |= TIM_CR1_ARPE;

	/* Counter enable */
	TIM_CR1(TIMER_MOTOR_PWM) |= TIM_CR1_CEN;
}

uint8_t set_motor_enable_pin(const uint32_t port, const uint16_t pin) {
	MOTOR_ENABLE_PORT = port;
	MOTOR_ENABLE_PIN = pin;
	return 0;
}

uint8_t set_motor_direction_pin(const uint32_t port, const uint16_t pin) {
	MOTOR_DIRECTION_PORT = port;
	MOTOR_DIRECTION_PIN = pin;
	return 0;
}

uint8_t set_motor_half_step_pin(const uint32_t port, const uint16_t pin) {
	MOTOR_HALF_STEP_PORT = port;
	MOTOR_HALF_STEP_PIN = pin;
	return 0;
}

uint8_t init_motor_control(const uint32_t timer_motor_pwm) {

	if((MOTOR_DIRECTION_PORT == 0x0) || (MOTOR_DIRECTION_PIN == 0x0) || (MOTOR_ENABLE_PORT == 0x0) || (MOTOR_ENABLE_PIN == 0x0) || (MOTOR_HALF_STEP_PIN == 0x0) || (MOTOR_HALF_STEP_PIN == 0x0)) {
		printf("motor enable/direction pin not set\n");
		return 0xFF;
	}

    TIMER_MOTOR_PWM = timer_motor_pwm;
    timer_setup();
    return 0;
}

uint8_t set_speed(const float speed) {

	// motors can't go faster or slower
	const float min_speed = 4.0F;
	const float max_speed = 25.0F;
	// makes it possible to turn off motors with this funtion
	const float turn_motor_off_treshhold = 0.01F;

	// factors that relate speed and value for auto reload register (PWM)
	/*
		ARR = (12*10^6Hz * 0.9degree * 5cm * STEP)/(360degree * speed)
		ARR 		= auto reload register (PWM)
		12MHz 		= speed of PWM clock
		0.9degree 	= degrees per step of motor (half step mode)
		5cm 		= radius of wheels
		STEP		= 1 for half step mode, 2 for step mode, because full step is 1.8 degree
		360 degree	= for full circle
		speed		= desired track speed in cm/s

		everything constant except speed -> factors
	*/
	const float full_step_factor = 300000.0F;
	const float half_step_factor = 150000.0F;

	float absolute_speed = (float)fabs(speed);
	uint8_t sign_speed = 0x0;
	uint32_t new_arr_value = 0x0;

	if(speed > 0.0F) {
		sign_speed = 0x0;
	} else {
		sign_speed = 0x1;
	}

	if(absolute_speed > max_speed) {
		absolute_speed = max_speed;
	}

	if(absolute_speed < turn_motor_off_treshhold) {
		// turn off motor
		gpio_set(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
		return 0;
	} else {
		gpio_clear(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN);
	}

	if(absolute_speed < min_speed) {
		absolute_speed = min_speed;
	}

	// decide between half and full step
	if(absolute_speed < 12.5F) { // half step
		new_arr_value = (uint32_t)(half_step_factor/absolute_speed);
		// set half step pin
		gpio_set(MOTOR_HALF_STEP_PORT, MOTOR_HALF_STEP_PIN);
	} else { // full step
		new_arr_value = (uint32_t)(full_step_factor/absolute_speed);
		// clear half step pin
		gpio_clear(MOTOR_HALF_STEP_PORT, MOTOR_HALF_STEP_PIN);
	}

	// set motor direction depending on sign of speed
	if(sign_speed == 0x0) {
		gpio_set(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);
	} else {
		gpio_clear(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN);
	}

	TIM_ARR(TIMER_MOTOR_PWM) = new_arr_value;
	return 0;
}
