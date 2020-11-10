#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/f4/nvic.h>


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "mpu_6050.h"
#include "motor_control.h"
#include "auxiliary.h"

static void clock_setup(void) {
	// set clock to 84 MHz
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[0]);

	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM3);

	rcc_periph_clock_enable(RCC_I2C1);

	rcc_periph_clock_enable(RCC_USART1);
}

static void gpio_setup(void) {

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

	// motor control
	// PWM pin TIM3_CH1
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6);
	gpio_set_af(GPIOA, GPIO_AF2, GPIO6);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO6);
	// motor enable
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO2);
	gpio_set(GPIOB, GPIO2);
	set_motor_enable_pin(GPIOB, GPIO2);
	// motor direction
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO7);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO7);
	gpio_clear(GPIOA, GPIO7);
	set_motor_direction_pin(GPIOA, GPIO7);
	// half step activate
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO1);
	gpio_set(GPIOB, GPIO1);
	set_motor_half_step_pin(GPIOB, GPIO1);

	// I2C pins for MPU6050
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO8);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO8);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD,GPIO_OSPEED_50MHZ, GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO9);

	// MPU6050 interrupt
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
}

// the parameter is to make it easier to deactivate the watchdog for debugging
static void iwdg_setup(uint8_t do_it) {
	if(do_it) {
		iwdg_set_period_ms(60);
		iwdg_start();
	}
}

static float calculate_angle(uint8_t initial_calibration) {
	static double last_angle = 0.0;
	const int32_t x_accel_offset = 28;
	const int32_t z_accel_offset = -63;
	const int32_t y_gyro_offset = 202;
	const double sensor_sample_time = 1.0/1000.0;
	// the sensor measures from -250 degree/second to +250 degree/second
	// value returned is an int16_t (largest absolute value 32768) -> remap with this factor
	const double sensor_data_remap_factor = 250.0/32768.0;
	const double rad_to_degree_factor = 360.0/(2.0*M_PI);

	int32_t mpu_accel_x_calibrated = mpu_accel_x_val - x_accel_offset;
	int32_t mpu_accel_z_calibrated = mpu_accel_z_val - z_accel_offset;
	int32_t mpu_gyro_y_calibrated = mpu_gyro_y_val - y_gyro_offset;

	// use acceleration information to calculate initial angle because it's an absolute measurement
	// the acceleration information is not accurate after the robot starts moving -> use gyro
	if(initial_calibration) {
		last_angle = atan((double)mpu_accel_x_calibrated/mpu_accel_z_calibrated)*rad_to_degree_factor;
		return (float)last_angle;
	} else {
		// not sure, why we have to multiply by 2.0. But we do :)
		double angle_change = 2.0*(mpu_gyro_y_calibrated*sensor_data_remap_factor)*sensor_sample_time;
		double gyro_angle = last_angle - angle_change;
		last_angle = gyro_angle;
		if(isnan(last_angle)) {
			return 0.0F;
		}
		return (float)last_angle;
	}
}

static float low_pass(float angle) {
	static float filtered = 0.0F;
	float c = 1.0F;
	filtered = c*angle + (1.0F-c)*filtered;
	return filtered;
}

static void calculate_PID(float angle) {

	float output = 0.0F;
	float error = 0.0F;

	const float set_point = -0.5F;
	float Kp = -1.89F;
	float Ki = -0.0053F;
	float Kd = 0.0;//05F;

	static float integral = 0.0F;

	float differential = 0.0F;
	static float last_val = 0.0F;


	error = set_point - angle;

	integral = integral + error;

	differential = last_val-error;
	last_val = error;

	output = Kp*error + Ki*integral + Kd*differential;

	set_speed(output);
}


int main(void) {
    clock_setup();
	gpio_setup();
	init_auxiliary(TIM2, USART1);
	init_motor_control(TIM3);
	set_speed(0.0F);
	iwdg_setup(1);
	mpu_6050_setup(I2C1, 0x0);

	// the senors first few values are not accurate sometimes
	// so we wait for a bit before we start controling the motor
	uint32_t sensor_warm_up_counter = 0x0;

	while(1) {
		iwdg_reset();

		// SENSOR_DATA_READY_FLAG indicates new data was set in the sensor interrupt
		if(SENSOR_DATA_READY_FLAG == 0x1) {
			if(sensor_warm_up_counter >200) {
				calculate_PID(low_pass(calculate_angle(0)));
			} else {
				calculate_angle(1);
				++sensor_warm_up_counter;
			}
			SENSOR_DATA_READY_FLAG = 0x0;
		}
	}
}

void hard_fault_handler(void) {
	printf("hardfault\n");
	while(1);
}

void nmi_handler(void) {
	printf("nmi\n");
}