#include "mpu_6050.h"
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/gpio.h>

#include "auxiliary.h"

#include <stdio.h>
#include <math.h>

static const uint8_t MPU6050_ADDRESS = 0x68;

static const uint8_t SELF_TEST_X_ADDR = 0xD;
static const uint8_t SELF_TEST_Y_ADDR = 0xE;
static const uint8_t SELF_TEST_Z_ADDR = 0xF;
static const uint8_t SELF_TEST_A_ADDR = 0x10;
static const uint8_t SMPLRT_DIV_ADDR = 0x19;
static const uint8_t CONFIG_ADDR = 0x1A;
static const uint8_t GYRO_CONFIG_ADDR = 0x1B;
static const uint8_t ACCEL_CONFIG_ADDR = 0x1C;
static const uint8_t FIFO_EN_ADDR = 0x23;
static const uint8_t INT_PIN_CFG_ADDR = 0x37;
static const uint8_t INT_ENABLE_ADDR = 0x38;
static const uint8_t GYRO_XOUT_H_ADDR = 0x43;
static const uint8_t GYRO_XOUT_L_ADDR = 0x44;
static const uint8_t GYRO_YOUT_H_ADDR = 0x45;
static const uint8_t GYRO_YOUT_L_ADDR = 0x46;
static const uint8_t GYRO_ZOUT_H_ADDR = 0x47;
static const uint8_t GYRO_ZOUT_L_ADDR = 0x48;
static const uint8_t ACCEL_XOUT_H_ADDR = 0x3B;
static const uint8_t ACCEL_XOUT_L_ADDR = 0x3C;
static const uint8_t ACCEL_YOUT_H_ADDR = 0x3D;
static const uint8_t ACCEL_YOUT_L_ADDR = 0x3E;
static const uint8_t ACCEL_ZOUT_H_ADDR = 0x3F;
static const uint8_t ACCEL_ZOUT_L_ADDR = 0x40;
static const uint8_t SIGNAL_PATH_RESET_ADDR = 0x68;
static const uint8_t USER_CTRL_ADDR = 0x6A;
static const uint8_t PWR_MGMT_1_ADDR = 0x6B;
static const uint8_t PWR_MGMT_2_ADDR = 0x6C;
static const uint8_t FIFO_COUNT_H_ADDR = 0x72;
static const uint8_t FIFO_COUNT_L_ADDR = 0x73;
static const uint8_t FIFO_DATA_ADDR = 0x74;

static uint8_t SMPLRT_DIV_SHADOW = 0x0;
static uint8_t CONFIG_SHADOW = 0x0;
static uint8_t GYRO_CONFIG_SHADOW = 0x0;
static uint8_t ACCEL_CONFIG_SHADOW = 0x0;
static uint8_t FIFO_EN_SHADOW = 0x0;
static uint8_t INT_PIN_CFG_SHADOW = 0x0;
static uint8_t INT_ENABLE_SHADOW = 0x0;
static uint8_t SIGNAL_PATH_RESET_SHADOW = 0x68;
static uint8_t USER_CTRL_SHADOW = 0x0;
static uint8_t PWR_MGMT_1_SHADOW = 0x1<<0x6;
static uint8_t PWR_MGMT_2_SHADOW = 0x0;


static uint32_t I2C_PERIPH;

#define WRITE_MPU_6050_REG(NAME, SET_MASK, CLEAR_MASK) NAME##_SHADOW = (uint8_t)(NAME##_SHADOW & ~(CLEAR_MASK));\
                                      NAME##_SHADOW = (uint8_t)(NAME##_SHADOW|(SET_MASK));\
                                      uint8_t NAME##_VAL[2] = {NAME##_ADDR, NAME##_SHADOW};\
                                      i2c_transfer7(I2C_PERIPH, MPU6050_ADDRESS, NAME##_VAL, 2, 0, 0);

#define READ_MPU_6050_REG(NAME, RETURN_VAL) uint8_t NAME##_VAL[1] = {NAME##_ADDR};\
                                            i2c_transfer7(I2C_PERIPH, MPU6050_ADDRESS, NAME##_VAL, 1, &RETURN_VAL, 1);

#define READ_BACK_MPU_6050_REG(NAME) uint8_t NAME##_READ_BACK = 0x0;\
                                     i2c_transfer7(I2C_PERIPH, MPU6050_ADDRESS, NAME##_VAL, 1, &NAME##_READ_BACK, 1);

#define READ_BACK_CHECK(NAME) if(NAME##_READ_BACK!=NAME##_SHADOW) {printf("%s fault\n", #NAME);}

#define READ_BACK_AND_CHECK(NAME) READ_BACK_MPU_6050_REG(NAME)\
                                  READ_BACK_CHECK(NAME)

static uint8_t init_mpu_6050(uint32_t ic2_periph) {
    I2C_PERIPH = ic2_periph;
    return 0;
}

uint8_t set_smplrt_div(uint8_t sample_rate_div) {
    WRITE_MPU_6050_REG(SMPLRT_DIV, sample_rate_div, 0x0)
    READ_BACK_AND_CHECK(SMPLRT_DIV)
    return 0;
}

uint8_t enable_interrupt_pin(void) {
    WRITE_MPU_6050_REG(INT_PIN_CFG, 0x1<<0x2, 0x0)
    READ_BACK_AND_CHECK(INT_PIN_CFG)

    return 0;
}

uint8_t enable_interrupt_generation(const uint8_t type) {
    WRITE_MPU_6050_REG(INT_ENABLE, type, 0x0)
    READ_BACK_AND_CHECK(INT_ENABLE)
    return 0;
}

uint8_t set_digital_filter_value(const uint8_t filter_value) {
    if(filter_value > 0x6) {
        return 0xFF;
    }
    WRITE_MPU_6050_REG(CONFIG, filter_value<<0x0, 0x7);
    READ_BACK_AND_CHECK(CONFIG)
    return 0;
}

uint8_t enable_fifo_for_sensor(enum enable_fifo_sensor_enum sensor) {
    WRITE_MPU_6050_REG(FIFO_EN, sensor, 0x0)
    READ_BACK_AND_CHECK(FIFO_EN)
    return 0;
}

uint8_t enable_fifo(void) {
    WRITE_MPU_6050_REG(USER_CTRL, 0x1<<0x6, 0x0)
    READ_BACK_AND_CHECK(USER_CTRL)
    return 0;
}

uint8_t clear_fifo_buffer(void) {
    WRITE_MPU_6050_REG(USER_CTRL, 0x1<<2, 0x0)
    USER_CTRL_SHADOW = (uint8_t)(USER_CTRL_SHADOW & ~(0x1<<2));
    return 0;
}

uint8_t set_sensor_standby(enum set_sensor_standby_enum sensor) {
    WRITE_MPU_6050_REG(PWR_MGMT_2, sensor, 0x0);
    READ_BACK_AND_CHECK(PWR_MGMT_2)
    return 0;
}

uint8_t disable_sleep_mode(void) {
    WRITE_MPU_6050_REG(PWR_MGMT_1, 0x0, 0x1<<0x6);
    READ_BACK_AND_CHECK(PWR_MGMT_1)
    return 0;
}

uint8_t disable_temperature_sensor(void) {
    WRITE_MPU_6050_REG(PWR_MGMT_1, 0x1<<0x3, 0x0);
    READ_BACK_AND_CHECK(PWR_MGMT_1)
    return 0;
}
uint8_t select_clock_source(enum mpu_clock_source_enum clk_source) {
    if((clk_source > 0x7) | (clk_source == 0x6)) {
        return 0xFF;
    }
    WRITE_MPU_6050_REG(PWR_MGMT_1, clk_source<<0x0, 0x7);
    READ_BACK_AND_CHECK(PWR_MGMT_1)
    return 0;
}

uint16_t get_fifo_counter(void) {
    uint8_t fifo_counter_high = 0x0;
    uint8_t fifo_counter_low = 0x0;
    uint16_t fifo_val = 0x0;

    READ_MPU_6050_REG(FIFO_COUNT_H, fifo_counter_high)
    READ_MPU_6050_REG(FIFO_COUNT_L, fifo_counter_low)

    fifo_val |= (uint16_t)(((uint16_t)fifo_counter_high)<<8U);
    fifo_val = (uint16_t)(fifo_val | fifo_counter_low);
    return fifo_val;
}

uint8_t get_fifo_values(uint8_t* output, const uint16_t len) {
    if(len>1024) {
        return 0xFF;
    }
    uint8_t fifo_data_addr_holder = FIFO_DATA_ADDR;
	i2c_transfer7(I2C_PERIPH, MPU6050_ADDRESS, &fifo_data_addr_holder, 1, output, len);
    return 0;
}

uint8_t reset_device(void) {
    WRITE_MPU_6050_REG(PWR_MGMT_1, 0x1<<0x7, 0x0);
    PWR_MGMT_1_SHADOW = (uint8_t) (PWR_MGMT_1_SHADOW & ~(0x1<<0x7));
	delay_us(100);
    return 0;
}

uint8_t reset_signal_paths(void) {
    WRITE_MPU_6050_REG(SIGNAL_PATH_RESET, 0x7<<0x0, 0x0)
    SIGNAL_PATH_RESET_SHADOW &= (uint8_t) (SIGNAL_PATH_RESET_SHADOW & ~(0x7<<0x0));
    return 0;
}

uint8_t set_accel_full_scale_range(enum accel_full_scale_range_enum range) {
    WRITE_MPU_6050_REG(ACCEL_CONFIG, range<<0x3, 0x0)
    READ_BACK_AND_CHECK(ACCEL_CONFIG)
    return 0;
}
uint8_t set_gyro_full_scale_range(enum gyro_full_scale_range_enum range) {
    WRITE_MPU_6050_REG(GYRO_CONFIG, range<<0x3, 0x0)
    READ_BACK_AND_CHECK(GYRO_CONFIG)
    return 0;
}

int16_t read_gyro_sensor(enum GYRO_AXIS axis) {
    uint8_t gyro_value_high = 0x0;
    uint8_t gyro_value_low = 0x0;

    switch(axis) {
        case X_GYRO: {
            READ_MPU_6050_REG(GYRO_XOUT_H, gyro_value_high)
            READ_MPU_6050_REG(GYRO_XOUT_L, gyro_value_low)
        } break;
        case Y_GYRO: {
            READ_MPU_6050_REG(GYRO_YOUT_H, gyro_value_high)
            READ_MPU_6050_REG(GYRO_YOUT_L, gyro_value_low)
        } break;
        case Z_GYRO: {
            READ_MPU_6050_REG(GYRO_ZOUT_H, gyro_value_high)
            READ_MPU_6050_REG(GYRO_ZOUT_L, gyro_value_low)
        } break;
        default: {
            printf("default case\n");
            return 0;
        }
    }
    return (int16_t)((gyro_value_high << 0x8) | gyro_value_low);
}

int16_t read_accel_sensor(enum ACCEL_AXIS axis) {
    uint8_t accel_value_high = 0x1;
    uint8_t accel_value_low = 0x1;

    switch(axis) {
        case X_ACCEL: {
            READ_MPU_6050_REG(ACCEL_XOUT_H, accel_value_high)
            READ_MPU_6050_REG(ACCEL_XOUT_L, accel_value_low)
        } break;
        case Y_ACCEL: {
            READ_MPU_6050_REG(ACCEL_YOUT_H, accel_value_high)
            READ_MPU_6050_REG(ACCEL_YOUT_L, accel_value_low)
        } break;
        case Z_ACCEL: {
            READ_MPU_6050_REG(ACCEL_ZOUT_H, accel_value_high)
            READ_MPU_6050_REG(ACCEL_ZOUT_L, accel_value_low)
        } break;
        default: {
            printf("default case\n");
            return 0;
        }
    }
    return (int16_t)((accel_value_high << 0x8) | accel_value_low);
}

static uint8_t enable_self_test_gyro(enum GYRO_AXIS axis) {
    WRITE_MPU_6050_REG(GYRO_CONFIG, axis, 0x0)
    READ_BACK_AND_CHECK(GYRO_CONFIG)
    return 0;
}

static uint8_t disable_self_test_gyro(enum GYRO_AXIS axis) {
    WRITE_MPU_6050_REG(GYRO_CONFIG, 0x0, axis)
    READ_BACK_AND_CHECK(GYRO_CONFIG)
    return 0;
}


static uint8_t read_gyro_test_value(enum GYRO_AXIS axis) {
    uint8_t ft_val = 0x0;
    switch(axis) {
        case X_GYRO:{
            READ_MPU_6050_REG(SELF_TEST_X, ft_val)
        };break;
        case Y_GYRO: {
            READ_MPU_6050_REG(SELF_TEST_Y, ft_val)
        };break;
        case Z_GYRO: {
            READ_MPU_6050_REG(SELF_TEST_Z, ft_val)
        }break;
        default: {
            return 0xFF;
        }
    }
    // there is an error, if this is zero
    if((ft_val&0x1F) == 0x0) {
        printf("Gyro FT value can not be 0\n");
        return 0xFF;
    } else {
        return ft_val & 0x1F;
    }
}

static uint8_t enable_self_test_accel(enum ACCEL_AXIS axis) {
    WRITE_MPU_6050_REG(ACCEL_CONFIG, axis, 0x0)
    READ_BACK_AND_CHECK(ACCEL_CONFIG)
    return 0;
}

static uint8_t disable_self_test_accel(enum ACCEL_AXIS axis) {
    WRITE_MPU_6050_REG(ACCEL_CONFIG, 0x0UL, axis)
    READ_BACK_AND_CHECK(ACCEL_CONFIG)
    return 0;
}

static uint8_t read_accel_test_value(enum ACCEL_AXIS axis) {
    uint8_t ft_val_high = 0x0;
    uint8_t ft_val_low = 0x0;
    uint8_t ft_val = 0x0;
    switch(axis) {
        case X_ACCEL:{
            READ_MPU_6050_REG(SELF_TEST_X, ft_val_high)
            READ_MPU_6050_REG(SELF_TEST_A, ft_val_low)
            ft_val_low = (ft_val_low&0x30)>>0x4;
        } break;
        case Y_ACCEL: {
            READ_MPU_6050_REG(SELF_TEST_Y, ft_val_high)
            READ_MPU_6050_REG(SELF_TEST_A, ft_val_low)
            ft_val_low = (ft_val_low&0xC)>>0x2;
        } break;
        case Z_ACCEL: {
            READ_MPU_6050_REG(SELF_TEST_Z, ft_val_high)
            READ_MPU_6050_REG(SELF_TEST_A, ft_val_low)
            ft_val_low = (ft_val_low&0x3)>>0x0;
        } break;
        default: {
            return 0xFF;
        }
    }
    ft_val_high &= 0xE0;
    ft_val_high >>= 0x3;
    ft_val = ft_val_high | ft_val_low;
    //ft_val = ((((uint16_t)ft_val_high)&0xE0)>>0x3)|(uint16_t)ft_val_low;
    // there is an error, if this is zero
    if(ft_val == 0x0) {
        printf("Accel FT value can not be 0\n");
        return 0xFF;
    } else {
        return ft_val;
    }
}

uint8_t self_test(uint8_t verbose) {

    // test settings
    reset_device();
    disable_sleep_mode();
	set_smplrt_div(0x1);
	set_digital_filter_value(0x1);
    set_gyro_full_scale_range(GYRO_FS_SEL_250);
    set_accel_full_scale_range(ACCEL_AFS_SEL_8);
    delay_us(3500);

    // gyro self test
    uint8_t gyro_test_ft_value_x = read_gyro_test_value(X_GYRO);
    uint8_t gyro_test_ft_value_y = read_gyro_test_value(Y_GYRO);
    uint8_t gyro_test_ft_value_z = read_gyro_test_value(Z_GYRO);

    if((gyro_test_ft_value_x == 0xFF) || (gyro_test_ft_value_y == 0xFF) || (gyro_test_ft_value_z == 0xFF)) {
        return 0xFF;
    }

    int16_t gyro_value_test_disabled_x = read_gyro_sensor(X_GYRO);
    int16_t gyro_value_test_disabled_y = read_gyro_sensor(Y_GYRO);
    int16_t gyro_value_test_disabled_z = read_gyro_sensor(Z_GYRO);

    enable_self_test_gyro(X_GYRO);
    enable_self_test_gyro(Y_GYRO);
    enable_self_test_gyro(Z_GYRO);
    delay_us(3500);

    int16_t gyro_value_test_enabled_x = read_gyro_sensor(X_GYRO);
    int16_t gyro_value_test_enabled_y = read_gyro_sensor(Y_GYRO);
    int16_t gyro_value_test_enabled_z = read_gyro_sensor(Z_GYRO);


    disable_self_test_gyro(X_GYRO);
    disable_self_test_gyro(Y_GYRO);
    disable_self_test_gyro(Z_GYRO);

    // accel self test
    uint8_t accel_test_ft_value_x = read_accel_test_value(X_ACCEL);
    uint8_t accel_test_ft_value_y = read_accel_test_value(Y_ACCEL);
    uint8_t accel_test_ft_value_z = read_accel_test_value(Z_ACCEL);

    if((accel_test_ft_value_x == 0xFF) || (accel_test_ft_value_y == 0xFF) || (accel_test_ft_value_z == 0xFF)) {
        return 0xFF;
    }

    int16_t accel_value_test_disabled_x = read_accel_sensor(X_ACCEL);
    int16_t accel_value_test_disabled_y = read_accel_sensor(Y_ACCEL);
    int16_t accel_value_test_disabled_z = read_accel_sensor(Z_ACCEL);

    enable_self_test_accel(X_ACCEL);
    enable_self_test_accel(Y_ACCEL);
    enable_self_test_accel(Z_ACCEL);
    delay_us(3500);


    int16_t accel_value_test_enabled_x = read_accel_sensor(X_ACCEL);
    int16_t accel_value_test_enabled_y = read_accel_sensor(Y_ACCEL);
    int16_t accel_value_test_enabled_z = read_accel_sensor(Z_ACCEL);

    disable_self_test_accel(X_ACCEL);
    disable_self_test_accel(Y_ACCEL);
    disable_self_test_accel(Z_ACCEL);

    // calculate change from ft percentage
    float gyro_ft_value_x = 25.0F*131.0F*(float)pow(1.046, gyro_test_ft_value_x - 1);
    float gyro_ft_value_y = -25.0F*131.0F*(float)pow(1.046, gyro_test_ft_value_y - 1);
    float gyro_ft_value_z = 25.0F*131.0F*(float)pow(1.046, gyro_test_ft_value_z - 1);

    float change_from_ft_gyro_x = ((float)(gyro_value_test_enabled_x - gyro_value_test_disabled_x) - gyro_ft_value_x)/gyro_ft_value_x;
    float change_from_ft_gyro_y = ((float)(gyro_value_test_enabled_y - gyro_value_test_disabled_y) - gyro_ft_value_y)/gyro_ft_value_y;
    float change_from_ft_gyro_z = ((float)(gyro_value_test_enabled_z - gyro_value_test_disabled_z) - gyro_ft_value_z)/gyro_ft_value_z;

    float accel_ft_value_x = 4096.0F*0.34F*(float)pow(0.92/0.34, ((float)accel_test_ft_value_x - 1)/30);
    float accel_ft_value_y = 4096.0F*0.34F*(float)pow(0.92/0.34, ((float)accel_test_ft_value_y - 1)/30);
    float accel_ft_value_z = 4096.0F*0.34F*(float)pow(0.92/0.34, ((float)accel_test_ft_value_z - 1)/30);

    float change_from_ft_accel_x = ((float)(accel_value_test_enabled_x - accel_value_test_disabled_x) - accel_ft_value_x)/accel_ft_value_x;
    float change_from_ft_accel_y = ((float)(accel_value_test_enabled_y - accel_value_test_disabled_y) - accel_ft_value_y)/accel_ft_value_y;
    float change_from_ft_accel_z = ((float)(accel_value_test_enabled_z - accel_value_test_disabled_z) - accel_ft_value_z)/accel_ft_value_z;

    // printf("gyro disabled \n");
    // printf("x: %d\n", gyro_value_test_disabled_x);
    // printf("y: %d\n", gyro_value_test_disabled_y);
    // printf("z: %d\n", gyro_value_test_disabled_z);
    // printf("gyro enabled \n");
    // printf("x: %d\n", gyro_value_test_enabled_x);
    // printf("y: %d\n", gyro_value_test_enabled_y);
    // printf("z: %d\n", gyro_value_test_enabled_z);
    // printf("gyro_ft_x: %d\n", gyro_test_ft_value_x);
    // printf("gyro_ft_y: %d\n", gyro_test_ft_value_y);
    // printf("gyro_ft_z: %d\n", gyro_test_ft_value_z);

    // printf("accel disabled \n");
    // printf("x: %d\n", accel_value_test_disabled_x);
    // printf("y: %d\n", accel_value_test_disabled_y);
    // printf("z: %d\n", accel_value_test_disabled_z);
    // printf("accel enabled \n");
    // printf("x: %d\n", accel_value_test_enabled_x);
    // printf("y: %d\n", accel_value_test_enabled_y);
    // printf("z: %d\n", accel_value_test_enabled_z);
    // printf("accel_ft_x: %d\n", accel_test_ft_value_x);
    // printf("accel_ft_y: %d\n", accel_test_ft_value_y);
    // printf("accel_ft_z: %d\n", accel_test_ft_value_z);
    if(verbose == 0x1) {
        printf("\nshould be around 0.14 (exde)\n");
        printf("change ft gyro x: %f\n", (double)change_from_ft_gyro_x);
        printf("change ft gyro y: %f\n", (double)change_from_ft_gyro_y);
        printf("change ft gyro z: %f\n", (double)change_from_ft_gyro_z);

        printf("change ft accel x: %f\n", (double)change_from_ft_accel_x);
        printf("change ft accel y: %f\n", (double)change_from_ft_accel_y);
        printf("change ft accel z: %f\n", (double)change_from_ft_accel_z);
    }
    return 0;
}


static void exti_setup(void) {
	exti_enable_request(EXTI1);
	exti_select_source(EXTI1, GPIOA);
	exti_set_trigger(EXTI1, EXTI_RTSR);
	nvic_enable_irq(NVIC_EXTI1_IRQ);
}

volatile uint8_t SENSOR_DATA_READY_FLAG = 0x0;
volatile int16_t mpu_accel_x_val = 0;
volatile int16_t mpu_accel_y_val = 0;
volatile int16_t mpu_accel_z_val = 0;
volatile int16_t mpu_gyro_y_val = 0;

void exti1_isr(void) {

	uint8_t mpu_fifo_data_values[8] = {0};

	exti_reset_request(EXTI1);

    if(SENSOR_DATA_READY_FLAG == 0x0) {
        get_fifo_values(mpu_fifo_data_values, 8);
        mpu_accel_x_val = (int16_t) (mpu_fifo_data_values[0]<<8 | mpu_fifo_data_values[1]);
        mpu_accel_y_val = (int16_t) (mpu_fifo_data_values[2]<<8 | mpu_fifo_data_values[3]);
        mpu_accel_z_val = (int16_t) (mpu_fifo_data_values[4]<<8 | mpu_fifo_data_values[5]);
        mpu_gyro_y_val =  (int16_t) (mpu_fifo_data_values[6]<<8 | mpu_fifo_data_values[7]);

        SENSOR_DATA_READY_FLAG = 0x1;
    } else {
        printf("main too slow\n");
    }
}

static void i2c_periph_setup(void) {

	i2c_peripheral_disable(I2C_PERIPH);
	i2c_set_clock_frequency(I2C_PERIPH, I2C_CR2_FREQ_42MHZ);
	i2c_disable_ack(I2C_PERIPH);
	i2c_set_speed(I2C_PERIPH, i2c_speed_sm_100k, 42);
	i2c_set_speed(I2C_PERIPH, i2c_speed_fm_400k, 42);
	i2c_peripheral_enable(I2C_PERIPH);
}

uint8_t mpu_6050_setup(const uint32_t i2c_periph, uint8_t verbose) {
	// order of these calls is important since some things only work after sleep mode is disabled
	init_mpu_6050(i2c_periph);
    i2c_periph_setup();
	if(self_test(verbose) != 0) {
		printf("Self test Error\n");
        return 0xFF;
	}

	reset_device();
    select_clock_source(CLOCK_GYRO_Z);
	enable_interrupt_pin();
	enable_interrupt_generation(DATA_RDY_EN);
	clear_fifo_buffer();
	set_sensor_standby(STBY_XG);
	disable_temperature_sensor();
    set_gyro_full_scale_range(GYRO_FS_SEL_250);
    set_accel_full_scale_range(ACCEL_AFS_SEL_2);
	disable_sleep_mode();
	set_digital_filter_value(0x6);
	set_smplrt_div(0x1);
	enable_fifo();
	enable_fifo_for_sensor(YG_FIFO_EN|ACCEL_FIFO_EN);
    exti_setup();
    return 0;
}