#ifndef MPU_6050_H
#define MPU_6050_H

#include <stdint.h>

extern volatile int16_t mpu_accel_x_val;
extern volatile int16_t mpu_accel_y_val;
extern volatile int16_t mpu_accel_z_val;
extern volatile int16_t mpu_gyro_y_val;
extern volatile uint8_t SENSOR_DATA_READY_FLAG;

static const uint8_t FIFO_OFLOW_EN = 0x1<<0x4;
static const uint8_t DATA_RDY_EN = 0x1<<0x0;

// enable writing sensor data to fifo
enum enable_fifo_sensor_enum {
    TEMP_FIFO_EN = 0x1<<0x7,
    XG_FIFO_EN = 0x1<<0x6,
    YG_FIFO_EN = 0x1<<0x5,
    ZG_FIFO_EN = 0x1<<0x4,
    ACCEL_FIFO_EN = 0x1<<0x3
};

// set sensor to standby
enum set_sensor_standby_enum {
    STBY_ZG = (0x1<<0x0),
    STBY_YG = (0x1<<0x1),
    STBY_XG = (0x1<<0x2),
    STBY_ZA = (0x1<<0x3),
    STBY_YA = (0x1<<0x4),
    STBY_XA = (0x1<<0x5),
};

// clock sources
 enum mpu_clock_source_enum{
    CLOCK_INTERNAL_8MHZ = 0x0U,
    CLOCK_GYRO_X= 0x1U,
    CLOCK_GYRO_Y = 0x2U,
    CLOCK_GYRO_Z = 0x3U,
    CLOCK_EXTERNAL_32KHZ = 0x4U,
    CLOCK_EXTERNAL_19MHZ = 0x5U,
    CLOCK_RESET = 0x7U
};

// gyro full scale ranges
enum gyro_full_scale_range_enum {
    GYRO_FS_SEL_250 = 0x0U,
    GYRO_FS_SEL_500 = 0x1U,
    GYRO_FS_SEL_1000 = 0x2U,
    GYRO_FS_SEL_2000 = 0x3U,
};

// accel full scale ranges
enum accel_full_scale_range_enum {
    ACCEL_AFS_SEL_2 = 0x0U,
    ACCEL_AFS_SEL_4 = 0x1U,
    ACCEL_AFS_SEL_8 = 0x2U,
    ACCEL_AFS_SEL_16 = 0x3U,
};
// static const uint8_t ACCEL_AFS_SEL_2 = 0x0;
// static const uint8_t ACCEL_AFS_SEL_4 = 0x1;
// static const uint8_t ACCEL_AFS_SEL_8 = 0x2;
// static const uint8_t ACCEL_AFS_SEL_16 = 0x3;

enum GYRO_AXIS {
    Z_GYRO = 0x20U,
    Y_GYRO = 0x40U,
    X_GYRO = 0x80U,
};

enum ACCEL_AXIS {
    Z_ACCEL = 0x20U,
    Y_ACCEL = 0x40U,
    X_ACCEL = 0x80U,
};


uint8_t mpu_6050_setup(const uint32_t i2c_periph, uint8_t verbose);
uint8_t set_smplrt_div(const uint8_t sampe_rate);
uint8_t enable_interrupt_pin(void);
uint8_t enable_interrupt_generation(const uint8_t type);
uint8_t set_digital_filter_value(const uint8_t filter_value);
uint8_t enable_fifo_for_sensor(enum enable_fifo_sensor_enum sensor);
uint8_t enable_fifo(void);
uint8_t clear_fifo_buffer(void);
uint8_t set_sensor_standby(enum set_sensor_standby_enum sensor);
uint8_t disable_sleep_mode(void);
uint8_t disable_temperature_sensor(void);
uint8_t select_clock_source(enum mpu_clock_source_enum clk_source);
uint16_t get_fifo_counter(void);
uint8_t get_fifo_values(uint8_t* output, const uint16_t len);
uint8_t reset_device(void);
uint8_t reset_signal_paths(void);
uint8_t reset_sensors(void);
uint8_t self_test(uint8_t verbose);
uint8_t set_accel_full_scale_range(enum accel_full_scale_range_enum range);
uint8_t set_gyro_full_scale_range(enum gyro_full_scale_range_enum range);
int16_t read_gyro_sensor(enum GYRO_AXIS axis);
int16_t read_accel_sensor(enum ACCEL_AXIS axis);
#endif