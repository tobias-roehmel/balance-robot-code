# Balance Robot Code

Uses libopencm3 (https://github.com/libopencm3)

# Pin connections
- UART Tx A9

- MPU6050
    - I2C SDA B9
    - I2C SCK B8
    - Data ready interrupt A1

- Motor Driver
    - PWM (TIM3_CH1) A6
    - Enable B2
    - Direction A7
    - Halft step activate B1