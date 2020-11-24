# How to build
Dependencies:
- meson
- git
- st-flash (https://github.com/stlink-org/stlink)
- arm compiler/tools (arm-none-eabi- tools)

Build with meson:
~~~
meson --cross-file=cross.build build
meson compile -C build bin
~~~
Flash with st-flash:
~~~
meson compile -C build flash
~~~

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

# Library
Uses libopencm3 (https://github.com/libopencm3)