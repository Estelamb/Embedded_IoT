# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(brightness_control)

target_sources(app PRIVATE 
    src/main.c
    src/sensors/rgb_led/rgb_led.c
    src/sensors/adc/adc.c
    src/sensors/user_button/user_button.c
)

target_include_directories(app PRIVATE
    src/sensors/rgb_led
    src/sensors/adc
    src/sensors/user_button
)