#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"

typedef enum
{
    sht21_i2c_speed_standard = 100000,
    sht21_i2c_speed_fast = 400000,
} sht21_i2c_speed_t;

esp_err_t sht21_init(i2c_port_t i2c_port, gpio_num_t sda_pin,
                     gpio_num_t scl_pin, sht21_i2c_speed_t i2c_speed);
