#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"

typedef enum
{
    sht21_i2c_speed_standard = 100000,
    sht21_i2c_speed_fast = 400000,
} sht21_i2c_speed_t;

// TODO LORIS: keep it private
// TODO LORIS: make it #defines, as it's hardcoded
typedef enum
{
    TRIG_T_MEASUREMENT_HM = 0xE3,   // command trig. temp meas. hold master
    TRIG_RH_MEASUREMENT_HM = 0xE5,  // command trig. humidity meas. hold master
    TRIG_T_MEASUREMENT_NHM = 0xF3,  // command trig. temp meas. no hold master
    TRIG_RH_MEASUREMENT_NHM = 0xF5, // command trig. humid. meas. no hold master
} sht21_command_t;

// TODO LORIS: rename struct?
// TODO LORIS: keep it private
typedef struct
{
    uint16_t data;
    // TODO LORIS: should just return a bool for checksum passed?
    uint8_t checksum;
} sensor_raw_value_t;

esp_err_t sht21_init(i2c_port_t i2c_port, gpio_num_t sda_pin,
                     gpio_num_t scl_pin, sht21_i2c_speed_t i2c_speed);

esp_err_t sht21_read_sensor(sht21_command_t command,
                            sensor_raw_value_t *sensor_raw_value);

esp_err_t sht21_get_temp(float *ans);

esp_err_t sht21_get_rh(float *ans);
