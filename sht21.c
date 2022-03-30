#include "sht21.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TX_BUF_DISABLE 0

#define ERR_RET(x)                                                             \
    do                                                                         \
    {                                                                          \
        esp_err_t esp_err_ = (x);                                              \
        if (esp_err_ != ESP_OK)                                                \
            return esp_err_;                                                   \
    } while (0)

esp_err_t sht21_init(i2c_port_t i2c_port, gpio_num_t sda_pin,
                     gpio_num_t scl_pin, sht21_i2c_speed_t i2c_speed)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_speed,
    };

    ERR_RET(i2c_param_config(i2c_port, &conf));
    return i2c_driver_install(i2c_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}
