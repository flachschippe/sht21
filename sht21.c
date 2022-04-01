#include "sht21.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_check.h"

#define ER(x) ESP_RETURN_ON_ERROR(x, __FILE__, "")

#define I2C_ADDRESS 0x40
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_TIMEOUT_MS 200

/* See Datasheet Page 8 Table 6 */
// clang-format off
#define SHT21_CMD_TRIG_T_MEASUREMENT_HM   0xE3 // command trig. temp meas. hold master
#define SHT21_CMD_TRIG_RH_MEASUREMENT_HM  0xE5 // command trig. humidity meas. hold master
#define SHT21_CMD_TRIG_T_MEASUREMENT_NHM  0xF3 // command trig. temp meas. no hold master
#define SHT21_CMD_TRIG_RH_MEASUREMENT_NHM 0xF5 // command trig. humid. meas. no hold master
// clang-format on

// TODO LORIS: rename struct? or just get rid of it
typedef struct
{
    uint16_t data;
    // TODO LORIS: should just return a bool for checksum passed?
    uint8_t checksum;
} sensor_raw_value_t;

static i2c_port_t i2c_port;
static esp_err_t read_sensor(uint8_t command,
                             sensor_raw_value_t *sensor_raw_value);

esp_err_t sht21_init(i2c_port_t i2c_num, gpio_num_t sda_pin, gpio_num_t scl_pin,
                     sht21_i2c_speed_t i2c_speed)
{
    i2c_port = i2c_num;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_speed,
    };

    ER(i2c_param_config(i2c_port, &conf));
    ER(i2c_driver_install(i2c_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
                          I2C_MASTER_TX_BUF_DISABLE, 0));
    return ESP_OK;
}

esp_err_t sht21_get_temperature(float *ans)
{
    sensor_raw_value_t value;
    ER(read_sensor(SHT21_CMD_TRIG_T_MEASUREMENT_NHM, &value));
    *ans = (float)value.data;
    return ESP_OK;
}

esp_err_t sht21_get_humidity(float *ans)
{
    sensor_raw_value_t value;
    ER(read_sensor(SHT21_CMD_TRIG_RH_MEASUREMENT_NHM, &value));
    *ans = (float)value.data;
    return ESP_OK;
}

static esp_err_t read_sensor(uint8_t command,
                             sensor_raw_value_t *sensor_raw_value)
{
    uint8_t data_msb;
    uint8_t data_lsb;
    uint8_t checksum;
    esp_err_t err = ESP_OK;

    i2c_cmd_handle_t write_cmd = i2c_cmd_link_create();
    ER(i2c_master_start(write_cmd));
    ER(i2c_master_write_byte(write_cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE,
                             I2C_MASTER_ACK));
    ER(i2c_master_write_byte(write_cmd, command, I2C_MASTER_ACK));
    ER(i2c_master_stop(write_cmd));

    err = i2c_master_cmd_begin(i2c_port, write_cmd,
                               I2C_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(write_cmd);
    ER(err);

    // TODO LORIS: change delay based on temperature or humidity
    //   datasheet page 9
    vTaskDelay(85 / portTICK_PERIOD_MS);

    i2c_cmd_handle_t read_cmd = i2c_cmd_link_create();
    ER(i2c_master_start(read_cmd));
    ER(i2c_master_write_byte(read_cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ,
                             I2C_MASTER_ACK));
    ER(i2c_master_read_byte(read_cmd, &data_msb, I2C_MASTER_ACK));
    ER(i2c_master_read_byte(read_cmd, &data_lsb, I2C_MASTER_ACK));
    ER(i2c_master_read_byte(read_cmd, &checksum, I2C_MASTER_NACK));
    ER(i2c_master_stop(read_cmd));

    err = i2c_master_cmd_begin(i2c_port, read_cmd,
                               I2C_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(read_cmd);
    ER(err);

    uint16_t data = data_msb << 8;
    data |= data_lsb;
    sensor_raw_value->data = data;
    sensor_raw_value->checksum = checksum;
    return ESP_OK;
}
