#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sht21.h"
#include <stdio.h>

void app_main(void)
{
    ESP_ERROR_CHECK(sht21_init(0, GPIO_NUM_21, GPIO_NUM_22, sht21_i2c_speed_standard));

    float temperature;
    ESP_ERROR_CHECK(sht21_get_temperature(&temperature));
    printf("Temperature: %f°C\n", temperature);

    float humidity;
    ESP_ERROR_CHECK(sht21_get_humidity(&humidity));
    printf("Relative humidity: %f%%\n", humidity);

    ESP_ERROR_CHECK(sht21_deinit());
    fflush(stdout);
    vTaskDelete(NULL);
}
