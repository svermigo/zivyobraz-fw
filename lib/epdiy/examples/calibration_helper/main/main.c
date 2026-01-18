/*
 * Simple helper program enabling EPD power to allow for easier VCOM calibration.
 *
 * This is only needed for boards V5 or lower!
 **/

#include <stdio.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "epdiy.h"

// choose the default demo board depending on the architecture
#define BOARD sverio_paperboard_v1

void enable_vcom() {
    epd_init(&BOARD, &ED097TC2, EPD_LUT_64K);
    ESP_LOGI("main", "waiting for one second before poweron...");
    vTaskDelay(1000);
    ESP_LOGI("main", "enabling VCOMM...");
    epd_poweron();
    ESP_LOGI(
        "main",
        "VCOMM enabled. You can now adjust the on-board trimmer to the VCOM value specified on the "
        "display."
    );
    while (1) {
        vTaskDelay(1000);
    };
}

void app_main() {
    enable_vcom();
}
