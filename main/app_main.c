#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "sdkconfig.h"

#include "app.h"
#include "mq.h"
#include "digital_io.h"
#include "cli.h"

static const char *TAG = "app_main";

/** @brief App entry point */
void app_main(void) {

    ESP_ERROR_CHECK(app_init());
    ESP_LOGI(TAG, "FreeRTOS resources created");

    ESP_ERROR_CHECK(digital_io_task_init());
    ESP_LOGI(TAG, "Digital IO peripherals initialized");

    ESP_ERROR_CHECK(mq_task_init());
    ESP_LOGI(TAG, "Analog inputs for MQs initialized");

    ESP_ERROR_CHECK(cli_task_init());
    ESP_LOGI(TAG, "CLI task initialized");

    ESP_LOGI(TAG, "Deleting main task");
    vTaskDelete(NULL);
}