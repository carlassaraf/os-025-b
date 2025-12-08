#include "app.h"
#include "esp_log.h"

const gpio_num_t gpio_map[APP_GPIO_COUNT] = {
  [GPIO_MQ2]       = GPIO_NUM_2,
  [GPIO_MQ3]       = GPIO_NUM_3,
  [GPIO_MQ7]       = GPIO_NUM_4,
  [GPIO_BUZZER]    = GPIO_NUM_5,
  [GPIO_LED_ALARM] = GPIO_NUM_6,
  [GPIO_EXTRACTOR] = GPIO_NUM_7,
  [GPIO_LED_G3]    = GPIO_NUM_8,
  [GPIO_LED_G2]    = GPIO_NUM_9,
  [GPIO_LED_G1]    = GPIO_NUM_10,
  [GPIO_RST_BTN]   = GPIO_NUM_11,
};

const adc_channel_t adc_map[APP_ADC_COUNT] = {
  [ADC_MQ2] = ADC_CHANNEL_1,  /**< GPIO2 */
  [ADC_MQ3] = ADC_CHANNEL_2,  /**< GPIO3 */
  [ADC_MQ7] = ADC_CHANNEL_3,  /**< GPIO4 */
};

EventGroupHandle_t alarm_event = NULL;
EventGroupHandle_t cli_event = NULL;
QueueHandle_t cli_data = NULL;

nvs_handle_t app_nvs_handle;

esp_err_t app_init(void) {
  alarm_event = xEventGroupCreate();
  if(alarm_event == NULL) {
    // No memory available to create event
    return ESP_ERR_NO_MEM;
  }

  cli_event = xEventGroupCreate();
  if(cli_event == NULL) {
    // No memory available to create event
    return ESP_ERR_NO_MEM;
  }

  cli_data = xQueueCreate(1, sizeof(uint32_t));
  if(cli_data == NULL) {
    // No memory available to create event
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}

esp_err_t app_nvs_init(void) {
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      APP_TRY(nvs_flash_erase());
      APP_TRY(nvs_flash_init());
  }
  APP_TRY(nvs_open("storage", NVS_READWRITE, &app_nvs_handle));
  return ESP_OK;
}

esp_err_t nvs_variable_init(char *tag, char *key, uint32_t *dst, uint32_t def) {

  if(nvs_get_u32(app_nvs_handle, key, dst) == ESP_ERR_NVS_NOT_FOUND) {
    // Key does not exist, we have to set default value in NVS
    nvs_set_u32(app_nvs_handle, key, def);
    nvs_commit(app_nvs_handle);
    *dst = def;
  }
  else {
    ESP_LOGI(tag, "%s = %d from NVS", key, *dst);
  }
  return ESP_OK;
}