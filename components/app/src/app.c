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
  [ADC_MQ2] = ADC_CHANNEL_2,  /**< GPIO2 */
  [ADC_MQ3] = ADC_CHANNEL_3,  /**< GPIO3 */
  [ADC_MQ7] = ADC_CHANNEL_4,  /**< GPIO4 */
};

EventGroupHandle_t alarm_event = NULL;

esp_err_t app_init(void) {
  alarm_event = xEventGroupCreate();
  if(alarm_event == NULL) {
    // No memory available to create event
    return ESP_ERR_NO_MEM;
  }
  return ESP_OK;
}