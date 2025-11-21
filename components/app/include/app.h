#ifndef _APP_H_
#define _APP_H_

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/** Macro to test functions and return error */
#define APP_TRY(x)  \
do {                  \
  esp_err_t res = x;  \
  if(res != ESP_OK) { \
    return res;       \
  }                   \
}                     \
while(0)

typedef enum app_gpio {
  GPIO_MQ2,         /**< MQ2 sensor output */
  GPIO_MQ3,         /**< MQ3 sensor output */
  GPIO_MQ7,         /**< MQ7 sensor output */
  GPIO_BUZZER,      /**< Buzzer output */
  GPIO_LED_ALARM,   /**< LED alarm indicator */
  GPIO_EXTRACTOR,   /**< Extractor output */
  GPIO_LED_G3,      /**< LED for gas N3 detection */
  GPIO_LED_G2,      /**< LED for gas N2 detection */
  GPIO_LED_G1,      /**< LED for gas N1 detection */
  GPIO_RST_BTN,     /**< Alarm reset input switch */
  APP_GPIO_COUNT    /**< Number of GPIO used */
} app_gpio_t;

typedef enum app_adc {
  ADC_MQ2,        /**< GPL sensor */
  ADC_MQ3,        /**< Alcohol sensor */
  ADC_MQ7,        /**< CO sensor */
  APP_ADC_COUNT
} app_adc_t;

typedef enum app_alarm_src {
  ALARM_RST,            /**< Reset button has been pressed */
  ALARM_THRESHOLD_MQ2,  /**< MQ2 voltage over threshold */
  ALARM_THRESHOLD_MQ3,  /**< MQ3 voltage over threshold */
  ALARM_THRESHOLD_MQ7   /**< MQ7 voltage over threshold */
} app_alarm_src_t;

typedef enum app_event_bits {
  ALARM_RST_BIT = 1 << ALARM_RST,
  ALARM_THRESHOLD_MQ2_BIT = 1 << ALARM_THRESHOLD_MQ2,
  ALARM_THRESHOLD_MQ3_BIT = 1 << ALARM_THRESHOLD_MQ3,
  ALARM_THRESHOLD_MQ7_BIT = 1 << ALARM_THRESHOLD_MQ7,
  ALARM_THRESHOLD_ALL = 1 << ALARM_THRESHOLD_MQ2 | 1 << ALARM_THRESHOLD_MQ3 | 1 << ALARM_THRESHOLD_MQ7
} app_event_bits_t;

extern const gpio_num_t gpio_map[APP_GPIO_COUNT];
extern const adc_channel_t adc_map[APP_ADC_COUNT];

extern EventGroupHandle_t alarm_event;

esp_err_t app_init(void);

#endif