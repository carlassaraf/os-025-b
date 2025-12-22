#ifndef _APP_H_
#define _APP_H_

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#include "nvs_flash.h"
#include "nvs.h"

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
  ALARM_CLI_RST,        /**< Reset command has been issued */
  ALARM_THRESHOLD_MQ2,  /**< MQ2 voltage over threshold */
  ALARM_THRESHOLD_MQ3,  /**< MQ3 voltage over threshold */
  ALARM_THRESHOLD_MQ7,  /**< MQ7 voltage over threshold */
  ALARM_OK_TO_TURN_OFF  /**< Its OK to turn off extractor */
} app_alarm_src_t;

typedef enum app_event_bits {
  ALARM_RST_BIT = 1 << ALARM_RST,
  ALARM_CLI_RST_BIT = 1 << ALARM_CLI_RST,
  ALARM_THRESHOLD_MQ2_BIT = 1 << ALARM_THRESHOLD_MQ2,
  ALARM_THRESHOLD_MQ3_BIT = 1 << ALARM_THRESHOLD_MQ3,
  ALARM_THRESHOLD_MQ7_BIT = 1 << ALARM_THRESHOLD_MQ7,
  ALARM_OK_TO_TURN_OFF_BIT = 1 << ALARM_OK_TO_TURN_OFF,
  ALARM_THRESHOLD_ALL = 1 << ALARM_THRESHOLD_MQ2 | 1 << ALARM_THRESHOLD_MQ3 | 1 << ALARM_THRESHOLD_MQ7
} app_event_bits_t;

typedef enum cli_cmd {
  GET_MQ2,
  GET_MQ3,
  GET_MQ7,
  GET_MQ2_THRES,
  GET_MQ3_THRES,
  GET_MQ7_THRES,
  SET_MQ2_THRES,
  SET_MQ3_THRES,
  SET_MQ7_THRES,
  GET_MQ_SAMPLING,
  SET_MQ_SAMPLING,
  GET_MQ_CYCLE,
  SET_MQ_CYCLE,
  EXEC_RST,
  GET_EXTRACTOR_MS,
  SET_EXTRACTOR_MS
} cli_cmd_t;

typedef enum cli_cmd_bits {
  GET_MQ2_BIT = 1 << GET_MQ2,
  GET_MQ3_BIT = 1 << GET_MQ3,
  GET_MQ7_BIT = 1 << GET_MQ7,
  GET_MQ2_THRES_BIT = 1 << GET_MQ2_THRES,
  GET_MQ3_THRES_BIT = 1 << GET_MQ3_THRES,
  GET_MQ7_THRES_BIT = 1 << GET_MQ7_THRES,
  SET_MQ2_THRES_BIT = 1 << SET_MQ2_THRES,
  SET_MQ3_THRES_BIT = 1 << SET_MQ3_THRES,
  SET_MQ7_THRES_BIT = 1 << SET_MQ7_THRES,
  GET_MQ_SAMPLING_BIT = 1 << GET_MQ_SAMPLING,
  SET_MQ_SAMPLING_BIT = 1 << SET_MQ_SAMPLING,
  GET_MQ_CYCLE_BIT = 1 << GET_MQ_CYCLE,
  SET_MQ_CYCLE_BIT = 1 << SET_MQ_CYCLE,
  MQ_EVENTS_ALL = GET_MQ2_BIT | GET_MQ3_BIT | GET_MQ7_BIT | GET_MQ2_THRES_BIT | GET_MQ3_THRES_BIT | GET_MQ7_THRES_BIT | SET_MQ2_THRES_BIT | SET_MQ3_THRES_BIT | SET_MQ7_THRES_BIT | GET_MQ_SAMPLING_BIT | SET_MQ_SAMPLING_BIT | GET_MQ_CYCLE_BIT | SET_MQ_CYCLE_BIT,
  EXEC_RST_BIT = 1 << EXEC_RST,
  GET_EXTRACTOR_MS_BIT = 1 << GET_EXTRACTOR_MS,
  SET_EXTRACTOR_MS_BIT = 1 << SET_EXTRACTOR_MS,
  DIGITAL_IO_EVENTS_ALL = EXEC_RST_BIT | GET_EXTRACTOR_MS_BIT | SET_EXTRACTOR_MS_BIT
} cli_cmd_bits_t;

extern const gpio_num_t gpio_map[APP_GPIO_COUNT];
extern const adc_channel_t adc_map[APP_ADC_COUNT];

extern EventGroupHandle_t alarm_event;
extern EventGroupHandle_t cli_event;
extern QueueHandle_t cli_data;

extern nvs_handle_t app_nvs_handle;

esp_err_t app_init(void);
esp_err_t app_nvs_init(void);

/**
 * @brief Initializes an MQ variable from NVS
 * @param key Name of the variable in NVS
 * @param dst Pointer to variable that host NVS value
 * @param def Variable default value if NVS was not initialized
 * @return ESP_OK if successfull
 */
esp_err_t nvs_variable_init(char *tag, char *key, uint32_t *dst, uint32_t def);

#endif