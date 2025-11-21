#include "app.h"
#include "digital_io.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "esp_log.h"

/** File tag */
#define TAG "digital_io"
/** Time to keep the outputs on during the initial test */
#define TICKS_INITIAL_OUTPUT_TEST   pdMS_TO_TICKS(3000)
/** Debouncing time */
#define DEBOUNCING_TIME_MS  pdMS_TO_TICKS(20)
/** Input blocking time */
#define RST_READING_TIME_MS pdMS_TO_TICKS(50)
/** LED blocking time */
#define LEDS_TIME_MS  pdMS_TO_TICKS(50)
/** Extractor blocking time */
#define EXTRACTOR_TIME_MS pdMS_TO_TICKS(30 * 60 * 1000)

// Private function prototypes

/**
 * @brief Routine to drive output by required value
 * @param gpio Number of GPIO
 * @param value 1 for HIGH 0 for LOW
 * @return ESP_OK if everything went well
 */
static esp_err_t digital_io_drive_output(gpio_num_t gpio, uint32_t value);

/**
 * @brief Gets the GPIO level with software 
 * @param gpio GPIO number
 * @param expected_value Value to test against depending on whether the GPIO has pull-up/pull-down
 * @return GPIO value
 */
static uint32_t digital_io_get_level(gpio_num_t gpio, uint32_t expected_value);

/** @brief Handles MQ LEDs */
static void digital_io_mq_leds_task(void *params);

/** @brief Handles buzzer */
static void digital_io_buzzer_task(void *params);

/** @brief Handles alarm LED */
static void digital_io_led_task(void *params);

/** @brief Handles extractor */
static void digital_io_extractor_task(void *params);

/** @brief Handles reset alarm button */
static void digital_io_rst_button(void *params);

// Public functions

esp_err_t digital_io_task_init(void) {

  ESP_LOGI(TAG, "Configuring outputs...");
  for(uint8_t app_gpio_num = 0; app_gpio_num < APP_GPIO_COUNT; app_gpio_num++) {
      gpio_num_t gpio = gpio_map[app_gpio_num];
      // Clear GPIO configuration
      APP_TRY(gpio_reset_pin(gpio));
      // Configuration
      if(app_gpio_num < GPIO_RST_BTN) {
          // Configure output
          APP_TRY(gpio_set_direction(gpio, GPIO_MODE_INPUT_OUTPUT));
      }
      else if(app_gpio_num == GPIO_RST_BTN) {
          // Configure input
          APP_TRY(gpio_set_direction(gpio, GPIO_MODE_INPUT));
      }
  }

  // Initial test routine for TICKS_INITIAL_OUTPUT_TEST seconds
  ESP_LOGI(TAG, "Testing outputs...");
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_BUZZER], 1));
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_LED_ALARM], 1));
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_EXTRACTOR], 1));
  vTaskDelay(TICKS_INITIAL_OUTPUT_TEST);
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_BUZZER], 0));
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_LED_ALARM], 0));
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_EXTRACTOR], 0));

  ESP_LOGI(TAG, "Testing LEDs...");
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_LED_G1], 0));
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_LED_G2], 0));
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_LED_G3], 0));
  vTaskDelay(TICKS_INITIAL_OUTPUT_TEST);
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_LED_G1], 1));
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_LED_G2], 1));
  APP_TRY(digital_io_drive_output(gpio_map[GPIO_LED_G3], 1));

  // Create tasks
  APP_TRY(!xTaskCreate(digital_io_buzzer_task, "Buzzer task", configMINIMAL_STACK_SIZE, NULL, 2, NULL));
  APP_TRY(!xTaskCreate(digital_io_led_task, "Alarm LED task", configMINIMAL_STACK_SIZE, NULL, 2, NULL));
  APP_TRY(!xTaskCreate(digital_io_extractor_task, "Extractor task", configMINIMAL_STACK_SIZE, NULL, 2, NULL));
  APP_TRY(!xTaskCreate(digital_io_mq_leds_task, "MQ LEDs Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL));
  APP_TRY(!xTaskCreate(digital_io_rst_button, "Reset task", configMINIMAL_STACK_SIZE, NULL, 3, NULL));

  return ESP_OK;
}

// Private functions

static esp_err_t digital_io_drive_output(gpio_num_t gpio, uint32_t value) {
  // Drive output to high
  APP_TRY(gpio_set_level(gpio, value));
  // Test output value
  bool tested_value = gpio_get_level(gpio);
  if(tested_value != value) { return ESP_ERR_INVALID_RESPONSE; }
  return ESP_OK;
}

static uint32_t digital_io_get_level(gpio_num_t gpio, uint32_t expected_value) {
  // Check flank
  if(gpio_get_level(gpio) == expected_value) {
    // Wait to make sure it actually was pressed
    vTaskDelay(DEBOUNCING_TIME_MS);
    if(gpio_get_level(gpio) == expected_value) {
      return expected_value;
    }
  }
  return !expected_value;
}

static void digital_io_mq_leds_task(void *params) {
  // LED GPIOs
  gpio_num_t led_mq2 = gpio_map[GPIO_LED_G1];
  gpio_num_t led_mq3 = gpio_map[GPIO_LED_G2];
  gpio_num_t led_mq7 = gpio_map[GPIO_LED_G3];

  while(1) {
    // Wait for any bit to be set
    EventBits_t events = xEventGroupWaitBits(alarm_event, ALARM_THRESHOLD_ALL | ALARM_RST, pdFALSE, pdFALSE, 0);
    // Handle the LEDs
    if(events & ALARM_RST) {
      // Reset all LEDs
      ESP_LOGI(TAG, "Turning off MQ LEDs");
      digital_io_drive_output(led_mq2, 1);
      digital_io_drive_output(led_mq3, 1);
      digital_io_drive_output(led_mq7, 1);
    }
    if(events & ALARM_THRESHOLD_MQ2_BIT) {
      ESP_LOGI(TAG, "Turning on MQ2 LED");
      digital_io_drive_output(led_mq2, 0);
    }
    if(events & ALARM_THRESHOLD_MQ3_BIT) {
      ESP_LOGI(TAG, "Turning on MQ3 LED");
      digital_io_drive_output(led_mq3, 0);
    }
    if(events & ALARM_THRESHOLD_MQ7_BIT) {
      ESP_LOGI(TAG, "Turning on MQ7 LED");
      digital_io_drive_output(led_mq7, 0);
    }
    vTaskDelay(LEDS_TIME_MS);
  }
}

static void digital_io_buzzer_task(void *params) {
  // GPIO number
  gpio_num_t gpio = gpio_map[GPIO_BUZZER];

  while(1) {
    // Wait for any event
    xEventGroupWaitBits(alarm_event, ALARM_THRESHOLD_ALL, pdFALSE, pdFALSE, portMAX_DELAY);
    // Turn on LED and wait for reset
    ESP_LOGI(TAG, "Turning on buzzer");
    digital_io_drive_output(gpio, 1);
    xEventGroupWaitBits(alarm_event, ALARM_RST_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    // Once reset has been reached, turn off LED
      ESP_LOGI(TAG, "Turning off buzzer");
    digital_io_drive_output(gpio, 0);
  }
}

static void digital_io_led_task(void *params) {
  // GPIO number
  gpio_num_t gpio = gpio_map[GPIO_LED_ALARM];

  while(1) {
    // Wait for any event
    xEventGroupWaitBits(alarm_event, ALARM_THRESHOLD_ALL, pdFALSE, pdFALSE, portMAX_DELAY);
    // Turn on LED and wait for reset
    ESP_LOGI(TAG, "Turning on alarm LED");
    digital_io_drive_output(gpio, 1);
    xEventGroupWaitBits(alarm_event, ALARM_RST_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    // Once reset has been reached, turn off LED
    ESP_LOGI(TAG, "Turning off alarm LED");
    digital_io_drive_output(gpio, 0);
  }
}

static void digital_io_extractor_task(void *params) {
  // GPIO number
  gpio_num_t gpio = gpio_map[GPIO_EXTRACTOR];

  while(1) {
    // Wait for any event
    EventBits_t events = xEventGroupWaitBits(alarm_event, ALARM_THRESHOLD_ALL, pdFALSE, pdFALSE, 0);
    // Turn on extractor for any of the MQ sensors but only turn off if after required time there are no more gas concentration
    if(events & (ALARM_THRESHOLD_ALL)) {
      ESP_LOGI(TAG, "Turning on extractor");
      digital_io_drive_output(gpio, 1);
    }
    else {
      ESP_LOGI(TAG, "Turning off extractor");
      digital_io_drive_output(gpio, 0);
    }
    vTaskDelay(EXTRACTOR_TIME_MS);
  }
}

static void digital_io_rst_button(void *params) {
  // GPIO number
  gpio_num_t gpio = gpio_map[GPIO_RST_BTN];

  while(1) {
    // Debouncing
    if(digital_io_get_level(gpio, 1)) {
      // Reset events to let other tasks know
      ESP_LOGI(TAG, "Reset button pressed");
      xEventGroupClearBits(alarm_event, ALARM_THRESHOLD_ALL);
      xEventGroupSetBits(alarm_event, ALARM_RST_BIT);
    }
    vTaskDelay(RST_READING_TIME_MS);
  }
}