#include "app.h"
#include "mq.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "nvs_flash.h"

/** File tag */
#define TAG "MQX"

/** ADC handle */
adc_oneshot_unit_handle_t adc_handle = {0};
/** ADC calibration handles */
adc_cali_handle_t adc_cali_handle[APP_ADC_COUNT] = {0};

// Default NVS values

#define MQ_SAMPLING_TIME_MS_DEFAULT   pdMS_TO_TICKS(20000)
#define MQ_CYCLE_MS_DEFAULT pdMS_TO_TICKS(60000)
#define MQ2_THRESHOLD_MV_DEFAULT  2500
#define MQ3_THRESHOLD_MV_DEFAULT  2500
#define MQ7_THRESHOLD_MV_DEFAULT  2500

/** Time to sample */
uint32_t mq_sampling_time_ms;
/** Time that the task is blocked */
uint32_t mq_cycle_ms;
/** Periodic time sampling */
#define MQ_TASK_SAMPLING_TICK   pdMS_TO_TICKS(100)
/** Number of samples */
#define MQ_SAMPLES              (mq_sampling_time_ms / MQ_TASK_SAMPLING_TICK)

/** Voltage thresholds */
uint32_t mq2_threshold_mv;
uint32_t mq3_threshold_mv;
uint32_t mq7_threshold_mv;

/**
 * @brief Initializes the ADC for the MQ sensors
 * @return ESP_OK if successfull
 */
static esp_err_t mq_adc_init(void);

/**
 * @brief Initializes MQ ADC channel
 * @param adc_channel Number of channel connected to the sensor
 * @return ESP_OK if successfull
 */
static esp_err_t mq_adc_channel_init(adc_channel_t adc_channel);

/**
 * @brief Gets current sensor reading in mV
 * @param adc_channel Number of channel connected to the sensor
 * @param mv[in] Pointer to variable to store the result
 * @return ESP_OK if successfull
 */
static esp_err_t mq_read_mv(adc_channel_t adc_channel, int *mv);

/**
 * @brief Handles the NVS initialization for the MQ variables
 * @return ESP_OK if successfull
 */
static esp_err_t mq_nvs_init(void);

/**
 * @brief Main MQ task
 */
static void mq_task(void *params);

/**
 * @brief MQ task to handle CLI commands
 */
static void mq_cli(void *params);

// Public functions

esp_err_t mq_task_init(void) {
  // ADC initialization
  ESP_LOGI(TAG, "Configuring ADC channels...");
  APP_TRY(mq_adc_init());
  // Configure channels
  APP_TRY(mq_adc_channel_init(adc_map[ADC_MQ2]));
  APP_TRY(mq_adc_channel_init(adc_map[ADC_MQ3]));
  APP_TRY(mq_adc_channel_init(adc_map[ADC_MQ7]));

  // Initialize variables from NVS
  APP_TRY(mq_nvs_init());

  APP_TRY(!xTaskCreate(mq_task, "MQ task", 2048, NULL, 1, NULL));
  APP_TRY(!xTaskCreate(mq_cli, "MQ cli", 2048, NULL, 2, NULL));
  return ESP_OK;
}

// Private functions

static esp_err_t mq_adc_init(void) {
  adc_oneshot_unit_init_cfg_t adc_config = { .unit_id = ADC_UNIT_1 };
  return adc_oneshot_new_unit(&adc_config, &adc_handle);
}

static esp_err_t mq_adc_channel_init(adc_channel_t adc_channel) {
  // Generic ADC configuration
  adc_oneshot_chan_cfg_t adc_channel_config = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
  APP_TRY(adc_oneshot_config_channel(adc_handle, adc_channel, &adc_channel_config));
  // Perform calibration
  adc_cali_curve_fitting_config_t config = { .unit_id = ADC_UNIT_1, .chan = adc_channel, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
  return adc_cali_create_scheme_curve_fitting(&config, &adc_cali_handle[adc_channel - adc_map[ADC_MQ2]]);
}

static esp_err_t mq_read_mv(adc_channel_t adc_channel, int *mv) {
  return adc_oneshot_get_calibrated_result(adc_handle, adc_cali_handle[adc_channel - adc_map[ADC_MQ2]], adc_channel, mv);
}

static esp_err_t mq_nvs_init(void) {
  nvs_variable_init(TAG, "mq2_thres", &mq2_threshold_mv, MQ2_THRESHOLD_MV_DEFAULT);
  nvs_variable_init(TAG, "mq3_thres", &mq3_threshold_mv, MQ3_THRESHOLD_MV_DEFAULT);
  nvs_variable_init(TAG, "mq7_thres", &mq7_threshold_mv, MQ7_THRESHOLD_MV_DEFAULT);
  nvs_variable_init(TAG, "mq_sampling", &mq_sampling_time_ms, MQ_SAMPLING_TIME_MS_DEFAULT);
  nvs_variable_init(TAG, "mq_cycle", &mq_cycle_ms, MQ_CYCLE_MS_DEFAULT);
  return ESP_OK;
}

static void mq_task(void *params) {
  // Tick timestamp
  uint32_t timestamp;

  while(1) {
    // Get current timestamp
    timestamp = xTaskGetTickCount();

    // The whole sampling should take mq_sampling_time_ms
    for(uint32_t j = 0; j < MQ_SAMPLES; j++) {
      int32_t mq2_mv = 0, mq3_mv = 0, mq7_mv = 0;

      for(uint32_t i = 0; i < 10; i++) {
        int dummy_mv;
        mq_read_mv(adc_map[ADC_MQ2], &dummy_mv);
        mq2_mv += dummy_mv;
        mq_read_mv(adc_map[ADC_MQ3], &dummy_mv);
        mq3_mv += dummy_mv;
        mq_read_mv(adc_map[ADC_MQ7], &dummy_mv);
        mq7_mv += dummy_mv;
        vTaskDelay(MQ_TASK_SAMPLING_TICK / 10);
      }
      // Do average
      mq2_mv /= 10; mq3_mv /= 10; mq7_mv /= 10;
      // Set bit when reading is greater than the threshold, clear otherwise

      if(mq2_mv >= mq2_threshold_mv) {
        xEventGroupSetBits(alarm_event, ALARM_THRESHOLD_MQ2_BIT);
      }
      else {
        xEventGroupClearBits(alarm_event, ALARM_THRESHOLD_MQ2_BIT);
      }
      
      if(mq3_mv >= mq3_threshold_mv) {
        xEventGroupSetBits(alarm_event, ALARM_THRESHOLD_MQ3_BIT);
      }
      else {
        xEventGroupClearBits(alarm_event, ALARM_THRESHOLD_MQ3_BIT);
      }
      
      if(mq7_mv >= mq7_threshold_mv) {
        xEventGroupSetBits(alarm_event, ALARM_THRESHOLD_MQ7_BIT);
      }
      else {
        xEventGroupClearBits(alarm_event, ALARM_THRESHOLD_MQ7_BIT);
      }

      if(mq2_mv < mq2_threshold_mv && mq3_mv < mq3_threshold_mv && mq7_mv < mq7_threshold_mv) {
        xEventGroupSetBits(alarm_event, ALARM_OK_TO_TURN_OFF_BIT);
      }
      else {
        xEventGroupClearBits(alarm_event, ALARM_OK_TO_TURN_OFF_BIT);
      }

      ESP_LOGI(TAG, "MQ2: %d mV | MQ3: %d mV | MQ7: %d mV", mq2_mv, mq3_mv, mq7_mv);
    }
    // Sampling should be done every 60s with 40s idle
    vTaskDelayUntil(&timestamp, mq_cycle_ms);
  }
}

static void mq_cli(void *params) {
  // Used for queue data
  int dummy;

  while(1) {
    // Check for any request from CLI
    EventBits_t events = xEventGroupWaitBits(cli_event, MQ_EVENTS_ALL, pdFALSE, pdFALSE, portMAX_DELAY);
    // Check what command was issued
    if(events & GET_MQ2_BIT) {
      mq_read_mv(adc_map[ADC_MQ2], &dummy);
      xQueueSend(cli_data, &dummy, portMAX_DELAY);
      xEventGroupClearBits(cli_event, GET_MQ2_BIT);
    }
    else if(events & GET_MQ3_BIT) {
      mq_read_mv(adc_map[ADC_MQ3], &dummy);
      xQueueSend(cli_data, &dummy, portMAX_DELAY);
      xEventGroupClearBits(cli_event, GET_MQ3_BIT);
    }
    else if(events & GET_MQ7_BIT) {
      mq_read_mv(adc_map[ADC_MQ7], &dummy);
      xQueueSend(cli_data, &dummy, portMAX_DELAY);
      xEventGroupClearBits(cli_event, GET_MQ7_BIT);
    }
    else if(events & GET_MQ2_THRES_BIT) {
      xQueueSend(cli_data, &mq2_threshold_mv, portMAX_DELAY);
      xEventGroupClearBits(cli_event, GET_MQ2_THRES_BIT);
    }
    else if(events & GET_MQ3_THRES_BIT) {
      xQueueSend(cli_data, &mq3_threshold_mv, portMAX_DELAY);
      xEventGroupClearBits(cli_event, GET_MQ3_THRES_BIT);
    }
    else if(events & GET_MQ7_THRES_BIT) {
      xQueueSend(cli_data, &mq7_threshold_mv, portMAX_DELAY);
      xEventGroupClearBits(cli_event, GET_MQ7_THRES_BIT);
    }
    else if(events & SET_MQ2_THRES_BIT) {
      xQueueReceive(cli_data, &mq2_threshold_mv, portMAX_DELAY);
      nvs_set_u32(app_nvs_handle, "mq2_thres", mq2_threshold_mv);
      nvs_commit(app_nvs_handle);
      xEventGroupClearBits(cli_event, SET_MQ2_THRES_BIT);
    }
    else if(events & SET_MQ3_THRES_BIT) {
      xQueueReceive(cli_data, &mq3_threshold_mv, portMAX_DELAY);
      nvs_set_u32(app_nvs_handle, "mq3_thres", mq3_threshold_mv);
      nvs_commit(app_nvs_handle);
      xEventGroupClearBits(cli_event, SET_MQ3_THRES_BIT);
    }
    else if(events & SET_MQ7_THRES_BIT) {
      xQueueReceive(cli_data, &mq7_threshold_mv, portMAX_DELAY);
      nvs_set_u32(app_nvs_handle, "mq7_thres", mq7_threshold_mv);
      nvs_commit(app_nvs_handle);
      xEventGroupClearBits(cli_event, SET_MQ7_THRES_BIT);
    }
    else if(events & GET_MQ_SAMPLING_BIT) {
      uint32_t dummy = mq_sampling_time_ms * portTICK_PERIOD_MS;
      xQueueSend(cli_data, &dummy, portMAX_DELAY);
      xEventGroupClearBits(cli_event, GET_MQ_SAMPLING_BIT);
    }
    else if(events & SET_MQ_SAMPLING_BIT) {
      xQueueReceive(cli_data, &mq_sampling_time_ms, portMAX_DELAY);
      mq_sampling_time_ms = pdMS_TO_TICKS(mq_sampling_time_ms);
      nvs_set_u32(app_nvs_handle, "mq_sampling", mq_sampling_time_ms);
      nvs_commit(app_nvs_handle);
      xEventGroupClearBits(cli_event, SET_MQ_SAMPLING_BIT);
    }
    else if(events & GET_MQ_CYCLE_BIT) {
      uint32_t dummy = mq_cycle_ms * portTICK_PERIOD_MS;
      xQueueSend(cli_data, &dummy, portMAX_DELAY);
      xEventGroupClearBits(cli_event, GET_MQ_CYCLE_BIT);
    }
    else if(events & SET_MQ_CYCLE_BIT) {
      xQueueReceive(cli_data, &mq_cycle_ms, portMAX_DELAY);
      mq_cycle_ms = pdMS_TO_TICKS(mq_cycle_ms);
      nvs_set_u32(app_nvs_handle, "mq_cycle", mq_cycle_ms);
      nvs_commit(app_nvs_handle);
      xEventGroupClearBits(cli_event, SET_MQ_CYCLE_BIT);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}