#include "app.h"
#include "mq.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

/** File tag */
#define TAG "MQX"

/** ADC handle */
adc_oneshot_unit_handle_t adc_handle = {0};
/** ADC calibration handles */
adc_cali_handle_t adc_cali_handle[APP_ADC_COUNT] = {0};

/** Periodic time sampling */
#define MQ_TASK_SAMPLING_TICK   pdMS_TO_TICKS(100)
/** Time to sample */
#define MQ_SAMPLING_TIME        pdMS_TO_TICKS(20000)
/** Number of samples */
#define MQ_SAMPLES              (MQ_SAMPLING_TIME / MQ_TASK_SAMPLING_TICK)
/** Time that the task is blocked */
#define TASK_BLOKING_TIME       pdMS_TO_TICKS(60000)

/** Voltage thresholds */
#define MQ2_THRESHOLD_MV  2500
#define MQ3_THRESHOLD_MV  2500
#define MQ7_THRESHOLD_MV  2500

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
 * @brief Main MQ task
 */
static void mq_task(void *params);

// Public functions

esp_err_t mq_task_init(void) {
  // ADC initialization
  ESP_LOGI(TAG, "Configuring ADC channels...");
  APP_TRY(mq_adc_init());
  // Configure channels
  APP_TRY(mq_adc_channel_init(adc_map[ADC_MQ2]));
  APP_TRY(mq_adc_channel_init(adc_map[ADC_MQ3]));
  APP_TRY(mq_adc_channel_init(adc_map[ADC_MQ7]));

  APP_TRY(!xTaskCreate(mq_task, "MQ task", 2048, NULL, 1, NULL));
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

static void mq_task(void *params) {
  // Tick timestamp
  uint32_t timestamp;

  while(1) {
    // Get current timestamp
    timestamp = xTaskGetTickCount();

    // The whole sampling should take MQ_SAMPLING_TIME
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

      if(mq2_mv >= MQ2_THRESHOLD_MV) {
        xEventGroupSetBits(alarm_event, ALARM_THRESHOLD_MQ2_BIT);
      }
      else {
        xEventGroupClearBits(alarm_event, ALARM_THRESHOLD_MQ2_BIT);
      }
      
      if(mq3_mv >= MQ3_THRESHOLD_MV) {
        xEventGroupSetBits(alarm_event, ALARM_THRESHOLD_MQ3_BIT);
      }
      else {
        xEventGroupClearBits(alarm_event, ALARM_THRESHOLD_MQ3_BIT);
      }
      
      if(mq7_mv >= MQ7_THRESHOLD_MV) {
        xEventGroupSetBits(alarm_event, ALARM_THRESHOLD_MQ7_BIT);
      }
      else {
        xEventGroupClearBits(alarm_event, ALARM_THRESHOLD_MQ7_BIT);
      }

      ESP_LOGI(TAG, "MQ2: %d mV | MQ3: %d mV | MQ7: %d mV", mq2_mv, mq3_mv, mq7_mv);
    }
    // Sampling should be done every 60s with 40s idle
    vTaskDelayUntil(&timestamp, TASK_BLOKING_TIME);
  }
}