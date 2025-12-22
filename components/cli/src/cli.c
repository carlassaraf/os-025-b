#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "cli.h"
#include "app.h"
#include "digital_io.h"
#include "mq.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h> 

#include "esp_log.h"

/** File tag */
#define TAG "cli"

/** Getter function pointer */
typedef uint32_t (*cli_getter)(void);
/** Setter function pointer */
typedef void (*cli_setter)(uint32_t);
/** Exec function pointer */
typedef void (*cli_exec)(void);

/**
 * @struct cli_param
 * @brief Struct to define possible commands
 */
typedef struct cli_param {
  const char *name;   /**< Command name */
  cli_getter getter;  /**< Pointer to getter function */
  cli_setter setter;  /**< Pointer to setter function */
  cli_exec exec;      /**< Pointer to exec function */
} cli_param_t;

// Wrappers

static uint32_t get_wrapper(cli_cmd_bits_t cli_event_bit) {
  int dummy;
  xEventGroupSetBits(cli_event, cli_event_bit);
  xQueueReceive(cli_data, &dummy, portMAX_DELAY);
  return (uint32_t) dummy;
}

static void set_wrapper(cli_cmd_bits_t cli_event_bit, uint32_t *value) {
  xEventGroupSetBits(cli_event, cli_event_bit);
  xQueueSend(cli_data, value, portMAX_DELAY);
}

static void exec_wrapper(cli_cmd_bits_t cli_event_bit) {
  xEventGroupSetBits(cli_event, cli_event_bit);
}

// Getter functions
static uint32_t get_mq2(void) { return get_wrapper(GET_MQ2_BIT); }
static uint32_t get_mq3(void) { return get_wrapper(GET_MQ3_BIT); }
static uint32_t get_mq7(void) { return get_wrapper(GET_MQ7_BIT); }
static uint32_t get_mq2_thres(void) { return get_wrapper(GET_MQ2_THRES_BIT); }
static uint32_t get_mq3_thres(void) { return get_wrapper(GET_MQ3_THRES_BIT); }
static uint32_t get_mq7_thres(void) { return get_wrapper(GET_MQ7_THRES_BIT); }
static uint32_t get_mq_sampling(void) { return get_wrapper(GET_MQ_SAMPLING_BIT); }
static uint32_t get_mq_cycle(void) { return get_wrapper(GET_MQ_CYCLE_BIT); }
static uint32_t get_extractor_ms(void) { return get_wrapper(GET_EXTRACTOR_MS_BIT); }

// Setter functions
static void set_mq2_thres(uint32_t thres) { set_wrapper(SET_MQ2_THRES_BIT, &thres); }
static void set_mq3_thres(uint32_t thres) { set_wrapper(SET_MQ3_THRES_BIT, &thres); }
static void set_mq7_thres(uint32_t thres) { set_wrapper(SET_MQ7_THRES_BIT, &thres); }
static void set_mq_sampling(uint32_t ms) { set_wrapper(SET_MQ_SAMPLING_BIT, &ms); }
static void set_mq_cycle(uint32_t ms) { set_wrapper(SET_MQ_CYCLE_BIT, &ms); }
static void set_extractor_ms(uint32_t ms) { set_wrapper(SET_EXTRACTOR_MS_BIT, &ms); }

// Exec functions
static void exec_rst(void) { exec_wrapper(EXEC_RST_BIT); }

// Available commands
cli_param_t commands[] = {
  { "mq2", get_mq2, NULL, NULL },
  { "mq3", get_mq3, NULL, NULL },
  { "mq7", get_mq7, NULL, NULL },
  { "mq2_thres", get_mq2_thres, set_mq2_thres, NULL },
  { "mq3_thres", get_mq3_thres, set_mq3_thres, NULL },
  { "mq7_thres", get_mq7_thres, set_mq7_thres, NULL },
  { "mq_sampling", get_mq_sampling, set_mq_sampling, NULL },
  { "mq_cycle", get_mq_cycle, set_mq_cycle, NULL },
  { "extractor_ms", get_extractor_ms, set_extractor_ms, NULL },
  { "rst", NULL, NULL, exec_rst },
  { NULL, NULL, NULL, NULL }
};

// Private prototypes

/** 
 * @brief Main CLI task
 */
static void cli_task(void *params);

// Public functions

esp_err_t cli_task_init(void) {

  APP_TRY(!xTaskCreate(cli_task, "CLI task", 4096, NULL, 4, NULL));
  return ESP_OK;
}

static void cli_process_command(char *input) {

  char *save_ptr;
  char *command = strtok_r(input, " ", &save_ptr);

  if(strcmp(command, "get") == 0) {
    char *variable = strtok_r(NULL, " ", &save_ptr);
    if(variable == NULL) { 
      ESP_LOGE(TAG, "Command 'get' requires a variable to read");
      return;
    }
    // Find the command to run
    for(uint32_t i = 0; commands[i].name != NULL; i++) {
      if(strcmp(commands[i].name, variable) == 0) {
        uint32_t ret = commands[i].getter();
        ESP_LOGI(TAG, "%s %s = %d", command, variable, ret);
        return;
      }
    }
  }

  else if(strcmp(command, "set") == 0) {
    char *variable = strtok_r(NULL, " ", &save_ptr);
    char *value = strtok_r(NULL, " ", &save_ptr);
    if(variable == NULL || value == NULL) {
      ESP_LOGE(TAG, "Command 'set' requires a variable and a value to write");
      return;
    }
    // Find the command to run
    for(uint32_t i = 0; commands[i].name != NULL; i++) {
      if(strcmp(commands[i].name, variable) == 0) {
        commands[i].setter(atoi(value));
        ESP_LOGI(TAG, "%s %s", command, variable);
        return;
      }
    }
  }

  else if(strcmp(command, "exec") == 0) {
    char *variable = strtok_r(NULL, " ", &save_ptr);
    if(variable == NULL) { 
      ESP_LOGE(TAG, "Command 'exec' requires a name to execute");
      return;
    }
    // Find the command to run
    for(uint32_t i = 0; commands[i].name != NULL; i++) {
      if(strcmp(commands[i].name, variable) == 0) {
        commands[i].exec();
        ESP_LOGI(TAG, "%s %s", command, variable);
        return;
      }
    }
  }

  else {
    ESP_LOGE(TAG, "Invalid command");
    return;
  }
}

static void cli_task(void *params) {

  int c;
  char input[32];

  while(1) {

    int i = 0;
    memset(input, 0, 32);
    do {
      c = getchar();
      if(c != EOF && c != '\n') { input[i++] = c; }
      vTaskDelay(pdMS_TO_TICKS(2));
    } while(c != EOF && c != '\n');

    // Discard if string is not even command length
    if(strlen(input) < 3) { continue; }
    cli_process_command(input);
  }
}