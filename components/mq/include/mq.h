#ifndef _MQ_H_
#define _MQ_H_

#include "esp_err.h"

/**
 * @brief Initializes the MQ task and peripherals
 */
esp_err_t mq_task_init(void);

#endif