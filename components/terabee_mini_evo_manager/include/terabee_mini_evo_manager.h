/*******************************************************************************
 * Copyright (c) 2025, Adam G. Sweeney <agsweeney@gmail.com>
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ******************************************************************************/

#ifndef TERABEE_MINI_EVO_MANAGER_H
#define TERABEE_MINI_EVO_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"
#include "system_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Terabee Evo Mini manager
 * 
 * This function initializes the sensor, detects it on I2C buses,
 * and starts the update task.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t terabee_mini_evo_manager_init(void);

/**
 * @brief Check if Terabee Evo Mini manager is initialized
 * 
 * @return true if initialized, false otherwise
 */
bool terabee_mini_evo_manager_is_initialized(void);

/**
 * @brief Get current configuration
 * 
 * @param config Pointer to configuration structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t terabee_mini_evo_manager_get_config(system_terabee_mini_evo_config_t *config);

/**
 * @brief Set configuration
 * 
 * @param config Pointer to configuration structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t terabee_mini_evo_manager_set_config(const system_terabee_mini_evo_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* TERABEE_MINI_EVO_MANAGER_H */

