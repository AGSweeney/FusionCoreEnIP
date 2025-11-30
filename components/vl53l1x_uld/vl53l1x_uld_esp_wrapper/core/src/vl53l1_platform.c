/**
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "vl53l1_platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "VL53L1X_PLATFORM";

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	if (g_vl53l1x_write_multi_ptr == NULL)
	{
		ESP_LOGE(TAG, "g_vl53l1x_write_multi_ptr is null, for default implementation assign i2c_write_multi, from i2c_handler.h, before initializing any VL53L1 device");
		return 255;
	}

	return g_vl53l1x_write_multi_ptr(dev, index, pdata, count);
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	if (g_vl53l1x_read_multi_ptr == NULL)
	{
		ESP_LOGE(TAG, "g_vl53l1x_read_multi_ptr is null, for default implementation assign i2c_read_multi, from i2c_handler.h, before initializing any VL53L1 device");
		return 255;
	}

	return g_vl53l1x_read_multi_ptr(dev, index, pdata, count);
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
	if (g_vl53l1x_write_byte_ptr == NULL)
	{
		ESP_LOGE(TAG, "g_vl53l1x_write_byte_ptr is null, for default implementation assign i2c_write_byte, from i2c_handler.h, before initializing any VL53L1 device");
		return 255;
	}

	return g_vl53l1x_write_byte_ptr(dev, index, data);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
	if (g_vl53l1x_write_word_ptr == NULL)
	{
		ESP_LOGE(TAG, "g_vl53l1x_write_word_ptr is null, for default implementation assign i2c_write_word, from i2c_handler.h, before initializing any VL53L1 device");
		return 255;
	}

	return g_vl53l1x_write_word_ptr(dev, index, data);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
	if (g_vl53l1x_write_dword_ptr == NULL)
	{
		ESP_LOGE(TAG, "g_vl53l1x_write_dword_ptr is null, for default implementation assign i2c_write_dword, from i2c_handler.h, before initializing any VL53L1 device");
		return 255;
	}

	return g_vl53l1x_write_dword_ptr(dev, index, data);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
	if (g_vl53l1x_read_byte_ptr == NULL)
	{
		ESP_LOGE(TAG, "g_vl53l1x_read_byte_ptr is null, for default implementation assign i2c_read_byte, from i2c_handler.h, before initializing any VL53L1 device");
		return 255;
	}

	return g_vl53l1x_read_byte_ptr(dev, index, data);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
	if (g_vl53l1x_read_word_ptr == NULL)
	{
		ESP_LOGE(TAG, "g_vl53l1x_read_word_ptr is null, for default implementation assign i2c_read_word, from i2c_handler.h, before initializing any VL53L1 device");
		return 255;
	}

	return g_vl53l1x_read_word_ptr(dev, index, data);
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
	if (g_vl53l1x_read_dword_ptr == NULL)
	{
		ESP_LOGE(TAG, "g_vl53l1x_read_dword_ptr is null, for default implementation assign i2c_read_dword, from i2c_handler.h, before initializing any VL53L1 device");
		return 255;
	}

	return g_vl53l1x_read_dword_ptr(dev, index, data);
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
	vTaskDelay(pdMS_TO_TICKS(wait_ms));
	return 0;
}

vl53l1x_write_multi g_vl53l1x_write_multi_ptr = NULL;
vl53l1x_read_multi g_vl53l1x_read_multi_ptr = NULL;
vl53l1x_write_byte g_vl53l1x_write_byte_ptr = NULL;
vl53l1x_write_word g_vl53l1x_write_word_ptr = NULL;
vl53l1x_write_dword g_vl53l1x_write_dword_ptr = NULL;
vl53l1x_read_byte g_vl53l1x_read_byte_ptr = NULL;
vl53l1x_read_word g_vl53l1x_read_word_ptr = NULL;
vl53l1x_read_dword g_vl53l1x_read_dword_ptr = NULL;