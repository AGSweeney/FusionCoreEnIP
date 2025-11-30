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

/**
 * @file  vl53l1_platform.h
 * @brief Those platform functions are platform dependent and have to be implemented by the user
 */

#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

#include "vl53l1_types.h"
#include "i2c_handler.h"

#ifdef __cplusplus
extern "C"
{
#endif

	typedef struct
	{
		uint32_t dummy;
	} VL53L1_Dev_t;

	typedef VL53L1_Dev_t *VL53L1_DEV;

	/** @brief VL53L1_WriteMulti() definition.\n
	 * To be implemented by the developer
	 */
	int8_t VL53L1_WriteMulti(
		uint16_t dev,
		uint16_t index,
		uint8_t *pdata,
		uint32_t count);
	/** @brief VL53L1_ReadMulti() definition.\n
	 * To be implemented by the developer
	 */
	int8_t VL53L1_ReadMulti(
		uint16_t dev,
		uint16_t index,
		uint8_t *pdata,
		uint32_t count);
	/** @brief VL53L1_WrByte() definition.\n
	 * To be implemented by the developer
	 */
	int8_t VL53L1_WrByte(
		uint16_t dev,
		uint16_t index,
		uint8_t data);
	/** @brief VL53L1_WrWord() definition.\n
	 * To be implemented by the developer
	 */
	int8_t VL53L1_WrWord(
		uint16_t dev,
		uint16_t index,
		uint16_t data);
	/** @brief VL53L1_WrDWord() definition.\n
	 * To be implemented by the developer
	 */
	int8_t VL53L1_WrDWord(
		uint16_t dev,
		uint16_t index,
		uint32_t data);
	/** @brief VL53L1_RdByte() definition.\n
	 * To be implemented by the developer
	 */
	int8_t VL53L1_RdByte(
		uint16_t dev,
		uint16_t index,
		uint8_t *pdata);
	/** @brief VL53L1_RdWord() definition.\n
	 * To be implemented by the developer
	 */
	int8_t VL53L1_RdWord(
		uint16_t dev,
		uint16_t index,
		uint16_t *pdata);
	/** @brief VL53L1_RdDWord() definition.\n
	 * To be implemented by the developer
	 */
	int8_t VL53L1_RdDWord(
		uint16_t dev,
		uint16_t index,
		uint32_t *pdata);
	/** @brief VL53L1_WaitMs() definition.\n
	 * To be implemented by the developer
	 */
	int8_t VL53L1_WaitMs(
		uint16_t dev,
		int32_t wait_ms);

	typedef int8_t (*vl53l1x_write_multi)(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
	typedef int8_t (*vl53l1x_read_multi)(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
	typedef int8_t (*vl53l1x_write_byte)(uint16_t dev, uint16_t index, uint8_t data);
	typedef int8_t (*vl53l1x_write_word)(uint16_t dev, uint16_t index, uint16_t data);
	typedef int8_t (*vl53l1x_write_dword)(uint16_t dev, uint16_t index, uint32_t data);
	typedef int8_t (*vl53l1x_read_byte)(uint16_t dev, uint16_t index, uint8_t *pdata);
	typedef int8_t (*vl53l1x_read_word)(uint16_t dev, uint16_t index, uint16_t *pdata);
	typedef int8_t (*vl53l1x_read_dword)(uint16_t dev, uint16_t index, uint32_t *pdata);

	extern vl53l1x_write_multi g_vl53l1x_write_multi_ptr;
	extern vl53l1x_read_multi g_vl53l1x_read_multi_ptr;
	extern vl53l1x_write_byte g_vl53l1x_write_byte_ptr;
	extern vl53l1x_write_word g_vl53l1x_write_word_ptr;
	extern vl53l1x_write_dword g_vl53l1x_write_dword_ptr;
	extern vl53l1x_read_byte g_vl53l1x_read_byte_ptr;
	extern vl53l1x_read_word g_vl53l1x_read_word_ptr;
	extern vl53l1x_read_dword g_vl53l1x_read_dword_ptr;

#ifdef __cplusplus
}
#endif

#endif
