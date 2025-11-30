#ifndef FUSION_CORE_ASSEMBLY_H
#define FUSION_CORE_ASSEMBLY_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t g_assembly_data064[72];
extern uint8_t g_assembly_data096[40];

SemaphoreHandle_t fusion_core_get_assembly_mutex(void);
SemaphoreHandle_t scale_application_get_assembly_mutex(void);  // Backward compatibility

#define INPUT_ASSEMBLY_100 g_assembly_data064
#define OUTPUT_ASSEMBLY_150 g_assembly_data096

#define VL53L1X_BYTE_START 0
#define VL53L1X_BYTE_END 15

#define LSM6DS3_BYTE_START 16
#define LSM6DS3_BYTE_END 23

#define NAU7802_BYTE_START 24
#define NAU7802_BYTE_END 39

#define MCP230XX_FEEDBACK_BYTE_START 40
#define MCP230XX_FEEDBACK_BYTE_END 55

#define DEVICE_COUNT_BYTE_START 56
#define DEVICE_COUNT_BYTE_END 60

#define MCP230XX_OUTPUT_BYTE_START 0
#define MCP230XX_OUTPUT_BYTE_END 15

#define GP8403_DAC_BYTE_START 16
#define GP8403_DAC_BYTE_END 31

#ifdef __cplusplus
}
#endif

#endif

