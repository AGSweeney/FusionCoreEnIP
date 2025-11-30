#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "vl53l1x.h"
#include "VL53L1X_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "VL53L1X_EXAMPLE"

#define VL53L1X_SCL_PIN GPIO_NUM_6
#define VL53L1X_SDA_PIN GPIO_NUM_5

static vl53l1x_handle_t vl53l1x_handle = VL53L1X_INIT;
static vl53l1x_device_handle_t vl53l1x_device = VL53L1X_DEVICE_INIT;

void app_main(void)
{
    ESP_LOGI(TAG, "VL53L1X Example Application");
    ESP_LOGI(TAG, "Initializing sensor...");

    vl53l1x_i2c_handle_t vl53l1x_i2c_handle = VL53L1X_I2C_INIT;
    vl53l1x_i2c_handle.scl_gpio = VL53L1X_SCL_PIN;
    vl53l1x_i2c_handle.sda_gpio = VL53L1X_SDA_PIN;

    vl53l1x_handle.i2c_handle = &vl53l1x_i2c_handle;
    if (!vl53l1x_init(&vl53l1x_handle))
    {
        ESP_LOGE(TAG, "VL53L1X initialization failed");
        return;
    }
    ESP_LOGI(TAG, "VL53L1X handle initialized");

    vl53l1x_device.vl53l1x_handle = &vl53l1x_handle;
    vl53l1x_device.i2c_address = 0x29;
    if (!vl53l1x_add_device(&vl53l1x_device))
    {
        ESP_LOGE(TAG, "Failed to add VL53L1X device");
        return;
    }
    ESP_LOGI(TAG, "VL53L1X device added successfully");

    vl53l1x_log_sensor_id(&vl53l1x_device);
    vl53l1x_log_ambient_light(&vl53l1x_device);
    vl53l1x_log_signal_rate(&vl53l1x_device);

    uint8_t roi_center = 0;
    if (vl53l1x_get_roi_center(&vl53l1x_device, &roi_center))
    {
        ESP_LOGI(TAG, "Current ROI center: SPAD %d", roi_center);
    }

    vl53l1x_set_roi(&vl53l1x_device, 8, 8);
    vl53l1x_set_roi_center(&vl53l1x_device, 199);
    ESP_LOGI(TAG, "ROI configured: 8x8, center at SPAD 199");

    #define ENABLE_CALIBRATION 0
    #define CALIBRATION_DISTANCE_MM 100

    if (ENABLE_CALIBRATION)
    {
        ESP_LOGI(TAG, "=== CALIBRATION MODE ===");
        ESP_LOGI(TAG, "Stopping ranging for calibration...");
        VL53L1X_StopRanging(vl53l1x_device.dev);
        vTaskDelay(pdMS_TO_TICKS(100));

        ESP_LOGI(TAG, "Preparing for offset calibration...");
        ESP_LOGI(TAG, "Please place a target at exactly %dmm from the sensor", CALIBRATION_DISTANCE_MM);
        ESP_LOGI(TAG, "Waiting 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000));

        int16_t offset = 0;
        if (vl53l1x_calibrate_offset(&vl53l1x_device, CALIBRATION_DISTANCE_MM, &offset))
        {
            ESP_LOGI(TAG, "Offset calibration completed successfully!");
            ESP_LOGI(TAG, "Calibrated offset: %d mm", offset);
        }
        else
        {
            ESP_LOGE(TAG, "Offset calibration failed!");
        }

        ESP_LOGI(TAG, "Preparing for xtalk calibration...");
        ESP_LOGI(TAG, "Please place a target at the inflection point (typically 600mm)");
        ESP_LOGI(TAG, "Waiting 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000));

        uint16_t xtalk = 0;
        if (vl53l1x_calibrate_xtalk(&vl53l1x_device, 600, &xtalk))
        {
            ESP_LOGI(TAG, "Xtalk calibration completed successfully!");
            ESP_LOGI(TAG, "Calibrated xtalk: %d cps", xtalk);
        }
        else
        {
            ESP_LOGE(TAG, "Xtalk calibration failed!");
        }

        ESP_LOGI(TAG, "=== CALIBRATION COMPLETE ===");
        ESP_LOGI(TAG, "Restarting ranging...");
        VL53L1X_StartRanging(vl53l1x_device.dev);
        ESP_LOGI(TAG, "Starting measurements in 2 seconds...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    else
    {
        ESP_LOGI(TAG, "Calibration disabled (set ENABLE_CALIBRATION to 1 to enable)");
    }

    ESP_LOGI(TAG, "Starting measurement loop...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    int measurement_count = 0;
    const TickType_t loop_delay_ticks = pdMS_TO_TICKS(100);

    while (1)
    {
        if (vl53l1x_handle.initialized)
        {
            uint8_t data_ready = 0;
            VL53L1X_ERROR status = VL53L1X_CheckForDataReady(vl53l1x_device.dev, &data_ready);

            if (status != 0)
            {
                ESP_LOGE(TAG, "CheckForDataReady failed with status: %d", status);
            }

            if (data_ready)
            {
                uint16_t distance = vl53l1x_get_mm(&vl53l1x_device);
                measurement_count++;

                if (distance > 0 && distance < 4000)
                {
                    ESP_LOGI(TAG, "Measurement #%d: Distance: %d mm", measurement_count, distance);
                }
                else if (distance == 0)
                {
                    ESP_LOGW(TAG, "Measurement #%d: Distance: 0 mm (no target or out of range)", measurement_count);
                }
                else
                {
                    ESP_LOGW(TAG, "Measurement #%d: Distance: %d mm (possibly invalid)", measurement_count, distance);
                }
            }
            else
            {
                if (measurement_count == 0)
                {
                    ESP_LOGI(TAG, "Waiting for first measurement (data not ready yet)...");
                }
            }
        }
        else
        {
            ESP_LOGW(TAG, "VL53L1X not initialized");
        }
        vTaskDelay(loop_delay_ticks);
    }
}

