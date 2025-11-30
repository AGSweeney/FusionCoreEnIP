#include "i2c_handler.h"
#include "esp_log.h"
#include "VL53L1X_api.h"
#include "i2c_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include <string.h>

#include "i2c_device_handler.h"

static const char *TAG = "I2C_HANDLER";

#define VL53L1X_I2C_TIMEOUT_MS 1000
#define VL53L1X_MAX_RETRIES 3

bool i2c_master_init(vl53l1x_i2c_handle_t *i2c_handle)
{
    if (i2c_handle->bus_handle != NULL) {
        ESP_LOGI(TAG, "Using existing I2C bus handle");
        i2c_handle->initialized = true;
        return true;
    }

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_handle->i2c_port,
        .scl_io_num = i2c_handle->scl_gpio,
        .sda_io_num = i2c_handle->sda_gpio,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = i2c_config_get_internal_pullup_enabled(),  // System-wide I2C config
            .allow_pd = 0}};

    esp_err_t err = i2c_new_master_bus(&bus_config, &i2c_handle->bus_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to initialize i2c", esp_err_to_name(err));
        return false;
    }

    i2c_handle->initialized = true;
    return true;
}

bool i2c_add_device(vl53l1x_device_handle_t *device)
{
    esp_err_t err = i2c_master_probe(device->vl53l1x_handle->i2c_handle->bus_handle, device->i2c_address, 1000);
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Sensor found at address 0x%02X", device->i2c_address);
    }
    else
    {
        ESP_LOGE(TAG, "Sensor not responding at 0x%02X: %s", device->i2c_address, esp_err_to_name(err));
        ESP_LOGW(TAG, "sensor found at 0x%02X", i2c_find_first_addr(device->vl53l1x_handle->i2c_handle));

        return false;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device->i2c_address,
        .scl_speed_hz = device->scl_speed_hz};

    dev_handle_t dev_handle;
    err = i2c_master_bus_add_device(device->vl53l1x_handle->i2c_handle->bus_handle, &dev_config, &dev_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "failled to create i2c device: %s", esp_err_to_name(err));
        return false;
    }

    uint16_t dev = create_dev(device->vl53l1x_handle->i2c_handle->i2c_port, device->i2c_address);
    add_device_key(dev, dev_handle);
    device->dev = dev;
    return true;
}

bool i2c_update_address(const uint16_t dev, const uint8_t new_address)
{
    if (VL53L1X_SetI2CAddress(dev, new_address) != 0)
    {
        ESP_LOGE(TAG, "VL53L1X device failled to update address");

        return false;
    }

    VL53L1_WaitMs(dev, 5);

    dev_handle_t dev_handle = get_device_handle(dev);
    if (i2c_master_device_change_address(dev_handle, new_address, 1000) != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c bus failled to update device address");
        VL53L1X_SetI2CAddress(new_address, get_address(dev));
        return false;
    }

    change_device_address(dev, new_address);

    return true;
}

// TODO remove device
void i2c_remove_device(vl53l1x_i2c_handle_t *i2c_handle)
{
    ESP_LOGE(TAG, "not implemented");
}

uint8_t i2c_find_first_addr(vl53l1x_i2c_handle_t *i2c_handle)
{
    for (uint8_t address = 1; address < 0x78; address++)
    {
        if (i2c_master_probe(i2c_handle->bus_handle, address, 1000) == ESP_OK)
        {
            return address;
        }
    }
    return 0;
}

bool i2c_write(const dev_handle_t *device, uint16_t reg, uint8_t value)
{
    uint8_t data[3] =
        {
            reg >> 8,
            reg & 0xFF,
            value};
    
    esp_err_t err;
    int retry_count = 0;
    
    do {
        err = i2c_master_transmit(*device, data, 3, pdMS_TO_TICKS(VL53L1X_I2C_TIMEOUT_MS));
        
        // Retry on timeout errors (bus contention)
        if (err == ESP_ERR_TIMEOUT && retry_count < VL53L1X_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(1 << retry_count)); // Exponential backoff: 1ms, 2ms, 4ms
            retry_count++;
        } else {
            break;
        }
    } while (retry_count < VL53L1X_MAX_RETRIES);
    
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "write failed at: %04X - %02X: %s", reg, value, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool i2c_read(const dev_handle_t *device, uint16_t reg, uint8_t *buf, size_t len)
{
    uint8_t reg_bytes[2] = {
        reg >> 8,
        reg & 0xFF};

    dev_handle_t dev_value = *device;

    esp_err_t err;
    int retry_count = 0;
    
    do {
        err = i2c_master_transmit_receive(dev_value, reg_bytes, 2, buf, len, pdMS_TO_TICKS(VL53L1X_I2C_TIMEOUT_MS));
        
        // Retry on timeout errors (bus contention)
        if (err == ESP_ERR_TIMEOUT && retry_count < VL53L1X_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(1 << retry_count)); // Exponential backoff: 1ms, 2ms, 4ms
            retry_count++;
        } else {
            break;
        }
    } while (retry_count < VL53L1X_MAX_RETRIES);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c_read failed on reg 0x%04X: %s", reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

int8_t i2c_write_multi(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    dev_handle_t handle = get_device_handle(dev);

    if (handle == NULL)
    {
        return 255;
    }

    uint8_t buffer[2 + 32];
    if (count > 32)
    {
        ESP_LOGE(TAG, "i2c_write_multi: count %lu exceeds buffer size", count);
        return 255;
    }
    
    buffer[0] = (index >> 8) & 0xFF;
    buffer[1] = index & 0xFF;
    memcpy(&buffer[2], pdata, count);
    
    esp_err_t err;
    int retry_count = 0;
    
    do {
        err = i2c_master_transmit(handle, buffer, 2 + count, pdMS_TO_TICKS(VL53L1X_I2C_TIMEOUT_MS));
        
        // Retry on timeout errors (bus contention)
        if (err == ESP_ERR_TIMEOUT && retry_count < VL53L1X_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(1 << retry_count)); // Exponential backoff: 1ms, 2ms, 4ms
            retry_count++;
        } else {
            break;
        }
    } while (retry_count < VL53L1X_MAX_RETRIES);
    
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C write multi failed at index 0x%04X, count %lu: %s", index, count, esp_err_to_name(err));
        return 255;
    }

    return 0;
}

int8_t i2c_read_multi(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    dev_handle_t handle = get_device_handle(dev);

    if (handle == NULL)
    {
        return 255;
    }

    if (!i2c_read(&handle, index, pdata, count))
    {
        return 255;
    }

    return 0;
}

int8_t i2c_write_byte(uint16_t dev, uint16_t index, uint8_t data)
{
    return i2c_write_multi(dev, index, &data, 1);
}

int8_t i2c_write_word(uint16_t dev, uint16_t index, uint16_t data)
{
    // The VL53L1X expects 16-bit data to be sent MSB first (Big Endian format)
    uint8_t buffer[2];
    buffer[0] = (data >> 8) & 0xFF; // MSB
    buffer[1] = data & 0xFF;        // LSB

    return i2c_write_multi(dev, index, buffer, 2);
}

int8_t i2c_write_dword(uint16_t dev, uint16_t index, uint32_t data)
{
    // The VL53L1X expects 32-bit data to be sent MSB first (Big Endian format)
    uint8_t buffer[4];
    buffer[0] = (data >> 24) & 0xFF;
    buffer[1] = (data >> 16) & 0xFF;
    buffer[2] = (data >> 8) & 0xFF;
    buffer[3] = data & 0xFF;

    return i2c_write_multi(dev, index, buffer, 4);
}

int8_t i2c_read_byte(uint16_t dev, uint16_t index, uint8_t *pdata)
{
    return i2c_read_multi(dev, index, pdata, 1);
}

int8_t i2c_read_word(uint16_t dev, uint16_t index, uint16_t *pdata)
{
    uint8_t buffer[2];
    int8_t status = i2c_read_multi(dev, index, buffer, 2);

    if (status == 0)
    {
        // Data is received MSB first (Big Endian), reconstruct it into a uint16_t
        *pdata = ((uint16_t)buffer[0] << 8) | buffer[1];
    }
    return status;
}

int8_t i2c_read_dword(uint16_t dev, uint16_t index, uint32_t *pdata)
{
    uint8_t buffer[4];
    int8_t status = i2c_read_multi(dev, index, buffer, 4);

    if (status == 0)
    {
        // Data is received MSB first (Big Endian), reconstruct it into a uint32_t
        *pdata = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) |
                 ((uint32_t)buffer[2] << 8) | buffer[3];
    }
    return status;
}