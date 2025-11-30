#include "mcp23017.h"

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#define MCP23017_I2C_TIMEOUT_MS 150
#define MCP23017_MAX_RETRIES 3

static esp_err_t write_reg(mcp23017_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t payload[2] = {reg, value};
    esp_err_t err;
    int retry_count = 0;
    
    do {
        err = i2c_master_transmit(dev->i2c_dev, payload, sizeof(payload), pdMS_TO_TICKS(MCP23017_I2C_TIMEOUT_MS));
        
        // Retry on timeout or bus errors (bus contention or device busy)
        if ((err == ESP_ERR_TIMEOUT || err == ESP_FAIL) && retry_count < MCP23017_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(1 << retry_count)); // Exponential backoff: 1ms, 2ms, 4ms
            retry_count++;
        } else {
            break;
        }
    } while (retry_count < MCP23017_MAX_RETRIES);
    
    return err;
}

static esp_err_t read_reg(mcp23017_t *dev, uint8_t reg, uint8_t *value)
{
    esp_err_t err;
    int retry_count = 0;
    
    do {
        err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, value, 1, pdMS_TO_TICKS(MCP23017_I2C_TIMEOUT_MS));
        
        // Retry on timeout or bus errors (bus contention or device busy)
        if ((err == ESP_ERR_TIMEOUT || err == ESP_FAIL) && retry_count < MCP23017_MAX_RETRIES) {
            vTaskDelay(pdMS_TO_TICKS(1 << retry_count)); // Exponential backoff: 1ms, 2ms, 4ms
            retry_count++;
        } else {
            break;
        }
    } while (retry_count < MCP23017_MAX_RETRIES);
    
    return err;
}

bool mcp23017_init(mcp23017_t *dev,
                   i2c_master_dev_handle_t i2c_dev,
                   const mcp23017_config_t *cfg)
{
    if (!dev || !i2c_dev)
    {
        return false;
    }
    dev->i2c_dev = i2c_dev;
    if (!cfg)
    {
        return true;
    }
    if (write_reg(dev, MCP23017_REG_IODIRA, cfg->iodir_a) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_IODIRB, cfg->iodir_b) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_IPOLA, cfg->ipol_a) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_IPOLB, cfg->ipol_b) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_GPINTENA, cfg->gpinten_a) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_GPINTENB, cfg->gpinten_b) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_DEFVALA, cfg->defval_a) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_DEFVALB, cfg->defval_b) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_INTCONA, cfg->intcon_a) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_INTCONB, cfg->intcon_b) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_IOCON, cfg->iocon) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_GPPUA, cfg->gppu_a) != ESP_OK)
    {
        return false;
    }
    if (write_reg(dev, MCP23017_REG_GPPUB, cfg->gppu_b) != ESP_OK)
    {
        return false;
    }
    return true;
}

esp_err_t mcp23017_write_register(mcp23017_t *dev, uint8_t reg, uint8_t value)
{
    return write_reg(dev, reg, value);
}

esp_err_t mcp23017_read_register(mcp23017_t *dev, uint8_t reg, uint8_t *value)
{
    return read_reg(dev, reg, value);
}

esp_err_t mcp23017_write_gpio(mcp23017_t *dev, uint16_t value)
{
    uint8_t high = (uint8_t)(value >> 8);
    uint8_t low = (uint8_t)(value & 0xFF);
    if (write_reg(dev, MCP23017_REG_GPIOA, low) != ESP_OK)
    {
        return ESP_FAIL;
    }
    return write_reg(dev, MCP23017_REG_GPIOB, high);
}

esp_err_t mcp23017_read_gpio(mcp23017_t *dev, uint16_t *value)
{
    uint8_t low = 0;
    uint8_t high = 0;
    if (read_reg(dev, MCP23017_REG_GPIOA, &low) != ESP_OK)
    {
        return ESP_FAIL;
    }
    if (read_reg(dev, MCP23017_REG_GPIOB, &high) != ESP_OK)
    {
        return ESP_FAIL;
    }
    *value = ((uint16_t)high << 8) | low;
    return ESP_OK;
}

static esp_err_t write_pair(mcp23017_t *dev, uint8_t reg_a, uint16_t value)
{
    esp_err_t err = write_reg(dev, reg_a, (uint8_t)(value & 0xFF));
    if (err != ESP_OK)
    {
        return err;
    }
    return write_reg(dev, reg_a + 1, (uint8_t)(value >> 8));
}

static esp_err_t read_pair(mcp23017_t *dev, uint8_t reg_a, uint16_t *value)
{
    uint8_t low = 0;
    uint8_t high = 0;
    esp_err_t err = read_reg(dev, reg_a, &low);
    if (err != ESP_OK)
    {
        return err;
    }
    err = read_reg(dev, reg_a + 1, &high);
    if (err != ESP_OK)
    {
        return err;
    }
    *value = ((uint16_t)high << 8) | low;
    return ESP_OK;
}

static esp_err_t update_pair(mcp23017_t *dev, uint8_t reg_a, uint16_t mask, uint16_t value)
{
    uint16_t current = 0;
    esp_err_t err = read_pair(dev, reg_a, &current);
    if (err != ESP_OK)
    {
        return err;
    }
    current = (current & ~mask) | (value & mask);
    return write_pair(dev, reg_a, current);
}

esp_err_t mcp23017_set_direction(mcp23017_t *dev, uint16_t mask)
{
    return write_pair(dev, MCP23017_REG_IODIRA, mask);
}

esp_err_t mcp23017_set_polarity(mcp23017_t *dev, uint16_t mask)
{
    return write_pair(dev, MCP23017_REG_IPOLA, mask);
}

esp_err_t mcp23017_set_pullups(mcp23017_t *dev, uint16_t mask)
{
    return write_pair(dev, MCP23017_REG_GPPUA, mask);
}

esp_err_t mcp23017_update_gpio_mask(mcp23017_t *dev, uint16_t mask, uint16_t value)
{
    return update_pair(dev, MCP23017_REG_OLATA, mask, value);
}

esp_err_t mcp23017_write_pin(mcp23017_t *dev, uint8_t pin, bool level)
{
    if (pin > 15)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t mask = (uint16_t)1U << pin;
    return mcp23017_update_gpio_mask(dev, mask, level ? mask : 0);
}

esp_err_t mcp23017_read_pin(mcp23017_t *dev, uint8_t pin, bool *level)
{
    if (pin > 15 || !level)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t value = 0;
    esp_err_t err = mcp23017_read_gpio(dev, &value);
    if (err != ESP_OK)
    {
        return err;
    }
    *level = (value >> pin) & 0x1;
    return ESP_OK;
}

