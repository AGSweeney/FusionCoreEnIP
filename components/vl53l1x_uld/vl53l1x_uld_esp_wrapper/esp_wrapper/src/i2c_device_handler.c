#include "i2c_device_handler.h"
#include "esp_log.h"
static const char *TAG = "I2C_DEVICE_HANDLER";

static device_map_t device_map = DEVICE_MAP_INIT;

void deinit_device_map()
{
    free(device_map.keys);
    device_map = DEVICE_MAP_INIT;
}

void add_device_key(const uint16_t dev, const dev_handle_t handle)
{
    uint8_t size = (device_map.count + 1) * sizeof(device_key_t);
    void *new_ptr = realloc(device_map.keys, size);

    if (new_ptr == NULL)
    {
        ESP_LOGE(TAG, "Failed to expand map");
        return; // Handle allocation error
    }

    device_map.keys = (device_key_t *)new_ptr;

    device_key_t key = {
        .dev = dev,
        .dev_handle = handle};

    device_map.keys[device_map.count] = key;
    device_map.count++;
}

int8_t remove_device_key(const uint16_t dev)
{
    int position = -1;
    for (uint8_t i = 0; i < device_map.count; i++)
    {
        if (device_map.keys[i].dev == dev)
        {
            position = i;
            break;
        }
    }

    if (position == -1)
    {
        ESP_LOGE(TAG, "address not found");

        return 255;
    }

    if (device_map.count < 2)
    {
        deinit_device_map(device_map);
        return 0;
    }

    uint8_t elements_to_move = device_map.count - position - 1;
    if (elements_to_move > 0)
    {
        memmove(
            &device_map.keys[position],
            &device_map.keys[position + 1],
            elements_to_move * sizeof(device_key_t));
    }

    device_map.count--;

    size_t new_size = device_map.count * sizeof(device_key_t);
    void *new_ptr = realloc(device_map.keys, new_size);

    if (new_size > 0 && new_ptr == NULL)
    {
        ESP_LOGE(TAG, "Failed to shrink map memory");
    }
    else
    {
        device_map.keys = (device_key_t *)new_ptr;
    }

    return 0;
}

void change_device_address(const uint16_t dev, const uint8_t new_address)
{
    for (size_t i = 0; i < device_map.count; i++)
    {
        if (device_map.keys[i].dev == dev)
        {
            device_map.keys[i].dev = create_dev(get_port(dev), new_address);
        }
    }
}

dev_handle_t get_device_handle(const uint16_t dev)
{
    for (size_t i = 0; i < device_map.count; i++)
    {
        if (device_map.keys[i].dev == dev)
        {
            return device_map.keys[i].dev_handle;
        }
    }
    return NULL;
}

uint16_t create_dev(const i2c_port_num_t port, const uint8_t address)
{
    return ((uint16_t)port << 8) | address;
}

uint8_t get_address(const uint16_t dev)
{
    return (uint8_t)(dev & 0xFF);
}

i2c_port_num_t get_port(const uint16_t dev)
{
    return (i2c_port_num_t)(dev >> 8);
}
