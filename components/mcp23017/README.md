# MCP23017 ESP-IDF Component

Minimal support library for the Microchip MCP23017 (16-bit I²C I/O expander). This component exposes register definitions and simple helpers to read/write the device over ESP-IDF’s `driver/i2c_master.h` API. It intentionally omits runtime logic (interrupt handling, tasks, etc.) so you can integrate it however you need.

## Directory Layout

```
components/mcp23017/
├── CMakeLists.txt
├── include/
│   └── mcp23017.h
└── mcp23017.c
```

## Public API

```c
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
} mcp23017_t;

typedef struct {
    uint8_t iodir_a;
    uint8_t iodir_b;
    uint8_t ipol_a;
    uint8_t ipol_b;
    uint8_t gpinten_a;
    uint8_t gpinten_b;
    uint8_t defval_a;
    uint8_t defval_b;
    uint8_t intcon_a;
    uint8_t intcon_b;
    uint8_t iocon;
    uint8_t gppu_a;
    uint8_t gppu_b;
} mcp23017_config_t;

bool mcp23017_init(mcp23017_t *dev,
                   i2c_master_dev_handle_t i2c_dev,
                   const mcp23017_config_t *cfg);

esp_err_t mcp23017_write_register(mcp23017_t *dev, uint8_t reg, uint8_t value);
esp_err_t mcp23017_read_register(mcp23017_t *dev, uint8_t reg, uint8_t *value);

esp_err_t mcp23017_write_gpio(mcp23017_t *dev, uint16_t value);
esp_err_t mcp23017_read_gpio(mcp23017_t *dev, uint16_t *value);
esp_err_t mcp23017_set_direction(mcp23017_t *dev, uint16_t mask);
esp_err_t mcp23017_set_polarity(mcp23017_t *dev, uint16_t mask);
esp_err_t mcp23017_set_pullups(mcp23017_t *dev, uint16_t mask);
esp_err_t mcp23017_update_gpio_mask(mcp23017_t *dev, uint16_t mask, uint16_t value);
esp_err_t mcp23017_write_pin(mcp23017_t *dev, uint8_t pin, bool level);
esp_err_t mcp23017_read_pin(mcp23017_t *dev, uint8_t pin, bool *level);
```

- `mcp23017_init`: pass an I²C device handle you created with `i2c_master_bus_add_device()`. Optionally supply a configuration structure to initialise direction, polarity, pull-ups, etc., in one go. Pass `NULL` to leave all registers untouched.
- Runtime helpers (`mcp23017_set_direction`, `mcp23017_set_pullups`, `mcp23017_set_polarity`) let you reconfigure the port without touching the rest of the register map.
- `mcp23017_update_gpio_mask` and the `write_pin` / `read_pin` wrappers simplify per-bit manipulation while preserving the untouched pins.
- `mcp23017_write_gpio` / `mcp23017_read_gpio`: treat the expander’s 16-bit port as a single value (GPIOB = upper byte, GPIOA = lower byte). Useful after configuring IODIR as outputs or inputs.
- `mcp23017_write_register` / `mcp23017_read_register`: direct access to any register when you need fine-grained control.

## Quick Example

```c
mcp23017_t expander;
mcp23017_config_t cfg = {
    .iodir_a = 0x0F,   // GPA0-3 inputs, GPA4-7 outputs
    .iodir_b = 0xF0,   // GPB4-7 inputs, GPB0-3 outputs
    .ipol_a = 0x00,
    .ipol_b = 0x00,
    .gpinten_a = 0x0F,
    .gpinten_b = 0xF0,
    .defval_a = 0x00,
    .defval_b = 0xF0,
    .intcon_a = 0x0F,
    .intcon_b = 0xF0,
    .iocon = 0x04,     // open-drain INT, sequential disabled
    .gppu_a = 0x0F,
    .gppu_b = 0xF0,
};

i2c_master_dev_handle_t dev_handle = /* created earlier */;

if (!mcp23017_init(&expander, dev_handle, &cfg)) {
    // handle error
}

// Drive lower four GPA pins high, others low
mcp23017_update_gpio_mask(&expander, 0x000F, 0x0005);

bool input_level = false;
mcp23017_read_pin(&expander, 12, &input_level);
```

## Notes

- Default I²C address is `0x20`. Tie address pins A0–A2 to select additional addresses if you have multiple expanders.
- Register map follows the device in “BANK=0” mode (default). If you set `BANK` in `IOCON`, you’ll need to adjust register addresses.
- All I²C calls block for up to 100 ms by default; tweak the timeout or wrap them with your own helpers if you need non-blocking behaviour.
- Interrupt handling is up to you. Configure `GPINTENx`, `INTCONx`, `DEFVALx`, and route the INT pin to a microcontroller GPIO to react to input changes.

For a lighter 8-bit variant, see the `mcp23008` component. Both share the same usage pattern and can coexist on the same I²C bus. 

