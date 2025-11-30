# MCP23008 ESP-IDF Component

Minimal support library for the Microchip MCP23008 I²C 8-bit I/O expander. This component provides register definitions and simple read/write helpers; it does not include any runtime polling or interrupt handling logic.

## Directory Layout

```
components/mcp23008/
├── CMakeLists.txt
├── include/
│   └── mcp23008.h
└── mcp23008.c
```

## Public API

```c
typedef struct {
    i2c_master_dev_handle_t i2c_dev;
} mcp23008_t;

typedef struct {
    uint8_t iodir;    // 1 = input, 0 = output
    uint8_t ipol;     // 1 = inverted input
    uint8_t gpinten;  // interrupt-on-change enable
    uint8_t defval;   // default compare for interrupt
    uint8_t intcon;   // interrupt control (compare vs previous)
    uint8_t iocon;    // configuration bits
    uint8_t gppu;     // internal pull-ups
} mcp23008_config_t;

bool mcp23008_init(mcp23008_t *dev,
                   i2c_master_dev_handle_t i2c_dev,
                   const mcp23008_config_t *cfg);

esp_err_t mcp23008_write_register(mcp23008_t *dev, uint8_t reg, uint8_t value);
esp_err_t mcp23008_read_register(mcp23008_t *dev, uint8_t reg, uint8_t *value);

esp_err_t mcp23008_write_gpio(mcp23008_t *dev, uint8_t value);
esp_err_t mcp23008_read_gpio(mcp23008_t *dev, uint8_t *value);
esp_err_t mcp23008_set_direction(mcp23008_t *dev, uint8_t mask);
esp_err_t mcp23008_set_polarity(mcp23008_t *dev, uint8_t mask);
esp_err_t mcp23008_set_pullups(mcp23008_t *dev, uint8_t mask);
esp_err_t mcp23008_update_gpio_mask(mcp23008_t *dev, uint8_t mask, uint8_t value);
esp_err_t mcp23008_write_pin(mcp23008_t *dev, uint8_t pin, bool level);
esp_err_t mcp23008_read_pin(mcp23008_t *dev, uint8_t pin, bool *level);
```

- Call `mcp23008_init` once you have created an ESP-IDF master device handle (`i2c_master_bus_add_device`). Pass a `mcp23008_config_t` to initialise direction/pullups, or `NULL` to leave registers untouched.
- Use `mcp23008_set_direction`, `mcp23008_set_pullups`, and `mcp23008_set_polarity` at runtime to adjust the port configuration.
- `mcp23008_update_gpio_mask` (and the convenience helpers `mcp23008_write_pin` / `mcp23008_read_pin`) let you manipulate individual bits without affecting the remaining port state.
- `mcp23008_write_register` / `mcp23008_read_register` give you direct access to any register if you need lower-level control.

## Quick Example

```c
mcp23008_t expander;
mcp23008_config_t cfg = {
    .iodir = 0xF0,    // upper 4 bits input, lower 4 bits output
    .ipol = 0x00,
    .gpinten = 0xF0,  // interrupt on upper inputs (optional)
    .defval = 0x00,
    .intcon = 0xF0,
    .iocon = 0x04,    // open-drain INT, sequential disabled
    .gppu = 0xF0,     // pull-ups on input bits
};

i2c_master_dev_handle_t io_handle = /* created via i2c_master_bus_add_device */;
if (!mcp23008_init(&expander, io_handle, &cfg)) {
    // handle error
}

// Toggle lower nibble LEDs
mcp23008_update_gpio_mask(&expander, 0x0F, 0x05);

bool button_pressed = false;
mcp23008_read_pin(&expander, 6, &button_pressed);
```

## Notes

- Default I²C address is `0x20`; wire A0-A2 accordingly if you need multiple expanders.
- All I²C operations use blocking `i2c_master_transmit` / `i2c_master_transmit_receive` with a default 100 ms timeout.
- Interrupt support is not provided here; enable it via `gpinten`, route the INT pin to a GPIO, and handle the interrupt in your application if required.
- If you need MCP23017 (16-bit) support, see the `mcp23017` companion component.

