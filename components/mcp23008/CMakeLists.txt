idf_component_register(SRCS "mcp23008.c"
                              "mcp23008_init.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c)

