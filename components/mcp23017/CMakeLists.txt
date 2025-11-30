idf_component_register(SRCS "mcp23017.c"
                              "mcp23017_init.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c system_config)

