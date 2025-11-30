idf_component_register(SRCS "i2c_bus_manager.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver
                    REQUIRES esp_common system_config)

