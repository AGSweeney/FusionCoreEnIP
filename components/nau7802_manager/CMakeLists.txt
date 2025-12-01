idf_component_register(SRCS "nau7802_manager.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver
                    REQUIRES i2c_bus_manager
                            nau7802
                            system_config
                            opener
                            ota_manager)

