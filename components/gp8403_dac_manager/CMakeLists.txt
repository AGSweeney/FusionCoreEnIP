idf_component_register(SRCS "gp8403_dac_manager.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver
                    REQUIRES i2c_bus_manager
                            gp8403_dac
                            system_config
                            opener
                            ota_manager)

