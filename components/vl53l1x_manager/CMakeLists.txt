idf_component_register(SRCS "vl53l1x_manager.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver
                    REQUIRES i2c_bus_manager
                            vl53l1x_uld
                            vl53l1x_config
                            system_config
                            opener)

