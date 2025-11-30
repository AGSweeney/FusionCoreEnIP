idf_component_register(SRCS "lsm6ds3_manager.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver
                    REQUIRES i2c_bus_manager
                            lsm6ds3
                            system_config
                            opener)

