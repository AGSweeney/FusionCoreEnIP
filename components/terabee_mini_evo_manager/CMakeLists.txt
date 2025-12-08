idf_component_register(SRCS "terabee_mini_evo_manager.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver
                    REQUIRES i2c_bus_manager
                            terabee_mini_evo
                            system_config
                            opener)

