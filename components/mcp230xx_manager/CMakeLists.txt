idf_component_register(SRCS "mcp230xx_manager.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver
                    REQUIRES i2c_bus_manager
                            mcp23008
                            mcp23017
                            system_config
                            opener)

