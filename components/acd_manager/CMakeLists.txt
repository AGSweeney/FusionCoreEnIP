idf_component_register(SRCS "acd_manager.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES lwip esp_netif driver
                    REQUIRES freertos opener)

