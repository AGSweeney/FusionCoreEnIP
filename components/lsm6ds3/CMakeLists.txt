idf_component_register(
    SRCS 
        "lsm6ds3.c"
        "lsm6ds3_fusion.c"
        "driver/lsm6ds3_reg.c"
    INCLUDE_DIRS 
        "include"
        "driver"
    PRIV_REQUIRES 
        driver
        nvs_flash
)

