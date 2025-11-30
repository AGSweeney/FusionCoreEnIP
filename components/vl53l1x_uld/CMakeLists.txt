idf_component_register(
  SRCS 
    "vl53l1x_uld_esp_wrapper/core/src/VL53L1X_api.c"
    "vl53l1x_uld_esp_wrapper/core/src/VL53L1X_calibration.c"
    "vl53l1x_uld_esp_wrapper/core/src/vl53l1_platform.c"
    "vl53l1x_uld_esp_wrapper/esp_wrapper/src/i2c_handler.c"
    "vl53l1x_uld_esp_wrapper/esp_wrapper/src/i2c_device_handler.c"
    "vl53l1x_uld_esp_wrapper/esp_wrapper/src/vl53l1x.c"

  INCLUDE_DIRS
    "vl53l1x_uld_esp_wrapper/core/include"
    "vl53l1x_uld_esp_wrapper/esp_wrapper/include"

  REQUIRES
    esp_driver_gpio  
    esp_driver_i2c
    vl53l1x_config
    system_config
)