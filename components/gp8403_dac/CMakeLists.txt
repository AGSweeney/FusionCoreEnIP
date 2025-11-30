# CMakeLists.txt for GP8403 DAC Component
#
# DFRobot Gravity 2-Channel I2C DAC Module (0-10V) Driver
# Product: https://www.dfrobot.com/product-2613.html
# SKU: DFR0971
# Chip: GP8403

idf_component_register(
    SRCS 
        "gp8403_dac.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        driver
)

