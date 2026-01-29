# CMakeLists.txt for MAX7219 Component
#
# MAX7219 7-Segment Display Driver
# Chip: MAX7219
# Interface: SPI

idf_component_register(
    SRCS 
        "max7219.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        driver
)
