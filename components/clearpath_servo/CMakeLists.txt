# ClearPath Servo Driver Component
# 
# NOTE: This component is scaffolded but NOT included in the build.
# To enable it:
# 1. Remove this file from .gitignore or ensure it's tracked
# 2. Add component to main CMakeLists.txt or ensure ESP-IDF auto-discovers it
# 3. Integrate clearpath_servo_manager into main.c initialization

idf_component_register(
    SRCS "clearpath_servo.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver freertos
)
