# ClearPath Servo Manager Component
# 
# NOTE: This component is scaffolded but NOT included in the build.
# To enable it:
# 1. Remove this file from .gitignore or ensure it's tracked
# 2. Add component to main CMakeLists.txt or ensure ESP-IDF auto-discovers it
# 3. Integrate clearpath_servo_manager_init() into main.c startup sequence
# 4. Add servo configuration to system_config component

idf_component_register(
    SRCS "clearpath_servo_manager.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver freertos clearpath_servo
    REQUIRES opener system_config
)
