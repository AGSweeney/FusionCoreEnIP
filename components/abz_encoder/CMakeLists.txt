# ABZ Encoder Driver Component
# 
# NOTE: This component is scaffolded but NOT included in the build.
# To enable it:
# 1. Remove this file from .gitignore or ensure it's tracked
# 2. Add component to main CMakeLists.txt or ensure ESP-IDF auto-discovers it
# 3. Integrate abz_encoder_manager into main.c initialization

idf_component_register(
    SRCS "abz_encoder.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver
)
