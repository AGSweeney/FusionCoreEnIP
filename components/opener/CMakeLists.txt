cmake_minimum_required(VERSION 3.16)

set(OPENER_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(OPENER_PORTS_DIR "${OPENER_SRC_DIR}/ports")
set(OPENER_ESP32_DIR "${OPENER_PORTS_DIR}/ESP32")

set(ESP32_PORT_SRCS
    "${OPENER_ESP32_DIR}/opener.c"
    "${OPENER_ESP32_DIR}/networkhandler.c"
    "${OPENER_ESP32_DIR}/networkconfig.c"
    "${OPENER_ESP32_DIR}/opener_error.c"
    "${OPENER_ESP32_DIR}/eth_media_counters.c"
    "${OPENER_ESP32_DIR}/fusion_core/fusioncore.c"
    "${OPENER_ESP32_DIR}/fusion_core/include/fusion_core_assembly.h"
)

set(PORTS_GENERIC_SRCS
    "${OPENER_PORTS_DIR}/generic_networkhandler.c"
    "${OPENER_PORTS_DIR}/socket_timer.c"
)

set(CIP_SRCS
    "${OPENER_SRC_DIR}/cip/appcontype.c"
    "${OPENER_SRC_DIR}/cip/cipassembly.c"
    "${OPENER_SRC_DIR}/cip/cipclass3connection.c"
    "${OPENER_SRC_DIR}/cip/cipcommon.c"
    "${OPENER_SRC_DIR}/cip/cipconnectionmanager.c"
    "${OPENER_SRC_DIR}/cip/cipconnectionobject.c"
    "${OPENER_SRC_DIR}/cip/cipdlr.c"
    "${OPENER_SRC_DIR}/cip/cipelectronickey.c"
    "${OPENER_SRC_DIR}/cip/cipepath.c"
    "${OPENER_SRC_DIR}/cip/cipethernetlink.c"
    "${OPENER_SRC_DIR}/cip/cipidentity.c"
    "${OPENER_SRC_DIR}/cip/cipioconnection.c"
    "${OPENER_SRC_DIR}/cip/cipmessagerouter.c"
    "${OPENER_SRC_DIR}/cip/cipparameter.c"
    "${OPENER_SRC_DIR}/cip/cipport.c"
    "${OPENER_SRC_DIR}/cip/cipqos.c"
    "${OPENER_SRC_DIR}/cip/cipstring.c"
    "${OPENER_SRC_DIR}/cip/cipstringi.c"
    "${OPENER_SRC_DIR}/cip/ciptcpipinterface.c"
    "${OPENER_SRC_DIR}/cip/ciptypes.c"
)

set(ENET_ENCAP_SRCS
    "${OPENER_SRC_DIR}/enet_encap/cpf.c"
    "${OPENER_SRC_DIR}/enet_encap/encap.c"
    "${OPENER_SRC_DIR}/enet_encap/endianconv.c"
)

set(UTILS_SRCS
    "${OPENER_SRC_DIR}/utils/doublylinkedlist.c"
    "${OPENER_SRC_DIR}/utils/enipmessage.c"
    "${OPENER_SRC_DIR}/utils/random.c"
    "${OPENER_SRC_DIR}/utils/xorshiftrandom.c"
)

set(NVDATA_SRCS
    "${OPENER_PORTS_DIR}/nvdata/conffile.c"
    "${OPENER_PORTS_DIR}/nvdata/nvdata.c"
    "${OPENER_PORTS_DIR}/nvdata/nvqos.c"
    "${OPENER_PORTS_DIR}/nvdata/nvtcpip.c"
)

# File Object sources (conditionally included based on CIP_FILE_OBJECT)
set(CIP_FILE_OBJECT_SRCS
    "${OPENER_SRC_DIR}/cip_objects/OpENerFileObject/cipfile.c"
    "${OPENER_SRC_DIR}/cip_objects/OpENerFileObject/memfile.c"
)

# Check for optional dependencies by checking if component directories exist
# These are project-specific and may not be available in all projects
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../system_config/CMakeLists.txt" OR EXISTS "${IDF_PATH}/components/system_config/CMakeLists.txt")
    set(has_system_config TRUE)
else()
    set(has_system_config FALSE)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../lldp/CMakeLists.txt" OR EXISTS "${IDF_PATH}/components/lldp/CMakeLists.txt")
    set(has_lldp TRUE)
else()
    set(has_lldp FALSE)
endif()

# Check if EDS file path is provided (for standalone component usage)
if(DEFINED OPENER_EDS_FILE_PATH AND EXISTS "${OPENER_EDS_FILE_PATH}")
    set(OPENER_GENERATE_EDS TRUE)
else()
    set(OPENER_GENERATE_EDS FALSE)
endif()

# Generate embedded EDS file from source EDS file (optional)
# For standalone component usage, set OPENER_EDS_FILE_PATH and OPENER_ICON_FILE_PATH
# Otherwise, EDS generation is handled in the main CMakeLists.txt
set(OPENER_BUILD_DIR "${CMAKE_BINARY_DIR}/opener")
set(EMBEDDED_EDS_HEADER "${OPENER_BUILD_DIR}/embedded_eds_file.h")
set(EMBEDDED_EDS_SOURCE "${OPENER_BUILD_DIR}/embedded_eds_file.c")
set(EMBEDDED_ICON_HEADER "${OPENER_BUILD_DIR}/embedded_icon_file.h")
set(EMBEDDED_ICON_SOURCE "${OPENER_BUILD_DIR}/embedded_icon_file.c")

# Create the build directory early so it exists for include paths
file(MAKE_DIRECTORY "${OPENER_BUILD_DIR}")

# Generate EDS files if OPENER_EDS_FILE_PATH is provided
if(OPENER_GENERATE_EDS)
    find_package(Python3 QUIET COMPONENTS Interpreter)
    if(NOT Python3_Interpreter_FOUND)
        set(PYTHON_CMD "python")
    else()
        set(PYTHON_CMD "${Python3_EXECUTABLE}")
    endif()

    set(EMBED_EDS_SCRIPT "${OPENER_SRC_DIR}/cip_objects/OpENerFileObject/embed_eds.py")
    set(CREATE_ICON_GZ_SCRIPT "${OPENER_SRC_DIR}/cip_objects/OpENerFileObject/create_icon_gz.py")
    set(EMBED_ICON_GZ_SCRIPT "${OPENER_SRC_DIR}/cip_objects/OpENerFileObject/embed_icon_gz.py")

    add_custom_command(
        OUTPUT "${EMBEDDED_EDS_HEADER}" "${EMBEDDED_EDS_SOURCE}"
        COMMAND ${PYTHON_CMD} "${EMBED_EDS_SCRIPT}"
                "${OPENER_EDS_FILE_PATH}"
                "${EMBEDDED_EDS_HEADER}"
                "${EMBEDDED_EDS_SOURCE}"
        DEPENDS "${OPENER_EDS_FILE_PATH}"
                "${EMBED_EDS_SCRIPT}"
        COMMENT "Embedding EDS file into build"
        VERBATIM
    )

    if(DEFINED OPENER_ICON_FILE_PATH AND EXISTS "${OPENER_ICON_FILE_PATH}")
        set(ICON_GZ_FILE_PATH "${OPENER_BUILD_DIR}/EDSCollection.gz")
        
        add_custom_command(
            OUTPUT "${ICON_GZ_FILE_PATH}"
            COMMAND ${PYTHON_CMD} "${CREATE_ICON_GZ_SCRIPT}"
                    "${OPENER_ICON_FILE_PATH}"
                    "${ICON_GZ_FILE_PATH}"
            DEPENDS "${CREATE_ICON_GZ_SCRIPT}" "${OPENER_ICON_FILE_PATH}"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            COMMENT "Creating EDSCollection.gz from icon file"
            VERBATIM
        )

        set_source_files_properties("${ICON_GZ_FILE_PATH}" PROPERTIES GENERATED TRUE)

        add_custom_command(
            OUTPUT "${EMBEDDED_ICON_HEADER}" "${EMBEDDED_ICON_SOURCE}"
            COMMAND ${PYTHON_CMD} "${EMBED_ICON_GZ_SCRIPT}"
                    "${ICON_GZ_FILE_PATH}"
                    "${EMBEDDED_ICON_HEADER}"
                    "${EMBEDDED_ICON_SOURCE}"
            DEPENDS "${EMBED_ICON_GZ_SCRIPT}" "${ICON_GZ_FILE_PATH}"
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            COMMENT "Generating embedded icon GZ file"
            VERBATIM
        )
    else()
        file(WRITE "${EMBEDDED_ICON_HEADER}" "// Icon file not provided - empty placeholder\n")
        file(WRITE "${EMBEDDED_ICON_SOURCE}" "// Icon file not provided - empty placeholder\n")
    endif()

    list(APPEND CIP_FILE_OBJECT_SRCS "${EMBEDDED_EDS_SOURCE}" "${EMBEDDED_ICON_SOURCE}")
else()
    list(APPEND CIP_FILE_OBJECT_SRCS "${EMBEDDED_EDS_SOURCE}" "${EMBEDDED_ICON_SOURCE}")
endif()

idf_component_register(
    SRCS 
        ${ESP32_PORT_SRCS}
        ${PORTS_GENERIC_SRCS}
        ${CIP_SRCS}
        ${ENET_ENCAP_SRCS}
        ${UTILS_SRCS}
        ${NVDATA_SRCS}
        ${CIP_FILE_OBJECT_SRCS}
    INCLUDE_DIRS 
        "${OPENER_SRC_DIR}"
        "${OPENER_PORTS_DIR}"
        "${OPENER_ESP32_DIR}"
        "${OPENER_ESP32_DIR}/fusion_core"
        "${OPENER_ESP32_DIR}/fusion_core/include"
        "${OPENER_SRC_DIR}/cip"
        "${OPENER_SRC_DIR}/enet_encap"
        "${OPENER_SRC_DIR}/utils"
        "${OPENER_PORTS_DIR}/nvdata"
        "${OPENER_SRC_DIR}/cip_objects/OpENerFileObject"
        "${OPENER_BUILD_DIR}"  # For generated embedded_eds_file.h
    REQUIRES 
        lwip
        freertos
        esp_eth
        esp_netif
        driver
        nvs_flash
        system_config
        lldp
    PRIV_REQUIRES
        lwip
        freertos
)

# Handle optional dependencies
# system_config and lldp are project-specific components
# If present in the project, they will be automatically detected
# Otherwise, define OPENER_SYSTEM_CONFIG_ENABLED=0 to disable system_config usage
# and OPENER_LLDP_ENABLED=0 to disable LLDP support

if(has_system_config)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE OPENER_SYSTEM_CONFIG_ENABLED=1)
elseif(DEFINED OPENER_SYSTEM_CONFIG_ENABLED)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE OPENER_SYSTEM_CONFIG_ENABLED=${OPENER_SYSTEM_CONFIG_ENABLED})
else()
    target_compile_definitions(${COMPONENT_LIB} PRIVATE OPENER_SYSTEM_CONFIG_ENABLED=0)
endif()

if(has_lldp)
    # LLDP is already conditionally compiled in opener.c using #if OPENER_LLDP_ENABLED
    # Default is enabled in opener_user_conf.h if not defined
else()
    # If lldp component not found, disable LLDP support
    target_compile_definitions(${COMPONENT_LIB} PRIVATE OPENER_LLDP_ENABLED=0)
endif()

target_compile_definitions(${COMPONENT_LIB} PRIVATE ESP32 CIP_FILE_OBJECT=1)
# EDS file is now embedded in the build - no SPIFFS needed
# Note: EDS file generation is handled in the main CMakeLists.txt to avoid ESP-IDF requirements phase issues

# Mark the generated files as GENERATED so CMake doesn't check for them during configuration
# These files are always generated (either by main CMakeLists.txt or by this component)
# This must be done after idf_component_register
set_source_files_properties("${EMBEDDED_EDS_SOURCE}" PROPERTIES GENERATED TRUE)
set_source_files_properties("${EMBEDDED_EDS_HEADER}" PROPERTIES GENERATED TRUE)
set_source_files_properties("${EMBEDDED_ICON_SOURCE}" PROPERTIES GENERATED TRUE)
set_source_files_properties("${EMBEDDED_ICON_HEADER}" PROPERTIES GENERATED TRUE)

