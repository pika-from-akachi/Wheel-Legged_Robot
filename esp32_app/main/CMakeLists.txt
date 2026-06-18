set(ROBOEYES_GIT_TAG "b42f8e596535234932be3514ac7a813d4ced0046")
set(ROBOEYES_SOURCE_DIR "" CACHE PATH "Path to FluxGarage RoboEyes source checkout")
set(ROBOEYES_MANAGED_BY_BUILD OFF)

if("${ROBOEYES_SOURCE_DIR}" STREQUAL "")
    set(ROBOEYES_SOURCE_DIR "${CMAKE_BINARY_DIR}/_deps/roboeyes-src")
    set(ROBOEYES_MANAGED_BY_BUILD ON)
endif()

if(ROBOEYES_MANAGED_BY_BUILD AND NOT EXISTS "${ROBOEYES_SOURCE_DIR}/src/FluxGarage_RoboEyes.h")
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/_deps")
    execute_process(
        COMMAND git clone https://github.com/FluxGarage/RoboEyes.git "${ROBOEYES_SOURCE_DIR}"
        RESULT_VARIABLE ROBOEYES_CLONE_RESULT
    )
    if(NOT ROBOEYES_CLONE_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to clone FluxGarage RoboEyes. Set -DROBOEYES_SOURCE_DIR=/path/to/RoboEyes to use a local checkout.")
    endif()
endif()

if(ROBOEYES_MANAGED_BY_BUILD)
    execute_process(
        COMMAND git -C "${ROBOEYES_SOURCE_DIR}" checkout ${ROBOEYES_GIT_TAG}
        RESULT_VARIABLE ROBOEYES_CHECKOUT_RESULT
    )
    if(NOT ROBOEYES_CHECKOUT_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to checkout FluxGarage RoboEyes ${ROBOEYES_GIT_TAG}")
    endif()
endif()

if(NOT EXISTS "${ROBOEYES_SOURCE_DIR}/src/FluxGarage_RoboEyes.h")
    message(FATAL_ERROR "FluxGarage_RoboEyes.h not found under ${ROBOEYES_SOURCE_DIR}/src")
endif()

set(ROBOEYES_INCLUDE_DIR "${ROBOEYES_SOURCE_DIR}/src")
if("${ROBOEYES_INCLUDE_DIR}" MATCHES " ")
    set(ROBOEYES_INCLUDE_DIR "${CMAKE_BINARY_DIR}/roboeyes_include")
    file(MAKE_DIRECTORY "${ROBOEYES_INCLUDE_DIR}")
    configure_file(
        "${ROBOEYES_SOURCE_DIR}/src/FluxGarage_RoboEyes.h"
        "${ROBOEYES_INCLUDE_DIR}/FluxGarage_RoboEyes.h"
        COPYONLY
    )
endif()

idf_component_register(SRCS
                        "main.c"
                        "app/wheel_app.c"
                        "bsp/wheel_bsp.c"
                        "communication/stm32_protocol.c"
                        "communication/telemetry_bridge.c"
                        "module/fan/fan_pwm.c"
                        "module/light/light_pwm.c"
                        "module/screen/screen_player.c"
                        "module/screen/screen_roboeyes.cpp"
                        "module/screen/screen_ui.c"
                        "network/web_server.c"
                    INCLUDE_DIRS
                        "."
                        "app"
                        "bsp"
                        "communication"
                        "module/fan"
                        "module/light"
                        "module/screen"
                        "network"
                        "${ROBOEYES_INCLUDE_DIR}"
                    REQUIRES
                        esp_driver_gpio
                        esp_driver_ledc
                        esp_driver_spi
                        esp_driver_uart
                        esp_event
                        esp_http_server
                        esp_hw_support
                        esp_netif
                        esp_timer
                        esp_wifi
                        nvs_flash
                        spiffs)

spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)

set_source_files_properties(
    "module/screen/screen_roboeyes.cpp"
    PROPERTIES COMPILE_OPTIONS "-Wno-error=uninitialized;-Wno-uninitialized"
)
