cmake_minimum_required(VERSION 3.14)

set(APP_NAME "rpi-pico-imu")
set(APP_VERSION "1.0.0")
set(BUILD_NUM 1)
add_compile_definitions(APP_NAME="${APP_NAME}")
add_compile_definitions(APP_VERSION="${APP_VERSION}")
add_compile_definitions(BUILD_NUM=${BUILD_NUM})


# Include app source code file(s)
add_executable(${APP_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/main.c
)

# Link to built libraries
target_link_libraries(${APP_NAME} LINK_PUBLIC
    pico_stdlib
    hardware_i2c
    FreeRTOS
    bno055)

# Enable/disable STDIO via USB and UART
pico_enable_stdio_usb(${APP_NAME} ENABLED)
pico_enable_stdio_uart(${APP_NAME} 0)

# Enable extra build products
pico_add_extra_outputs(${APP_NAME})
