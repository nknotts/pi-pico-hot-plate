# What CMake to start at
cmake_minimum_required(VERSION 3.12)
 
set(PICO_BOARD pico_w)
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/FreeRTOS-Kernel)

include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)
 
project(HotPlate VERSION 1.0.0)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
add_definitions(-DMQTT_OUTPUT_RINGBUF_SIZE=1200)
add_definitions(-DMQTT_REQ_MAX_IN_FLIGHT=60)
 
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(AddVersionHeader)
AddVersionHeader(hot-plate-version HotPlate)

add_subdirectory(thirdparty/fmt)
add_subdirectory(thirdparty/pico-ssd1306)

file(GLOB_RECURSE pico_sources CONFIGURE_DEPENDS
    src/*.cpp src/*.c thirdparty/BME68x-Sensor-API/bme68x.c)
file(GLOB_RECURSE pico_headers CONFIGURE_DEPENDS src/*.hpp src/*.h)

add_executable(HotPlate ${pico_sources} ${pico_headers})
target_include_directories(HotPlate PUBLIC src)
target_link_libraries(HotPlate PRIVATE 
    pico_stdlib
    hardware_i2c
    hardware_pwm
    hardware_spi
    hardware_rtc
    pico_ssd1306
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_mqtt
    pico_lwip_sntp
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    tinyusb_device_unmarked
    hot-plate-version
    fmt)

pico_sdk_init()
pico_add_extra_outputs(HotPlate)
