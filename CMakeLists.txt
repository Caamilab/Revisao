cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)
project(Alarme C CXX ASM)
pico_sdk_init()
add_executable(Alarme Alarme.c inc/ssd1306.c)
pico_generate_pio_header(Alarme ${CMAKE_CURRENT_LIST_DIR}/Alarme.pio)
# Add the standard include files to the build
target_include_directories(Alarme PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(Alarme pico_stdlib hardware_rtc hardware_i2c hardware_adc hardware_pwm hardware_pio)
pico_enable_stdio_usb(Alarme 1)
pico_enable_stdio_uart(Alarme 1)
pico_add_extra_outputs(Alarme)