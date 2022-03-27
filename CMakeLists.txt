cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(led_equalizer C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

pico_sdk_init()

include("PicoLed/PicoLed.cmake")

add_executable(led_equalizer led_equalizer.cpp)
add_library(kiss_fftr kiss_fftr.cpp)
add_library(kiss_fft kiss_fft.cpp)

target_link_libraries(kiss_fftr kiss_fft)

pico_enable_stdio_usb(led_equalizer 1)
pico_enable_stdio_uart(led_equalizer 0)

pico_add_extra_outputs(led_equalizer)

target_link_libraries(led_equalizer
	pico_stdlib
	hardware_adc
	hardware_dma
	kiss_fftr
	PicoLed
	)