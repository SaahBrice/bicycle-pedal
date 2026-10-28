cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(bicycle_pedal)

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
  src/main.c
  src/configuration/configuration.c
  src/algorithms/standard.c
  src/algorithms/simplified.c
  src/revolution/revolution.c
  src/sensors/lookup_source.c
  src/lookup/samples.c
  src/activity/activity.c
  src/activity/button/button.c
  src/activity/gpio/gpio.c
)

if(NOT CONFIG_PEDAL_POWER_BUILD)
  target_sources(app PRIVATE src/communication/uart.c src/timing/timing.c)
endif()
