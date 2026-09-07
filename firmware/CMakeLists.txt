cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(boxe_node)

target_sources(app PRIVATE
  src/main.c
  src/punch_detect.c
  src/ble_service.c
  src/adxl375.c
  src/lsm6ds33.c
)
target_include_directories(app PRIVATE src)
