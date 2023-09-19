cmake_minimum_required(VERSION 3.10)
project(ads1258)

find_library(GPIODCXX_LIBRARY NAMES libgpiodcxx.so)
if(NOT GPIODCXX_LIBRARY)
  message(FATAL_ERROR "gpiod library not found. Install apt install libgpiod-dev")
endif()


# Create a static library
add_library(ads1258 "ADS1258.cpp")

# Specify where the header files are
target_include_directories(ads1258 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)

target_link_libraries(ads1258 ${GPIODCXX_LIBRARY})