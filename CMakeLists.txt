cmake_minimum_required(VERSION 3.11)

project(trillingsmeter)

set(CMAKE_CXX_STANDARD 20)

# add_subdirectory("./lib")

add_executable(trillingsmeter
    "main.cpp"
) 

# target_link_libraries(trillingsmeter ads1258)