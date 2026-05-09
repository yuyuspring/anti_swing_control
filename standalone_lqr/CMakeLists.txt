cmake_minimum_required(VERSION 3.14)
project(standalone_lqr VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Standalone LQR simulator executable
add_executable(standalone_lqr
    main.cpp
    standalone_lqr.cpp
)

target_include_directories(standalone_lqr PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
