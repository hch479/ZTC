cmake_minimum_required(VERSION 3.16)
project(chassis_control_core C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_library(chassis_core
    common/chassis_can_protocol.c
    firmware/stm32f407/src/chassis_params.c
    firmware/stm32f407/src/wheel_controller.c
    firmware/stm32f407/src/chassis_app.c
)

target_include_directories(chassis_core PUBLIC
    common
    firmware/stm32f407/inc
)

if(NOT MSVC)
    target_compile_options(chassis_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(chassis_core_tests tests/test_main.c)
target_link_libraries(chassis_core_tests PRIVATE chassis_core m)

enable_testing()
add_test(NAME chassis_core_tests COMMAND chassis_core_tests)

