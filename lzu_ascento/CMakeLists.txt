cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME lzu_ascento)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)


#DSP
add_definitions(-DARM_MATH_CM7 -DARM_MATH_MATRIX_CHECK -DARM_MATH_ROUNDING)
include_directories(Core/Inc)
link_directories(Drivers/CMSIS/DSP/Lib/GCC)
link_libraries(libarm_cortexM7lfdp_math.a)



# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/bsp/can_bsp.c
        Core/Inc/can_bsp.h
        Core/Src/bsp/bmi088driver.c
        Core/Inc/bmi088driver.h
        Core/Src/bsp/bmi088midware.c
        Core/Inc/bmi088midware.h
        Core/Src/bsp_dwt.c
        Core/Inc/bsp_dwt.h
        Core/Inc/bmi088reg.h
        Core/Src/bsp/sfdmctrl.c
        Core/Inc/sfdmctrl.h
        Core/Inc/arm_math.h
        Core/Src/bsp/tjc_usart_hmi.c
        Core/Inc/tjc_usart_hmi.h
        Core/Src/task/INS_task.c
        Core/Src/task/chassis_task.c
        Core/Inc/chassis_task.h
        Core/Inc/INS_task.h
        Core/Src/bsp/elrs.c
        Core/Inc/elrs.h
        Core/Src/Filtering_algorithm/mahony_filter.c
        Core/Inc/mahony_filter.h
        Core/Src/Filtering_algorithm/QuaternionEKF.c
        Core/Inc/QuaternionEKF.h
        Core/Src/Filtering_algorithm/kalman_filter.c
        Core/Inc/kalman_filter.h
        Core/Src/bsp/rc_ctrl.c
        Core/Inc/rc_ctrl.h
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float

)