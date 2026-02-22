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
#set(CMAKE_MAKE_PROGRAM "${CMAKE_CURRENT_SOURCE_DIR}/Tools/build_tools/ninja.exe")


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME tanfolyamrobot)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths

)

set(DRIVER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ColorSensor/src/clr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Drive/src/drv_motor.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Drive/src/drv_speedControl.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Encoder/src/enc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Infrared/src/ir.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Lcd/src/lcd_circularBuffer.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Lcd/src/lcd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Lcd/src/lcd_statusIndicator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Mpu9250/src/mpu.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Mpu9250/src/mpu_basicOrientation.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Pwm/src/pwm.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Servo/src/srv.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/TextUart/src/txt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/Ultrasonic/src/us.c
)

set(MAIN_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Main/src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Main/src/main_interruptHandler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Main/src/main_usDelay.c
    ${CMAKE_CURRENT_SOURCE_DIR}/RobotApi/src/api.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${DRIVER_SOURCES}
    ${MAIN_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/src/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Telemetry/src/tel.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    IR_SENSOR
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Convert to bin file
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE}/${CMAKE_PROJECT_NAME}.bin
)
