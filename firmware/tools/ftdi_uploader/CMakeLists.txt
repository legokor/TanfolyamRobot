cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(ftdi_uploader VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add preprocessor definitions
add_definitions(-DFTD2XX_STATIC)

# Add the executable
add_executable(ftdi_uploader main.cpp)

# Include the header directory
include_directories(libftd2xx/include)

# Link the appropriate library based on the operating system
if(WIN32)
    target_link_libraries(ftdi_uploader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libftd2xx/static/ftd2xx.lib)
else()
    target_link_libraries(ftdi_uploader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/libftd2xx/static/ftd2xx.a)
endif()

# Move the executable to the root of the build directory
set_target_properties(ftdi_uploader PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../bin)


