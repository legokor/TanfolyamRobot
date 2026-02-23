set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# If the $ENV{GCC_TOOLCHAIN_ROOT} environment variable is set, use it as the root for the toolchain, otherwise assume the tools are in the PATH
if(DEFINED ENV{GCC_TOOLCHAIN_ROOT})
    set(TOOLCHAIN_ROOT_TMP $ENV{GCC_TOOLCHAIN_ROOT})
    message("GCC_TOOLCHAIN_ROOT set to ${TOOLCHAIN_ROOT_TMP}")
    cmake_path(CONVERT "${TOOLCHAIN_ROOT_TMP}" TO_CMAKE_PATH_LIST TOOLCHAIN_ROOT)
    set(TOOLCHAIN_ROOT "${TOOLCHAIN_ROOT}/")
    message("LINUX_PATH: ${TOOLCHAIN_ROOT}")
else()
    set(TOOLCHAIN_ROOT "")
    message("GCC_TOOLCHAIN_ROOT is not set, assuming gcc is in path")
endif()

# If on windows, the file extension for the toolchain binaries is .exe, otherwise it is empty
if(WIN32)
    set(TOOLCHAIN_BIN_EXTENSION ".exe")
else()
    set(TOOLCHAIN_BIN_EXTENSION "")
endif()

# Some default GCC settings
set(TOOLCHAIN_PREFIX                ${TOOLCHAIN_ROOT}arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_BIN_EXTENSION})
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_BIN_EXTENSION})
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_BIN_EXTENSION})
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_BIN_EXTENSION})
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size${TOOLCHAIN_BIN_EXTENSION})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m3 ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F103XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
# Add scanf support for floating point numbers. If printf support is needed, add -u _printf_float
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _scanf_float")
set(TOOLCHAIN_LINK_LIBRARIES "m")
