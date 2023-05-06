cmake_minimum_required(VERSION 3.20)

set(BOARD nrf52840dk_nrf52840)

# Try to find arm-gcc if toolchain isn't specified.
if (NOT DEFINED ENV{GNUARMEMB_TOOLCHAIN_PATH})
    find_program(ARM_GCC arm-none-eabi-gcc)
    if (${ARM_GCC} STREQUAL ARM_GCC-NOTFOUND)
        message(FATAL_ERROR "Unable to find arm-none-eabi-gcc.")
    else ()
        file(REAL_PATH ${ARM_GCC} REAL_GCC)
        cmake_path(GET REAL_GCC PARENT_PATH GCC_ROOT)
        file(REAL_PATH ${GCC_ROOT}/../ GNUARMEMB_TOOLCHAIN_PATH)
        set(ENV{ZEPHYR_TOOLCHAIN_VARIANT} gnuarmemb)
    endif ()
endif ()

file(REAL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../ WORKSPACE)
set(ENV{ZEPHYR_BASE} ${WORKSPACE}/zephyr)

find_package(Zephyr HINTS $ENV{ZEPHYR_BASE})

set(CMAKE_BUILD_TYPE ZDebug)

project(lt7822)

target_sources(app PRIVATE
        src/main.c
        )
