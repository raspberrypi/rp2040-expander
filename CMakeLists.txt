cmake_minimum_required(VERSION 3.13)

set(HOST_RPi "RPi")
set(HOST_RP2040 "RP2040")

if (NOT DEFINED PEXP_HOST)
    set(PEXP_HOST ${HOST_RPi})
    message(STATUS "No 'PEXP_HOST' defined, defaulting to a Pi host build: PEXP_HOST=" ${PEXP_HOST})
endif()

if (${PEXP_HOST} STREQUAL ${HOST_RPi})
    project(pico_expander_demo C)
    set(PORTDIR "port_rpi")
    set(PORTLIBS "gpiod")

elseif (${PEXP_HOST} STREQUAL ${HOST_RP2040})
    set(PORTDIR "port_rp2040")
    set(PORTLIBS "pico_stdlib")
    include(${PORTDIR}/pico_sdk_import.cmake)
    project(pico_expander_demo C CXX ASM)
    pico_sdk_init()

else()
    message(FATAL_ERROR "Erroneous or unsupported host specified")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(APPLICATION "pico_expander_demo")
set(APPLICATION_DIR ${CMAKE_CURRENT_LIST_DIR}/.)

set(pico_expander_demo_sources
    ${APPLICATION_DIR}/host/picoexp.c
    ${APPLICATION_DIR}/host/picoexp_dap.c
    ${APPLICATION_DIR}/${PORTDIR}/picoexp_port.c
    ${APPLICATION_DIR}/demo/picoexp_demo.c
)

add_executable(${APPLICATION} ${pico_expander_demo_sources})

target_include_directories(${APPLICATION} PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/${PORTDIR}/
    ${CMAKE_CURRENT_LIST_DIR}/host/
    ${CMAKE_CURRENT_LIST_DIR}/host/include/
)

target_link_libraries(${APPLICATION} ${PORTLIBS})

if (${PEXP_HOST} STREQUAL ${HOST_RP2040})
    pico_add_extra_outputs(${APPLICATION})
endif()
