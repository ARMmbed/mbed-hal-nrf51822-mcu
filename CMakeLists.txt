# mbed Microcontroller Library
# Copyright (c) 2014 ARM Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# this custom build file pulls in different startup assembly based on the
# toolchain being used


if(${CMAKE_C_COMPILER_ID} STREQUAL GNU)
    set(STARTUP_ASM "bootstrap_gcc/startup_nRF51822.S")
    set(STARTUP_SRC "")
elseif(${CMAKE_C_COMPILER_ID} STREQUAL ARMCC)
    set(STARTUP_ASM "bootstrap_armcc/startup_nRF51822.S")
    set(STARTUP_SRC "bootstrap_armcc/sys.cpp")

    # unfortunately, for armlink we have to pass the executable startup code as
    # an object file, not as part of an archive.
    #
    add_library(mbed-hal-nrf51822-startupcod OBJECT ${STARTUP_ASM})

    # This is the "neatest" way of making sure the startup code gets linked
    # into every executable. Needless to say this is, despite its brevity,
    # not neat.
    macro (add_executable _name)
        _add_executable(${ARGV} $<TARGET_OBJECTS:mbed-hal-nrf51822-startupcod>)
    Endmacro()
else()
    set(STARTUP_ASM "")
    set(STARTUP_SRC "")
    message(FATAL_ERROR "Unsupported target toolchain (gcc and armcc are supported)")
endif()


enable_language(ASM)
set_property(SOURCE ${STARTUP_ASM} PROPERTY LANGUAGE ASM)

add_library(mbed-hal-nrf51822-mcu
    ${STARTUP_ASM}
    ${STARTUP_SRC}
    "source/analogin_api.c"
    "source/gpio_irq_api.c"
    "source/pinmap.c"
    "source/pwmout_api.c"
    "source/sleep.c"
    "source/us_ticker.c"
    "source/gpio_api.c"
    "source/i2c_api.c"
    "source/port_api.c"
    "source/serial_api.c"
    "source/spi_api.c"
    "source/lp_ticker.c"
)
set_target_properties(mbed-hal-nrf51822-mcu PROPERTIES
    LINKER_LANGUAGE C
)

# uvisor-lib is only a dependency if we're building for gcc
if(TARGET_LIKE_GCC)
    # set(EXTRA_LINK_LIBS uvisor-lib)
else()
    set(EXTRA_LINK_LIBS)
endif()

target_link_libraries(mbed-hal-nrf51822-mcu
    ${EXTRA_LINK_LIBS}
    mbed-hal
    mbed-hal-nordic
)
