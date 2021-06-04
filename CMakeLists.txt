cmake_minimum_required(VERSION 3.15.3)

set(CMAKE_TOOLCHAIN_FILE arm-none-eabi-gcc.cmake)
project(minimal-cpp)

enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Debug)

set(STM32CUBEMX_GENERATED_FILES
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c        
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        BSP/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
        BSP/Core/Inc/main.h
        BSP/Core/Inc/stm32f4xx_hal_conf.h
        BSP/Core/Inc/stm32f4xx_it.h
        BSP/Core/Src/main.cpp
        BSP/Core/Src/stm32f4xx_hal_msp.c
        BSP/Core/Src/stm32f4xx_it.c
        BSP/Core/Src/system_stm32f4xx.c
        BSP/Core/Startup/startup_stm32f413zhtx.s)

set(APPLICATION_SPECIFIC_FILES
        app/app.hpp
        app/loop.cpp
        app/setup.cpp
        )

set(EXECUTABLE_FILENAME ${PROJECT_NAME})
set(EXECUTABLE "${EXECUTABLE_FILENAME}.elf")
set(ARTIFACTS_DIR "${CMAKE_CURRENT_LIST_DIR}/output")

add_executable(${EXECUTABLE} ${STM32CUBEMX_GENERATED_FILES} ${APPLICATION_SPECIFIC_FILES})

set_target_properties( ${EXECUTABLE}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${ARTIFACTS_DIR}
    LIBRARY_OUTPUT_DIRECTORY ${ARTIFACTS_DIR}
    RUNTIME_OUTPUT_DIRECTORY ${ARTIFACTS_DIR}
)

target_compile_definitions(${EXECUTABLE} PRIVATE
        -DUSE_HAL_DRIVER
        -DSTM32F413xx
        )

target_include_directories(${EXECUTABLE} PRIVATE
        BSP/Core/Inc
        BSP/Drivers/STM32F4xx_HAL_Driver/Inc
        BSP/Drivers/CMSIS/Device/ST/STM32F4xx/Include
        BSP/Drivers/CMSIS/Include
        app/
        )

target_compile_options(${EXECUTABLE} PRIVATE
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -fdata-sections
        -ffunction-sections
        -fno-exceptions
        $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
        -Wall
        $<$<CONFIG:Debug>:-Og>
        )

target_link_options(${EXECUTABLE} PRIVATE
        -T${CMAKE_SOURCE_DIR}/BSP/STM32F413ZHTx_FLASH.ld
        -mcpu=cortex-m4
        -mthumb
        -mfpu=fpv4-sp-d16
        -mfloat-abi=hard
        -specs=nano.specs
        -lc
        -lm
        -lnosys
        -fno-exceptions
        -fno-rtti
        -Wl,-Map=${PROJECT_NAME}.map,--cref
        -Wl,--gc-sections
        )

# Print executable size
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-size "${ARTIFACTS_DIR}/${EXECUTABLE}")

# Create hex file
add_custom_command(TARGET ${EXECUTABLE}
        POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex "${ARTIFACTS_DIR}/${EXECUTABLE}" "${ARTIFACTS_DIR}/${EXECUTABLE_FILENAME}.hex"
        COMMAND arm-none-eabi-objcopy -O binary "${ARTIFACTS_DIR}/${EXECUTABLE}" "${ARTIFACTS_DIR}/${EXECUTABLE_FILENAME}.bin")

