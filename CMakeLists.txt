cmake_minimum_required(VERSION 3.22)

project(OmniLayer C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)

set(MCU_TARGET "G3507" CACHE STRING "Target MCU family: G3507")
set_property(CACHE MCU_TARGET PROPERTY STRINGS G3507)

if(NOT MCU_TARGET STREQUAL "G3507")
    message(FATAL_ERROR "This repository is now G3507-only. Please use -DMCU_TARGET=G3507.")
endif()

set(RESOLVED_MCU_TARGET "G3507")
set(ENROLL_MCU_TARGET_VALUE 2)

message(STATUS "Resolved MCU target: ${RESOLVED_MCU_TARGET}")

set(APP_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/A_Entry/main.c
    ${CMAKE_SOURCE_DIR}/app/Control/Control.c
    ${CMAKE_SOURCE_DIR}/app/My_Usart/My_Usart.c
    ${CMAKE_SOURCE_DIR}/app/Control_Task/Control_Task.c
    ${CMAKE_SOURCE_DIR}/app/PID/PID.c
    ${CMAKE_SOURCE_DIR}/app/Filter/Filter.c
)

set(SYS_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/SYSTEM/sys.c
)

set(API_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/API/src/adc.c
    ${CMAKE_SOURCE_DIR}/API/src/exti.c
    ${CMAKE_SOURCE_DIR}/API/src/gpio.c
    ${CMAKE_SOURCE_DIR}/API/src/tim.c
    ${CMAKE_SOURCE_DIR}/API/src/usart.c
    ${CMAKE_SOURCE_DIR}/API/src/pwm.c
    ${CMAKE_SOURCE_DIR}/API/src/Encoder.c
    ${CMAKE_SOURCE_DIR}/API/API_I2C/API_I2C.c
    ${CMAKE_SOURCE_DIR}/API/API_SPI/API_SPI.c
)

set(ENROLL_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/Enroll/Enroll.c
)

set(COMMON_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/A_Entry
    ${CMAKE_SOURCE_DIR}/app
    ${CMAKE_SOURCE_DIR}/app/Control_Task
    ${CMAKE_SOURCE_DIR}/app/Control
    ${CMAKE_SOURCE_DIR}/app/My_Usart
    ${CMAKE_SOURCE_DIR}/app/PID
    ${CMAKE_SOURCE_DIR}/app/Filter
    ${CMAKE_SOURCE_DIR}/API
    ${CMAKE_SOURCE_DIR}/API/inc
    ${CMAKE_SOURCE_DIR}/API/API_I2C
    ${CMAKE_SOURCE_DIR}/API/API_SPI
    ${CMAKE_SOURCE_DIR}/Enroll
    ${CMAKE_SOURCE_DIR}/SYSTEM
)

set(BSP_COMMON_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/BSP/LED
    ${CMAKE_SOURCE_DIR}/BSP/KEY
    ${CMAKE_SOURCE_DIR}/BSP/OLED
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050/eMPL
    ${CMAKE_SOURCE_DIR}/BSP/TB6612
    ${CMAKE_SOURCE_DIR}/BSP/gray_adc
    ${CMAKE_SOURCE_DIR}/BSP/JY61P
)

set(BSP_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/BSP/LED/LED.c
    ${CMAKE_SOURCE_DIR}/BSP/KEY/KEY.c
    ${CMAKE_SOURCE_DIR}/BSP/OLED/OLED.c
    ${CMAKE_SOURCE_DIR}/BSP/OLED/OLED_Data.c
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050/MPU6050.c
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050/MPU6050_Int.c
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050/eMPL/inv_mpu.c
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050/eMPL/inv_mpu_dmp_motion_driver.c
    ${CMAKE_SOURCE_DIR}/BSP/TB6612/TB6612.c
    ${CMAKE_SOURCE_DIR}/BSP/gray_adc/gray_adc.c
    ${CMAKE_SOURCE_DIR}/BSP/JY61P/jy61p.c
)

set(TARGET_NAME OmniLayer_MSPM0G3507)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/cmake/mspm0g3507.lds)
set(OPENOCD_CFG ${CMAKE_SOURCE_DIR}/OpenOCD/G3507_OpenOCD.cfg)
set(OPENOCD_ERASE_CMD "mspm0 mass_erase 0")
set(MCU_CPU_FLAGS -mcpu=cortex-m0plus -mthumb)
set(MCU_FPU_FLAGS)
set(MCU_DEFINITIONS __MSPM0G3507__)

set(MCU_INCLUDE_DIRS
    ${COMMON_INCLUDE_DIRS}
    ${BSP_COMMON_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/inc
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/G3507_soft_i2c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/G3507_soft_spi
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_M0G3507/source
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_M0G3507/source/ti/devices/msp/m0p
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_M0G3507/source/third_party/CMSIS/Core/Include
)

set(MCU_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/G3507_delay.c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/G3507_sys.c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/G3507_soft_i2c/G3507_soft_i2c.c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/G3507_soft_spi/G3507_soft_spi.c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/src/G3507_adc.c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/src/G3507_exti.c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/src/G3507_gpio.c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/src/G3507_pwm.c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/src/G3507_tim.c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/src/G3507_usart.c
    ${CMAKE_SOURCE_DIR}/Core/MSPM0G3507/src/G3507_Encoder.c
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_M0G3507/source/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_M0G3507/source/ti/driverlib/dl_adc12.c
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_M0G3507/source/ti/driverlib/m0p/sysctl/dl_sysctl_mspm0g1x0x_g3x0x.c
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_M0G3507/source/ti/driverlib/dl_timer.c
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_M0G3507/source/ti/driverlib/dl_uart.c
)

set(TARGET_SOURCES
    ${APP_LAYER_SOURCES}
    ${SYS_LAYER_SOURCES}
    ${API_LAYER_SOURCES}
    ${ENROLL_LAYER_SOURCES}
    ${BSP_LAYER_SOURCES}
    ${MCU_SOURCES}
)

add_executable(${TARGET_NAME} ${TARGET_SOURCES})

target_include_directories(${TARGET_NAME} PRIVATE
    ${MCU_INCLUDE_DIRS}
)

target_compile_definitions(${TARGET_NAME} PRIVATE
    ENROLL_MCU_G3507=2
    ENROLL_MCU_TARGET=${ENROLL_MCU_TARGET_VALUE}
    ${MCU_DEFINITIONS}
)

target_compile_options(${TARGET_NAME} PRIVATE
    ${MCU_CPU_FLAGS}
    ${MCU_FPU_FLAGS}
    -Wall
    -Wextra
)

target_link_options(${TARGET_NAME} PRIVATE
    ${MCU_CPU_FLAGS}
    ${MCU_FPU_FLAGS}
    -T${LINKER_SCRIPT}
    --specs=nosys.specs
    --specs=nano.specs
    -u
    _printf_float
    -Wl,--gc-sections
    -Wl,-Map=${TARGET_NAME}.map
    -Wl,--print-memory-usage
)

target_link_libraries(${TARGET_NAME} PRIVATE
    m
    c
)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
    COMMENT "Generate .hex/.bin and print size"
)

add_custom_target(flash
    COMMAND openocd -f ${OPENOCD_CFG} -c "program $<TARGET_FILE:${TARGET_NAME}> reset exit"
    DEPENDS ${TARGET_NAME}
    COMMENT "Flash firmware via OpenOCD"
)

add_custom_target(erase
    COMMAND openocd -f ${OPENOCD_CFG} -c "init; reset halt; ${OPENOCD_ERASE_CMD}; reset run; exit"
    COMMENT "Mass erase firmware via OpenOCD"
)
