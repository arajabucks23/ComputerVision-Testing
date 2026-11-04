cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/arm-none-eabi.cmake)
project(evac_cam C ASM)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_C_FLAGS_RELEASE "-O2 -g")
set(CMAKE_C_STANDARD 11)

# ST's STM32CubeF7 package (HAL, CMSIS, lwIP, LAN8742 driver), fetched from GitHub.
set(ST ${CMAKE_CURRENT_SOURCE_DIR}/st_cubef7)
set(HAL ${ST}/Drivers/STM32F7xx_HAL_Driver)
set(LWIP_DIR ${ST}/Middlewares/Third_Party/LwIP)
include(${LWIP_DIR}/src/Filelists.cmake)

set(HAL_MODULES hal hal_cortex hal_rcc hal_rcc_ex hal_gpio hal_dma hal_dma_ex hal_pwr hal_pwr_ex
                hal_flash hal_flash_ex hal_uart hal_uart_ex hal_i2c hal_i2c_ex hal_eth hal_dcmi hal_dcmi_ex)
list(TRANSFORM HAL_MODULES PREPEND ${HAL}/Src/stm32f7xx_)
list(TRANSFORM HAL_MODULES APPEND .c)

add_executable(evac_cam
  Core/Src/main.c
  Core/Src/ethernetif.c
  Core/Src/stm32f7xx_it.c
  Core/Src/syscalls.c
  Core/Src/camera.c
  Core/Src/dht11.c
  Core/Src/wiring_check.c
  Core/Src/log.c
  Core/Src/http_server.c
  Core/Src/capture.c
  Core/Src/adc.c
  ${ST}/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/gcc/startup_stm32f746xx.s
  ${ST}/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c
  ${HAL_MODULES}
  ${ST}/Drivers/BSP/Components/lan8742/lan8742.c
  st_ov5640/ov5640.c
  st_ov5640/ov5640_reg.c
  ${lwipcore_SRCS}
  ${lwipcore4_SRCS}
  ${LWIP_DIR}/src/netif/ethernet.c
)

target_compile_definitions(evac_cam PRIVATE STM32F746xx USE_HAL_DRIVER)
target_include_directories(evac_cam PRIVATE
  Core/Inc
  ${HAL}/Inc
  ${ST}/Drivers/CMSIS/Device/ST/STM32F7xx/Include
  ${ST}/Drivers/CMSIS/Include
  ${ST}/Drivers/BSP/Components/lan8742
  st_ov5640
  ${LWIP_DIR}/src/include
  ${LWIP_DIR}/system
)

target_link_libraries(evac_cam PRIVATE m)   # powf (CO ppm estimate)
target_link_options(evac_cam PRIVATE
  -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F746ZG_FLASH.ld
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/evac_cam.map
)

# evac_cam.bin for flashing, plus a size summary.
add_custom_command(TARGET evac_cam POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:evac_cam> ${CMAKE_CURRENT_BINARY_DIR}/evac_cam.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:evac_cam>
)
