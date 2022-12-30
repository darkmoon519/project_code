
cmake_minimum_required(VERSION 3.13)

project(magic_wand_ble C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

set(FAMILY rp2040)
set(BOARD pico_sdk)
set(TINYUSB_FAMILY_PROJECT_NAME_PREFIX "tinyusb_dev_")

add_executable(magic_wand_ble "")

target_include_directories(magic_wand_ble
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/.
  ${CMAKE_CURRENT_LIST_DIR}/tusb_hid/.
  )

set_target_properties(
  magic_wand_ble
  PROPERTIES
  COMPILE_FLAGS -fno-rtti
  COMPILE_FLAGS -fno-exceptions
  COMPILE_FLAGS -fno-threadsafe-statics
  COMPILE_FLAGS -nostdlib
)

# pico_generate_pio_header(magic_wand_ble ${CMAKE_CURRENT_LIST_DIR}/pio_i2c/i2c.pio)

target_sources(magic_wand_ble
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/imu_provider.h
  ${CMAKE_CURRENT_LIST_DIR}/magic_wand_model_data.cpp
  ${CMAKE_CURRENT_LIST_DIR}/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/main_functions.cpp
  ${CMAKE_CURRENT_LIST_DIR}/main_functions.h
  ${CMAKE_CURRENT_LIST_DIR}/magic_wand_model_data.h
  ${CMAKE_CURRENT_LIST_DIR}/rasterize_stroke.cpp
  ${CMAKE_CURRENT_LIST_DIR}/rasterize_stroke.h
  ${CMAKE_CURRENT_LIST_DIR}/micro_features_data.cpp
  ${CMAKE_CURRENT_LIST_DIR}/micro_features_data.h
  ${CMAKE_CURRENT_LIST_DIR}/kalman_filter.h
  ${CMAKE_CURRENT_LIST_DIR}/kalman_filter_MPU6050.cpp
  ${CMAKE_CURRENT_LIST_DIR}/tusb_hid/hid_src.h              # hid
  ${CMAKE_CURRENT_LIST_DIR}/tusb_hid/hid_src.cpp            # hid
  ${CMAKE_CURRENT_LIST_DIR}/tusb_hid/tusb_config.h          # hid
  ${CMAKE_CURRENT_LIST_DIR}/tusb_hid/usb_descriptors.h      # hid
  ${CMAKE_CURRENT_LIST_DIR}/tusb_hid/usb_descriptors.cpp    # hid

  # ${CMAKE_CURRENT_LIST_DIR}/pio_i2c/pio_i2c.c               # PIO_I2C
  # ${CMAKE_CURRENT_LIST_DIR}/pio_i2c/pio_i2c.h               # PIO_I2C

  )

target_link_libraries(
  magic_wand_ble
  pico-tflmicro
  hardware_pwm
  pico_stdlib
  ICM42622
  ICM20948
  LCD_st7735
  tinyusb_device
  tinyusb_board
  # hardware_pio
)

# enable usb output, disable uart output
pico_enable_stdio_usb(magic_wand_ble 1)
pico_enable_stdio_uart(magic_wand_ble 0)

pico_add_extra_outputs(magic_wand_ble)
