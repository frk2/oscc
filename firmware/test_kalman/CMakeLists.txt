project(test-kalman)

set(ARDUINO_DEFAULT_BOARD mega2560)
SET(ARDUINO_DEFAULT_PORT ${SERIAL_PORT_CAN_GATEWAY})
set(ARDUINO_DEFAULT_BAUDRATE ${SERIAL_BAUD_CAN_GATEWAY})

add_definitions(-DSERIAL_BAUD=${ARDUINO_DEFAULT_BAUDRATE})

generate_arduino_firmware(
        test-kalman
        SRCS
        ${CMAKE_SOURCE_DIR}/common/libs/arduino_init/arduino_init.cpp
        ${CMAKE_SOURCE_DIR}/common/libs/timer/oscc_timer.cpp
        ${CMAKE_SOURCE_DIR}/common/libs/serial/oscc_serial.cpp
        ${CMAKE_SOURCE_DIR}/common/libs/pid/oscc_pid.cpp
        src/main.cpp
        src/init.cpp
        src/timer.cpp)

target_include_directories(
        test-kalman
        PRIVATE
        include
        ${CMAKE_SOURCE_DIR}/common/include
        ${CMAKE_SOURCE_DIR}/common/libs/arduino_init
        ${CMAKE_SOURCE_DIR}/common/libs/serial
        ${CMAKE_SOURCE_DIR}/common/libs/timer
        ${CMAKE_SOURCE_DIR}/common/libs/pid
        ${CMAKE_SOURCE_DIR}/common/libs/BasicLinearAlgebra
        ${CMAKE_SOURCE_DIR}/../api/include)
        
