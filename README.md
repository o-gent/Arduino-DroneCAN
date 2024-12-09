# Arduino DroneCAN

This repository allows easy integration of sensors to be used with Ardupilot and PX4 via DroneCAN. Sensors can be integrated within minutes by using pre-existing Arduino libraries for sensors, and through this library which abstracts the UAVCAN layer so you can focus on sending and receiving messages.

By using the Arduino core and PlatformIO with pre-configured board setups, you can start developing instantly.

## Currently Supported Hardware
One of the sticking points is writing the CAN driver, as for STM32 there is not an Arduino compatible interface. We plan to get around this by supporting the MCP2515 which has well supported Arduino drivers.

- Beyond Robotics CAN Node - STM32 L431CCU6 >> plug and play

Planned hardware:
- Common MCUs such as RP2040, ESP32, combined with the MCP2515
- STM32 H7, a FDCAN driver needs writing



## Standing on the shoulders of Giants.

This repo is built on many resources:
- Arduino Core https://github.com/stm32duino/Arduino_Core_STM32
- Basis of the CAN API https://github.com/nopnop2002/Arduino-STM32-CAN/blob/master/stm32l452/stm32l452.ino#L55
- How to use libcanard https://github.com/dronecan/libcanard/blob/master/examples/ServoNode/servo_node.c
- https://github.com/seeers/CAN-Bus-Arduino_Core_STM32/blob/master/CanLowlevel.ino
- https://github.com/geosmall/UAVCAN-for-STM32-Arduino/blob/master/libcanard_example/src/libcanard/drivers/stm32/canard_stm32.c
- the platformio board is based on this being the closest supported board https://docs.platformio.org/en/latest/boards/ststm32/nucleo_l432kc.html
- These are the HAL drivers for the specific MCU being used, although they're all generic https://github.com/stm32duino/Arduino_Core_STM32/tree/main/variants/STM32L4xx/L431C(B-C)(T-U)


Other stuff which didn't get used but interesting 
- https://github.com/UBC-Solar/firmware-v2/tree/master
- https://github.com/geosmall/UAVCAN-for-STM32-Arduino/blob/master/libcanard_example/src/libcanard/drivers/stm32/canard_stm32.c
- https://github.com/pazi88/8Ch-EGT
- https://github.com/J-f-Jensen/libraries/blob/master/STM32CAN/STM32CAN.cpp
- https://github.com/Dmivaka/STM32-HAL-Libcanard/tree/main


Other notes
- No CAN API support in STM32 Arduino https://github.com/stm32duino/Arduino_Core_STM32/issues/259
- The HAL drivers are generated for L431 but not working https://community.st.com/t5/stm32cubemx-mcus/the-stm32cubeide-does-not-generate-the-stm32h7xx-hal-driver-file/td-p/101970


## Setup

The L431CCU6.json needs to be placed here: {$USER}\.platformio\platforms\ststm32\boards
The L431C(B-C)(T-U) folder needs placing in {$USER}\.platformio\packages\framework-arduinoststm32\variants\STM32L4xx


