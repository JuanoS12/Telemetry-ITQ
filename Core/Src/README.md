# STM32F4 IMU Project

This project is for an STM32F4 microcontroller, implementing an Inertial Measurement Unit (IMU) application. It initializes and uses peripherals such as I2C, USART, CAN, DMA, and GPIO to communicate with sensors and other devices.

## Project Structure
- `main.c`: Entry point, system and peripheral initialization, main loop.
- `i2c.c`, `usart.c`, `can.c`, `dma.c`, `gpio.c`: Peripheral drivers.
- `stm32f4xx_it.c`: Interrupt handlers.
- `syscalls.c`, `sysmem.c`, `system_stm32f4xx.c`: System support files.

## How to Build & Flash
Use STM32CubeIDE or VS Code with appropriate tasks to build and flash the firmware.

## Configuration
Hardware parameters are defined in `config.h`.

## Improvements
- Modular code structure
- Error handling stubs
- Documentation and comments
- Configuration header
- Consistent code style
