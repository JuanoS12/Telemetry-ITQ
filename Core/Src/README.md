# STM32F4 Telemetry Node

This folder contains the application sources for an STM32F407-based telemetry node.
The firmware is generated from STM32CubeMX and extended with FreeRTOS tasks that
read an MPU6050 IMU and a GPS receiver, and publish the resulting telemetry over
CAN and over a UART link to a host computer.

## Key modules

- `freertos.c` – FreeRTOS object creation and task logic (sensor acquisition,
  GPS parsing, CAN/UART transmission).
- `usart.c` – USART1 RX (GPS via DMA Receive-to-Idle) and USART2 TX (PC telemetry).
- `can.c` – CAN1 peripheral setup plus helper to transmit standard frames.
- `mpu6050.c` – Minimal MPU6050 driver to obtain linear acceleration, angular rate
  and die temperature.
- `main.c` – HAL initialisation sequence and scheduler start.

## Data flow summary

1. **SensorTask** (100 Hz)
   - Reads MPU6050 via I²C1.
   - Sends two CAN frames (`0x100`, `0x101`).
   - Enqueues a CSV line on the UART stream buffer.
2. **GPSTask**
   - Collects NMEA sentences from USART1 DMA into a stream buffer.
   - Parses RMC/GGA sentences and, when a fix is valid, publishes CAN frames
     (`0x120`, `0x121`) and a CSV line.
3. **CanTxTask** drains the CAN queue and transmits frames with retry on transient
   errors.
4. **UartTxTask** pulls ASCII chunks from the UART stream buffer and writes them to
   the PC via USART2 (115200-8N1).

## Building

Import the `Core/` folder into STM32CubeIDE (or regenerate a project with the same
peripheral configuration) and build/flash with an ST-LINK probe. FreeRTOS heap and
stack sizes assume the default CubeMX configuration.
