# Core Firmware Overview

This document summarizes the key functions implemented under `Core/` and highlights the conditions that can interrupt the normal runtime loop. It also references the PlantUML flow diagram in `core_flow.pu`.

## Main entry points

### `main` (`Core/Src/main.c`)
* Initializes HAL, system clock, and peripheral drivers (GPIO, DMA, I2C, USART, CAN).【F:Core/Src/main.c†L72-L102】
* Starts project-specific peripherals (`can_start`, FreeRTOS setup) and launches the RTOS scheduler.【F:Core/Src/main.c†L103-L127】

### `SystemClock_Config`
* Configures oscillators and bus clocks; failure triggers `Error_Handler`.【F:Core/Src/main.c†L133-L192】

### `Error_Handler`
* Disables interrupts and enters an infinite loop when critical initialization fails.【F:Core/Src/main.c†L199-L209】

## FreeRTOS initialization (`Core/Src/freertos.c`)

### Kernel objects
* `Q_CAN_TX` queue holds CAN frames for the transmit task.【F:Core/Src/freertos.c†L92-L101】【F:Core/Src/freertos.c†L141-L149】
* `SB_GPS` stream buffer shares GPS bytes between the USART ISR and the GPS task.【F:Core/Src/freertos.c†L92-L101】

### Tasks
* **SensorTask** – Initializes the MPU6050 IMU, samples at 100 Hz, and enqueues CAN frames with accelerometer and gyroscope data.【F:Core/Src/freertos.c†L167-L213】
* **GPSTask** – Reassembles NMEA sentences from `SB_GPS`, updates fix data, and enqueues latitude/longitude and velocity messages.【F:Core/Src/freertos.c†L226-L262】
* **CanTxTask** – Blocks on `Q_CAN_TX`, transmitting each frame via `CAN1_SendStd`, retrying on failure.【F:Core/Src/freertos.c†L273-L288】
* **LedTask** – Placeholder loop with a 1 ms delay for status LED work.【F:Core/Src/freertos.c†L298-L307】

During RTOS setup `GPS_Start()` is called so that DMA-driven UART reception can fill `SB_GPS`.【F:Core/Src/freertos.c†L151-L159】

## Peripheral support modules

### CAN (`Core/Src/can.c`)
* `MX_CAN1_Init` configures bit timing, filters, starts the controller, and enables interrupts; `CAN_ErrorHandler` is a stub for future recovery logic.【F:Core/Src/can.c†L56-L113】

### UART/GPS (`Core/Src/usart.c`)
* `MX_USART1_UART_Init` configures UART1 and DMA reception.【F:Core/Src/usart.c†L57-L138】
* `GPS_Start` arms DMA “receive-to-idle” to feed the GPS buffer and asserts that `SB_GPS` exists.【F:Core/Src/usart.c†L205-L223】
* `HAL_UARTEx_RxEventCallback` pushes received bytes into `SB_GPS`, counts overruns, re-arms DMA, and yields to higher-priority tasks if needed.【F:Core/Src/usart.c†L225-L240】
* `HAL_UART_ErrorCallback` also re-arms DMA after an error.【F:Core/Src/usart.c†L242-L248】

### IMU (`Core/Src/mpu6050.c`)
* `MPU6050_Init_200Hz` wakes the sensor over I²C.【F:Core/Src/mpu6050.c†L11-L23】
* `MPU6050_Read` reads accelerometer, gyroscope, and temperature data, converts to physical units, and timestamps the sample.【F:Core/Src/mpu6050.c†L25-L45】

## Conditions that interrupt or degrade the routine

* **Initialization failure** – Any HAL init returning `HAL_ERROR` triggers `Error_Handler`, halting the system.【F:Core/Src/main.c†L133-L209】【F:Core/Src/freertos.c†L169-L174】
* **RTOS object creation failure** – `configASSERT` stops execution if queue or stream buffer allocation fails.【F:Core/Src/freertos.c†L141-L149】
* **CAN queue saturation** – `CAN_SendMessage` increments drop counters when `Q_CAN_TX` is full, causing telemetry loss until space frees.【F:Core/Src/freertos.c†L68-L90】【F:Core/Src/freertos.c†L187-L212】
* **CAN driver errors** – `CAN1_SendStd` retry loop blocks the transmit task until the driver succeeds; severe faults should be handled via `CAN_ErrorHandler`.【F:Core/Src/freertos.c†L273-L288】【F:Core/Src/can.c†L27-L41】
* **GPS buffer overruns** – If `SB_GPS` lacks space, bytes are dropped and counted by `gps_stream_overruns_local`, reducing GPS fidelity.【F:Core/Src/usart.c†L214-L240】
* **UART DMA errors** – HAL callbacks attempt automatic recovery by re-arming reception; persistent errors require user-defined handling.【F:Core/Src/usart.c†L225-L248】
* **Assert failures** – `ASSERT`/`configASSERT` macros halt or reset the system, depending on FreeRTOS configuration.【F:Core/Src/main.c†L50-L57】【F:Core/Src/freertos.c†L141-L149】

## Flow diagram

See `core_flow.pu` for a PlantUML diagram of the runtime flow from startup through the RTOS tasks and data paths.

