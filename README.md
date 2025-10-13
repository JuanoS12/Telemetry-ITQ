# Telemetry-ITQ Firmware

Firmware for an STM32F407-based telemetry node that acquires motion and navigation
information and streams it to the vehicle CAN bus and to a host computer over UART.
The codebase is organised as a CubeMX-generated FreeRTOS application with the
custom logic living under the `Core/` directory.

## Features

- **MPU6050 IMU acquisition** over I²C at 100 Hz with scaling to physical units.
- **GPS reception** using USART1 with DMA Receive-to-Idle, NMEA parsing for RMC/GGA.
- **CAN transmission** of IMU and GPS frames (standard IDs 0x100/0x101/0x120/0x121).
- **UART streaming to PC** (USART2 @ 115200 bps) emitting CSV telemetry for logging.
- **FreeRTOS task architecture** with bounded queues/stream buffers for deterministic flow.

## Repository layout

```
Core/
  Inc/        # Public headers (mpu6050.h, peripheral handles, FreeRTOS config)
  Src/        # Application sources (freertos.c, main.c, can.c, usart.c, ...)
  Startup/    # Startup files for STM32F407VETx
  core_analysis.md  # High-level documentation of the firmware
  core_flow.pu      # PlantUML activity diagram of runtime flow
```

## Runtime architecture

### FreeRTOS scheduling model

The firmware runs with a 1 kHz RTOS tick (`configTICK_RATE_HZ = 1000`). Each task
has a defined execution cadence, priority, and stack budget so the system can be
analysed for timing closure:

| Task          | Period / Trigger                               | Priority (CMSIS)          | Stack (bytes) | Responsibilities |
|---------------|------------------------------------------------|---------------------------|---------------|------------------|
| `SensorTask`  | 10 ms periodic (`vTaskDelayUntil`)              | `osPriorityAboveNormal1`  | 2048          | Polls MPU6050 at 100 Hz, fills two CAN frames (`0x100`/`0x101`) and a CSV line per sample. |
| `GPSTask`     | Event driven when GPS stream buffer has bytes   | `osPriorityNormal`        | 3072          | Reassembles DMA-fed NMEA, validates fix quality, produces CAN frames (`0x120`/`0x121`) and CSV exports. |
| `CanTxTask`   | Event driven when `Q_CAN_TX` provides a message | `osPriorityNormal1`       | 3072          | Sole CAN producer; retries `CAN1_SendStd` until HAL acknowledges the frame. |
| `UartTxTask`  | Event driven when UART stream buffer has bytes  | `osPriorityBelowNormal`   | 1536          | Flushes CSV telemetry bursts to the PC on USART2 with blocking HAL writes. |
| `LedTask`     | 1 ms delay placeholder                          | `osPriorityLow`           | 1024          | Reserved for future diagnostics; currently maintains stack for instrumentation. |

Supporting communication primitives:

- `SB_GPS` (512 B, trigger = 32 B): USART1 Receive-to-Idle DMA pushes raw bytes here; GPSTask drains it without busy waiting.
- `SB_UART_TX` (512 B, trigger = 32 B): Aggregates CSV lines from IMU/GPS publishers and decouples them from the blocking TX task. Drops accumulate in `uart_drop_bytes`.
- `Q_CAN_TX` (32 entries): Priority-neutral arbitration between IMU and GPS producers. `CAN_SendMessage` increments per-source drop counters on timeout.

With these periods the CPU budget is dominated by the 10 ms IMU poll. The
remaining tasks run only when data is available, keeping average utilisation low
while guaranteeing deterministic CAN timing.

### Fault detection and recovery

- `CAN1_SendStd` retries for up to 5 ms before surfacing an error to the caller,
  preventing permanent blockage if all mailboxes are temporarily busy.
- GPS parsing clears fix/quality fields immediately when a sentence reports an
  invalid solution so stale coordinates are never re-used.
- UART stream-buffer overruns increment `uart_drop_bytes` for post-run analysis.
- The USART1 Receive-to-Idle ISR re-arms DMA via `GPS_Start()` on init, idle, and
  error callbacks to guarantee continuous GPS reception.

## Data interfaces

### CAN (11-bit identifiers)

| ID    | Payload description                                                                 |
|-------|---------------------------------------------------------------------------------------|
| 0x100 | IMU accelerations in mm/s² (`ax`, `ay`, `az`) + sample counter (`ctr`)               |
| 0x101 | IMU angular rates in cdeg/s (`gx`, `gy`, `gz`) + temperature in c°C (`temp`)         |
| 0x120 | GPS latitude/longitude as signed degrees × 10⁷ (`lat`, `lon`)                        |
| 0x121 | GPS speed cm/s, course cdeg, HDOP × 100, fix flag, satellite count                   |

### UART (CSV @ 115200-8N1 on USART2)

```
IMU,<timestamp_ms>,<ax_mm_s2>,<ay_mm_s2>,<az_mm_s2>,<gx_cdeg_s>,<gy_cdeg_s>,<gz_cdeg_s>,<temp_cC>
GPS,<timestamp_ms>,<lat_degE7>,<lon_degE7>,<speed_cm_s>,<course_cdeg>,<hdop_centi>,<fix>,<sats>
```

The PC link is unidirectional (MCU → host). Connect to pins PA2 (TX) and PA3 (RX)
through a USB-UART bridge configured for 115200 bps.

## Building and flashing

1. Open the project in **STM32CubeIDE** (create a workspace and import the `Core/`
   sources) or generate a CubeMX project targeting STM32F407VETx with the same
   peripheral configuration (I²C1, USART1 RX DMA, USART2 TX, CAN1, FreeRTOS).
2. Ensure the generated project references the provided sources in `Core/Src` and
   headers in `Core/Inc`.
3. Build and flash using an ST-LINK probe.

## Testing

Static analysis can be executed on the sources from this repository using `cppcheck`:

```
cppcheck --enable=all --inconclusive --std=c99 Core/Src Core/Inc
```

> **Note:** CubeMX-generated sources expect STM32 HAL headers from the IDE
> installation. When those headers are not present relative to this repository,
> `cppcheck` reports "missing include" diagnostics for files such as
> `stm32f4xx_hal.h`. These messages are expected and do not indicate functional
> issues with the firmware sources.

Hardware-in-the-loop testing should confirm:
- IMU frames on CAN IDs 0x100/0x101 and UART `IMU` lines at 100 Hz.
- GPS frames on CAN IDs 0x120/0x121 and UART `GPS` lines when a fix is present.
- No CAN transmit underruns (monitor drop counters via debugger if necessary).

For more background see `Core/core_analysis.md` and the PlantUML flow chart in
`Core/core_flow.pu`.
