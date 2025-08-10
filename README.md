# Telemetry-ITQ

**Baja SAE Telemetry System**  
End-to-end, open-source telemetry stack for a Baja SAE vehicle.

---

## 1. Architecture (High Level)

**Sensors → Analog front-ends → STM32F407 (RTOS)**  
→ **CAN 500 kbps → ESP32 + MCP2515**  
→ **MQTT (Wi-Fi) → Mosquitto → Node-RED → InfluxDB → Grafana**

**Key design goals:**
- Low-latency acquisition and robust scheduling with RTOS.
- Efficient CAN payloads (binary), mapping to human-friendly MQTT JSON.
- Topics and QoS aligned with criticity (kinematics > thermal/housekeeping).

---

## 2. Hardware (Summary)

- **Wheel speed (4x VR):** Reluctance variable sensors + op-amp front-ends (gain + filter + 0–3.3 V clamp).
- **IMU:** MPU6050 (I²C) at vehicle CG.
- **GPS:** UART (NMEA).
- **Temperatures:** LM35 or thermocouples (INA + cold-junction compensation) — MCU box & near engine.
- **Brake position:** Potentiometer/Hall → ADC (0–3.3 V) + LPF.
- **MCU:** STM32F407VET6 (ADC, TIM2 IC CH1-4, I²C1, USART2, CAN1, DMA).
- **Gateway:** ESP32 + MCP2515 (SPI) + CAN transceiver (TJA1050/MCP2551).
- **Power:** 12 V → DC-DC 5 V / 3.3 V, TVS, reverse polarity, LC filters, star ground.
- **CAN:** 500 kbps, twisted pair, 120 Ω terminators at both ends.

**Suggested pin map (STM32):**
| Function           | Pins        |
|--------------------|-------------|
| VR FL/FR/RL/RR     | TIM2 CH1/CH2/CH3/CH4 (PA0..PA3) |
| IMU I²C1           | PB6/PB7     |
| GPS USART2         | PA2/PA3     |
| Temps/Brake        | ADC1 IN10/IN11/IN12 (PC0/PC1/PC2) |
| CAN1               | PD0/PD1 (with transceiver) |

---

## 3. Software Layout

```
/firmware-stm32/              # STM32F407VET6 (HAL + FreeRTOS)
  ├─ Core/Src/main.c          # Acquisition, processing, CAN Tx (provided)
  ├─ ...                      # CubeMX generated files
/gateway-esp32-arduino/       # ESP32 + MCP2515 → Wi-Fi/MQTT
  └─ gateway.ino              # CAN→MQTT publisher (provided)
/flows/                       # Node-RED flows (optional)
/dashboards/                  # Grafana JSON exports (optional)
```

---

## 4. Data Flow & Protocols

### 4.1 MQTT Topic Scheme

Use placeholders in angle brackets:

```
baja/<vehiculoId>/kinematics/wheels/<pos>   # pos: FL|FR|RL|RR
baja/<vehiculoId>/imu
baja/<vehiculoId>/gps
baja/<vehiculoId>/temp/<loc>                # loc: mcu_box|motor
baja/<vehiculoId>/brake/position
baja/<vehiculoId>/health
```

**Subscribe examples:**
```
baja/+/kinematics/wheels/+
baja/+/imu
baja/+/gps
baja/+/temp/+
baja/+/brake/position
baja/+/health
```

### 4.2 CAN → MQTT Mapping

| CAN ID        | MQTT Topic                                | JSON Fields (examples)                                       |
|---------------|-------------------------------------------|--------------------------------------------------------------|
| 0x100..0x103  | baja/<vehiculoId>/kinematics/wheels/FL FR |                                                              |
| 0x110 (IMU)   | baja/<vehiculoId>/imu                     | `{ "ts": ms, "imu": {ax,ay,az,gx,gy,gz} }`                   |
| 0x120 (GPS)   | baja/<vehiculoId>/gps                     | `{ "ts": ms, "gps": {speed, lat?, lon?, hdop?}}`             |
| 0x130 (TEMP)  | baja/<vehiculoId>/temp/<loc>              | `{ "ts": ms, "value": degC }`                                |
| 0x140 (BRAKE) | baja/<vehiculoId>/brake/position          | `{ "ts": ms, "position": 0..1 }`                             |
| 0x150 (HEALTH)| baja/<vehiculoId>/health                  | `{ "ts": ms, "vbat": V, "rssi": dBm, "cpu_load":%,"status":"online" }` |

**Binary scaling over CAN (from STM32):**
- Wheel speed: int16 = mps × 1000
- IMU: ax,ay,az (×100), gz (×1000) (rad/s)
- GPS speed: int16 = mps × 100
- Temp: int16 = degC × 10
- Brake: uint16 = pos × 1000
- Health: vbat (×100), rssi (int8), cpu (uint8), status(0x01)

---

## 5. Building & Flashing

### 5.1 STM32F407VET6 (HAL + FreeRTOS)

- CubeMX: Create project with HAL + FreeRTOS. Enable:  
  TIM2 (IC CH1..CH4), I²C1, USART2, ADC1 (3 channels), DMA for ADC, CAN1.
- System clock to 168 MHz.
- Pin assignment as in 'Hardware'.
- In `Core/Src/main.c`, paste the provided implementation (acquisition, processing, CAN Tx).
- Build & flash (ST-Link).
- Tune constants:
    - `K_TICK_TO_MPS` (wheel geometry & tooth count)
    - UART baud for GPS (9 600–115 200)
    - CAN timing if your APB clocks differ

**Main features implemented:**
- FreeRTOS tasks: acq (high), proc (mid-high), canTx (mid-high), health (low).
- TIM2 IC for VR pulses, I²C read of MPU6050, UART NMEA parsing (GPRMC speed), ADC+DMA for temps/brake.
- Simple LPFs + basic fusion (wheels vs GPS).
- Compact CAN frames per mapping above.

### 5.2 ESP32 (Arduino)

- **Libraries (Arduino Library Manager):**
    - MCP_CAN (Cory J. Fowler)
    - PubSubClient (Nick O’Leary)
- **Pins (default VSPI):**
    ```
    SCK=18, MISO=19, MOSI=23, CS=5, INT=27  // adjust if needed
    ```
- Configure in `gateway.ino`:
    - `WIFI_SSID`, `WIFI_PASS`
    - `MQTT_HOST`, `MQTT_PORT`
    - `VEHICULO_ID`, `CLIENT_ID`
    - MCP2515 crystal: MCP_16MHZ (or MCP_8MHZ)
- Build & upload.
- Subscribe to `baja/#` in MQTTX to see data.

---

## 6. Server Stack

### 6.1 Mosquitto (broker)

Minimal `/etc/mosquitto/mosquitto.conf`:
```
listener 1883
allow_anonymous false
password_file /etc/mosquitto/passwd
persistence true
persistence_location /var/lib/mosquitto/
# listener 8883  # (enable for TLS)
# cafile /etc/mosquitto/certs/ca.crt
# certfile /etc/mosquitto/certs/server.crt
# keyfile /etc/mosquitto/certs/server.key
```

### 6.2 Node-RED (ingest & ETL)

**Typical flow:**
```
[mqtt in] -> [json] -> [function(tagging/timestamp)] -> [switch by measurement] -> [influxdb out]
```
**Subscriptions:**  
`baja/+/kinematics/wheels/+`, `baja/+/imu`, `baja/+/gps`, `baja/+/temp/+`, `baja/+/brake/position`, `baja/+/health`

### 6.3 InfluxDB

- **Buckets:** telemetry_raw (30d), telemetry_agg (1y).
- **Measurements:**
    - `wheel_speed(vehicle,pos) -> value,q`
    - `imu(vehicle) -> ax,ay,az,gx,gy,gz`
    - `gps(vehicle) -> speed,lat,lon,hdop`
    - `temp(vehicle,loc) -> value`
    - `brake(vehicle) -> position`
    - `health(vehicle) -> vbat, rssi, cpu_load`
- Flux task for downsampling (1–5 s).

### 6.4 Grafana / InfluxDB UI

**Recommended dashboards:**
- Dynamics: wheel speeds, total speed (IMU+GPS), accel/gyro.
- Driving: brake position vs speed.
- Thermal: MCU box / engine temps with alerts.
- Health: battery voltage, RSSI, CPU load, end-to-end latency.

---

## 7. Testing & Validation

- **Latency end-to-end (sensor → dashboard):** target ≤ 150–200 ms.
- **Packet loss:** ≤ 1% over 30 min.
- **Accuracy:**
    - Wheels vs optical tachometer.
    - IMU vs lab reference; bias/scale calibration.
    - Temp vs calibrated thermometer.
    - GPS speed vs reference GPS.

**Tools:**  
- MQTTX for topic sanity  
- Node-RED debug nodes for payload checks  
- InfluxDB queries to verify storage

---

## 8. Troubleshooting

- **No data on dashboard:** Check Wi-Fi, broker status, Node-RED subscriptions, and `baja/<vehiculoId>/health` status.
- **Overlapping/garbled topics in docs:** In LaTeX use `tabularx` + `\url{}` + `hidelinks` to wrap/break topics.
- **High jitter / missed captures:** Reduce acquisition rate, verify TIM2 IC filter, check analog front-end thresholds.
- **CAN errors:** Ensure 120 Ω termination, same bitrate, clean grounds.
- **MQTT disconnects:** Inspect RSSI; enable exponential backoff (already in ESP32 sketch).

---

## 9. Security & Resilience

- MQTT auth (username/password), LWT online/offline, reconnection backoff.
- Optional TLS (MQTTS) on port 8883.
- Buffering/queues in Node-RED to absorb bursts; outlier filters.

---

## 10. Credits / License

- STM32 HAL / FreeRTOS.
- ESP32 Arduino core, MCP_CAN, PubSubClient.
- Mosquitto, Node-RED, InfluxDB, Grafana.
- *Add your license of choice (MIT/BSD/GPL) here.*

---

**Related files:**  
- `ESP32:` `gateway-esp32-arduino/gateway.ino`  
- (Optional) `flows/node-red-flow.json`, `dashboards/grafana-dashboard.json`

ESP32: gateway-esp32-arduino/gateway.ino

(Optional) flows/node-red-flow.json, dashboards/grafana-dashboard.json
