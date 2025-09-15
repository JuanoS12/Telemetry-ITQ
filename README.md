Telemetría (STM32F407VET6 → CAN → ESP32 → Nube)

Estado actual: IMU (MPU6050) + GPS (NEO-6M) leídos por el STM32F407VET6 (FreeRTOS) y publicados por CAN @ 500 kbit/s hacia un ESP32 (TWAI).
Diseño preparado para integrar velocidad de ruedas (VR), pedal (ADC), temperaturas (NTC) y más, sin romper el contrato de datos.

🔎 TL;DR

Nodo de adquisición: STM32F407VET6 @ 160 MHz, FreeRTOS, I²C (MPU), UART+DMA (GPS), CAN (transmisor único).

Contrato CAN estable: IDs fijos para IMU (0x100/0x101) y GPS (0x120/0x121). Salud 0x150 (opcional).

Pasarela: ESP32 (TWAI) recibe y decodifica. (En el Doc 2 se cubre MQTT/InfluxDB/Grafana).

Escalable: lugares reservados (ruedas, pedal, temperaturas) ya definidos.




Estructura del repositorio
├─ firmware/
│  └─ stm32f407/
│     ├─ Core/Src/        # freertos.c, can.c, usart.c, i2c.c, dma.c, stm32f4xx_it.c, main.c, ...
│     ├─ Core/Inc/
│     └─ Telemetria.ioc   # Configuración CubeMX (160 MHz, I2C1, USART1+DMA, CAN1, FreeRTOS)
│
├─ gateway/
│  └─ esp32/
│     ├─ arduino_twai_monitor/      # Monitor TWAI simple (imprime/decodifica IDs actuales)
│     └─ idf_twai_monitor/          # Variante ESP-IDF
│
├─ docs/
│  ├─ Documento_1_Telemetria_Embebida.pdf   # Manual de configuración/uso (IMU+GPS→CAN)
│  └─ diagramas/                             # TikZ/figuras (arquitectura, Gantt, MSC, instrumentación)
│
├─ hardware/
│  ├─ can_bus/           # Recomendaciones de cableado, terminación, TVS, CMC
│  └─ sensores/          # Instrumentación VR, pedal (ADC), NTC (fórmulas y valores de partida)
│
└─ tools/
   └─ decoders/          # Scripts de apoyo (ejemplos de decodificación CAN en PC)


🧱 Arquitectura (resumen)

MPU6050 (I2C1 PB6/PB7)   NEO-6M (USART1 RX+DMA)
         \                   /
          \                 /
            STM32F407VET6  ---- CAN @ 500 kbit/s ----  Transceptor (SN65HVD230) ---- ESP32 (TWAI)
                    FreeRTOS: SensorTask (IMU), GPSTask (NMEA), CanTxTask (TX único), Led/Health


🔌 Pinout principal (STM32F407VET6)
Función	Puerto/Pin	Notas
I²C1 SCL/SDA	PB6 / PB7	AF4, 400 kHz, pull-ups 3.3–4.7 kΩ
USART1 RX/TX	PA10 / PA9	RX desde NEO-6M, 9600 bps, DMA2 Stream2 Channel4
CAN1 RX/TX	PA11 / PA12	AF9, vía transceptor 3.3 V (SN65HVD230)
(Futuro) PPS GPS	PC5 (sugerido)	EXTI para sincronía 1 Hz
(Futuro) Ruedas VR	TIM3 CHx	Input-capture
(Futuro) Pedal/Temp	ADCx	RC anti-ruido + protección
⚙️ Configuración CubeMX / HAL

Relojes: SYSCLK/HCLK = 160 MHz (HSE 8 MHz → PLL). APB1 = 40 MHz, APB2 = 80 MHz.

I²C1: Fast Mode 400 kHz (PB6/PB7), pull-ups 3.3–4.7 kΩ.

USART1 (GPS): 9600 bps, 8N1, DMA RX en modo NORMAL + HAL_UARTEx_ReceiveToIdle_DMA(...).

DMA: DMA2/Stream2/Channel4. Deshabilitar Half-Transfer IRQ en RX.

CAN1: 500 kbit/s. Típico (PCLK1=40 MHz): Prescaler=5, BS1=13TQ, BS2=2TQ, SJW=1TQ (como en can.c).

Filtro “accept-all” → FIFO0, HAL_CAN_Start(), notificaciones de error activadas.

FreeRTOS: usar vTaskDelayUntil para periodicidad exacta (IMU @ 100 Hz).

NVIC (FreeRTOS-safe): prioridad 5 para IRQ que llaman APIs de FreeRTOS (USART1, DMA2 S2, CAN RX0/SCE).

🧵 Tareas (FreeRTOS)

SensorTask (↑ prioridad): lee MPU6050 @ 100 Hz; publica

0x100: ax/ay/az [mg], ctr [u16]

0x101: gx/gy/gz [cdeg/s], temp [c°C]

GPSTask: parsea RMC/GGA 1–10 Hz; publica si fix=1

0x120: lat/lon [grados×10⁷]

0x121: vel [cm/s], rumbo [cdeg], HDOP×100, fix [u8], sats [u8]

CanTxTask: único transmisor CAN (lee Queue<CANMsg> y transmite).

Led/Health: latido y/o 0x150 (Vbat mV, CPU ‰, errno, ctr).

Comunicación interna: xQueueCreate(32, sizeof(CANMsg)) y xStreamBufferCreate(512, 32) para NMEA crudo.

🧾 Diccionario CAN (contrato)

Convenciones: CAN 2.0A (11 bits), DLC=8, little-endian; s16/s32 con signo (C2), u8/u16 sin signo.

ID	Nombre	Frec.	Payload (0..7) y escala
0x100	IMU_ACC	100 Hz	0:ax s16 mg, 2:ay s16 mg, 4:az s16 mg, 6:ctr u16
0x101	IMU_GYR_TMP	100 Hz	0:gx s16 cdeg/s, 2:gy s16, 4:gz s16, 6:temp s16 c°C
0x120	GPS_POS	1–10 Hz	0:lat s32 (°×10⁷), 4:lon s32 (°×10⁷)
0x121	GPS_NAV	1–10 Hz	0:vel s16 cm/s, 2:rumbo s16 cdeg, 4:HDOP s16×100, 6:fix u8, 7:sats u8
0x150	HEALTH	1 Hz	0:Vbat u16 mV, 2:CPU u16 ‰, 4:errno u16, 6:ctr u16
0x130–0x133	WHEEL_VEL_[FL..RR]	(res)	v s16 cm/s, f u16 cHz, status u8, res, ctr u16
0x140	PEDAL	(res)	nivel s16 ‰ (0..1000), status u8, res, ctr u16
0x145..	TEMP_x	(res)	T s16 c°C, status u8, res, ctr u16

Política: no romper payloads existentes. Cambios → nuevo ID (v2) y deprecación gradual.

🚀 Cómo compilar y flashear (STM32)

Requisitos: STM32CubeIDE (o Makefile si lo prefieres), ST-LINK, HAL/FreeRTOS generados por CubeMX.

Abre firmware/stm32f407/Telemetria.ioc en CubeMX/CubeIDE y genera el código.

Asegúrate de mantener los bloques /* USER CODE BEGIN */ existentes (colas, parsers).

Compila y flashea.

En main.c, al final de las inits, arranca la recepción GPS:

HAL_UARTEx_ReceiveToIdle_DMA(&huart1, gps_rx_buf, sizeof(gps_rx_buf));
__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); // sin Half-Transfer


Conecta al bus CAN (terminado con dos resistencias de 120 Ω).

🛰️ Monitor de recepción (ESP32, TWAI)

Arduino Core (rápido):

#include <driver/twai.h>
void setup(){
  Serial.begin(115200);
  auto g = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
  auto t = TWAI_TIMING_CONFIG_500KBITS();
  auto f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g,&t,&f); twai_start();
}
void loop(){
  twai_message_t m;
  if (twai_receive(&m, pdMS_TO_TICKS(1000)) == ESP_OK) {
    Serial.printf("ID 0x%03X DLC %d : ", m.identifier, m.data_length_code);
    for(int i=0;i<m.data_length_code;i++) Serial.printf("%02X ", m.data[i]);
    Serial.println();
  }
}


Verás 0x100/0x101 ≈100 Hz y 0x120/0x121 cuando el GPS tenga fix=1.

🧪 Puesta en marcha y pruebas

Bring-up rápido

3.3 V estable y GND común.

CAN: ≈ 60 Ω entre CANH-CANL con el bus apagado (dos terminaciones).

IMU: en reposo → az ≈ +1000 mg, giros ≈ 0 (tras bias).

GPS: exterior → fix=1, sats ≥ 5, HDOP ≤ 1.5.

Robustez

IWDG (watchdog) refrescado por tarea de baja prioridad.

BOR configurado en Option Bytes (3.3 V).

ctr (u16) para detectar pérdidas en receptor.

🧰 Integraciones futuras (plantillas incluidas)

Ruedas (VR): MAX9926/LM1815 o comparador Schmitt → TIM3 input-capture. Publicar 0x130–0x133.

Pedal (ADC): RC 1 kΩ/1 µF, clamps de protección → 0x140 (‰).

Temperaturas (NTC 10k): divisor con 10k, modelo Beta → 0x145+.

PPS (GPS): sincronía de 1 Hz por EXTI.

AHRS (opcional): Mahony/Madgwick @ 100–200 Hz (STM32 o ESP32).

🐛 Solución de problemas (rápido)

No veo CAN: bitrate distinto, transceptor en standby, sin terminadores, sin GND común.

IMU “congelada”: revisar pull-ups I²C, recuperar bus (toggling SCL), re-init.

GPS sin datos: confirmar ReceiveToIdle_DMA y antena a cielo abierto.

Reinicios: verificar IWDG, BOR, caídas de tensión; aumentar desacoples.

🗺️ Roadmap (propuesto)

Ruedas VR (1 canal) con plantilla → validar vs GPS en recta.

Pedal + Temperaturas (ADC) → dashboards básicos.

PPS + Salud (0x150) → mayor confiabilidad.

Pasarela MQTT/Influx/Grafana (Doc 2) → sesión en tiempo real.

AHRS (si aporta valor) → publicar orientación.

🤝 Contribuir

Ramas: feature/* para nuevas integraciones; dev para integrar; main solo releases.

Commits: feat: ..., fix: ..., docs: ..., chore: ....

Cambios de contrato CAN → nuevo ID y actualización del diccionario + README.

📜 Licencia

Indica aquí tu licencia preferida (por ejemplo, MIT).
SPDX-License-Identifier: MIT

📚 Referencias internas

docs/Documento_1_Telemetria_Embebida.pdf (manual operativo y de instrumentación).

gateway/esp32/* (monitores TWAI para pruebas).

hardware/sensores/* (fórmulas y valores sugeridos para VR, pedal, NTC).

Nota final

Este repositorio prioriza simplicidad y estabilidad: una sola tarea TX CAN, escalas fijas, y contratos claros. Si añades sensores, usa los IDs reservados y documenta los cambios. Si necesitas el README bilingüe o una plantilla de release, dímelo y lo adaptamos.
