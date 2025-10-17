#include <SPI.h>
#include "mcp_can.h"

// ==== Pines (ajusta si usas otros) ====
static const int PIN_CS  = 5;   // CS/NSS del MCP2515 CHIP SELECT
static const int PIN_INT = 4;   // INT del MCP2515 (activo en LOW) pin de interrupción del MCP2515; lo pone en LOW cuando llega un mensaje CAN.

// ==== Parámetros del bus (ajústalos a tu red) ====
#define CAN_BAUD CAN_500KBPS     // CAN_500KBPS, bits por segundo 
#define MCP_CLK  MCP_8MHZ        // MCP_8MHZ o MCP_16MHZ según tu módulo

MCP_CAN CAN(PIN_CS);  //Crea el objeto controlador CAN y le dice qué pin usa para CS.

void setup() {
  Serial.begin(115200); //INICIALIZACION DEL PUERTO SERIE. tasa de baudios (bits por segundo)
  while (!Serial) { delay(10); }

  pinMode(PIN_INT, INPUT); //Declara PIN_INT como entrada. Mensajes informativos por Serial.
  Serial.println();
  Serial.println(F("ESP32 + MCP2515 - Receptor CAN (API antigua de mcp_can)"));

  // Inicializa MCP2515 (acepta STD (estandar) y EXT) y reintenta hasta que funcione MCP_ANY: acepta IDs estándar (11 bits) y extendidos (29 bits). CAN_BAUD: la velocidad que definiste. MCP_CLK: frecuencia del cristal.
  while (CAN.begin(MCP_ANY, CAN_BAUD, MCP_CLK) != CAN_OK) {
    Serial.println(F("Error init MCP2515 (clock/baudios/cableado). Reintentando..."));
    delay(500);
  }

  // Modo normal; para esnifar sin ACK : MCP_LISTENONLY
  CAN.setMode(MCP_NORMAL);

  // Aceptar TODO (máscaras/filtros abiertos) Máscaras/Filtros: le dicen al MCP2515 qué IDs aceptar.ACEPTA TODOS 
  CAN.init_Mask(0, 0, 0x00000000);
  CAN.init_Mask(1, 0, 0x00000000);
  CAN.init_Filt(0, 0, 0x00000000);
  CAN.init_Filt(1, 0, 0x00000000);
  CAN.init_Filt(2, 0, 0x00000000);
  CAN.init_Filt(3, 0, 0x00000000);
  CAN.init_Filt(4, 0, 0x00000000);
  CAN.init_Filt(5, 0, 0x00000000);

  Serial.println(F("Listo. Esperando tramas..."));
}

void loop() {
  if (CAN.checkReceive() == CAN_MSGAVAIL) {  // ¿Hay mensaje?
    uint32_t id = 0;
    uint8_t  ext = 0;                        // 0 = STD (11 bits), 1 = EXT (29 bits) 
    uint8_t  len = 0;                           //Prepara variables: id, si es std/ext, el DLC/len (tamaño) y buf (datos, máx. 8 bytes en CAN clásico).
    uint8_t  buf[8];

    // Usa la variante con 4 argumentos de tu librería
    if (CAN.readMsgBuf(&id, &ext, &len, buf) == CAN_OK) {  //Lee la trama CAN: rellena id, ext, len y buf[]. Devuelve CAN_OK si salió bien.
      // Cabecera
      Serial.print(F("[CAN] ID=0x"));
      Serial.print(id, HEX);
      Serial.print(ext ? F(" EXT") : F(" STD"));
      Serial.print(F(" | DLC=")); // DLC (número de bytes de datos).
      Serial.print(len);
      Serial.print(F(" | Data: "));

      // Payload en HEX Muestra los datos byte a byte en HEX, con cero a la izquierda para alinear. QUE ME LOS IMPRIMA EN STRING O DECIMALES 
      for (uint8_t i = 0; i < len; i++) {
        if (buf[i] < 0x10) Serial.print('0');
        Serial.print(buf[i], HEX);
        if (i < len - 1) Serial.print(' ');
      }
      Serial.println();
    }
  }

  delay(1); // pequeño respiro
}
