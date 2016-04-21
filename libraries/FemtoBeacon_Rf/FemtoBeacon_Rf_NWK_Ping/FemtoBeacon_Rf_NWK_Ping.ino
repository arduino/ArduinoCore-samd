#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define Serial SERIAL_PORT_USBVIRTUAL

#include "lwm.h"
#include "lwm/sys/sys.h"
#include "lwm/nwk/nwk.h"

extern "C" {
  void println(char *x) { Serial.println(x); Serial.flush(); }
}
int meshAddress = 1; // 1 sends ping, 2 receives

static void sendMessage(void);
static bool receiveMessage(NWK_DataInd_t *ind);
byte pingCounter = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("LWP Ping Demo");

  SPI.usingInterrupt(digitalPinToInterrupt(PIN_SPI_IRQ));
    
  SPI.beginTransaction(
      SPISettings(
          MODULE_AT86RF233_CLOCK, 
          MSBFIRST, 
          SPI_MODE0
      )
  );

  attachInterrupt(digitalPinToInterrupt(PIN_SPI_IRQ), HAL_IrqHandlerSPI, RISING);
  /*  wait for SPI to be ready  */
  delay(10);
  
  SYS_Init();
  NWK_SetAddr(meshAddress);
  NWK_SetPanId(0x01);
  PHY_SetChannel(0x1a);
  PHY_SetRxState(true);
  NWK_OpenEndpoint(1, receiveMessage);
}

void loop() {
  SYS_TaskHandler();
  if(meshAddress == 1) sendMessage();
  if(pingCounter % 2) {
    //analogWrite(LED_GREEN, 128);
    Serial.println("-");
  } else {
    //analogWrite(LED_GREEN, 255);
    Serial.println("*");
  }
  delay(1000);
}

static void sendMessage(void) {
  pingCounter++;
  Serial.print("ping ");
  Serial.println(pingCounter);

  // we just leak for now
  NWK_DataReq_t *message = (NWK_DataReq_t*)malloc(sizeof(NWK_DataReq_t));
  message->dstAddr = 2;
  message->dstEndpoint = 1;
  message->srcEndpoint = 1;
  message->options = 0;
  message->data = &pingCounter;
  message->size = sizeof(pingCounter);
  NWK_DataReq(message);
}

static bool receiveMessage(NWK_DataInd_t *ind) {
  Serial.print("Received message - ");
  Serial.print("lqi: ");
  Serial.print(ind->lqi, DEC);

  Serial.print("  ");

  Serial.print("rssi: ");
  Serial.print(ind->rssi, DEC);
  Serial.print("  ");

  Serial.print("ping: ");
  pingCounter = (byte)*(ind->data);
  Serial.println(pingCounter);
  return true;
}
