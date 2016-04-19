#include <SPI.h>
#define Serial SERIAL_PORT_USBVIRTUAL

// Based on https://github.com/msolters/arduino-at86rf233
#include "at86rf2xx.h"

volatile int received = 0;

void setup() {
  while(!Serial);
  
  Serial.begin(9600);
  at86rf2xx.init(PIN_ATRF233_SEL, PIN_ATRF233_IRQ, PIN_ATRF233_SLP_TR, PIN_ATRF233_RST);
  at86rf2xx.set_chan(26); // set channel to 26
  
}

void loop() {
  if (at86rf2xx.events)
    at86rf2xx_eventHandler();
  return;
}

void at86rf2xx_eventHandler() {
  Serial.println("*");
  /* One less event to handle! */
  at86rf2xx.events--;

  /* If transceiver is sleeping register access is impossible and frames are
   * lost anyway, so return immediately.
   */
  byte state = at86rf2xx.get_status();
  if(state == AT86RF2XX_STATE_SLEEP)
    return;

  /* read (consume) device status */
  byte irq_mask = at86rf2xx.reg_read(AT86RF2XX_REG__IRQ_STATUS);

  /*  Incoming radio frame! */
  if (irq_mask & AT86RF2XX_IRQ_STATUS_MASK__RX_START)
    Serial.println("[at86rf2xx] EVT - RX_START");

  /*  Done receiving radio frame; call our receive_data function.
   */
  if (irq_mask & AT86RF2XX_IRQ_STATUS_MASK__TRX_END)
  {
    if(state == AT86RF2XX_STATE_RX_AACK_ON || state == AT86RF2XX_STATE_BUSY_RX_AACK) {
      Serial.println("[at86rf2xx] EVT - RX_END");
      at86rf2xx_receive_data();
    }
  }
}

void at86rf2xx_receive_data() {
  /*  print the length of the frame
   *  (including the header)
   */
  size_t pkt_len = at86rf2xx.rx_len();
  Serial.print("Frame length: ");
  Serial.print(pkt_len);
  Serial.println(" bytes");

  /*  Print the frame, byte for byte  */
  Serial.println("Frame dump (ASCII):");
  uint8_t data[pkt_len];
  at86rf2xx.rx_read(data, pkt_len, 0);
  for (int d=0; d<pkt_len; d++)
    Serial.print((char)data[d]);
  Serial.println();

  /* How many frames is this so far?  */
  Serial.print("[[Total frames received: ");
  Serial.print(++received);
  Serial.println("]]\n");
}
