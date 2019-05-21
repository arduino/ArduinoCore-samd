// DMA-based SPI buffer write.  This is transmit-only as written, i.e.
// not equivalent to Arduino's SPI.transfer() which both sends and
// receives a single byte or word.  Also, this is single-buffered to
// demonstrate a simple SPI write case.  See zerodma_spi2.ino for an
// example using double buffering (2 buffers alternating fill & transmit).

#include <SPI.h>
#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"

Adafruit_ZeroDMA myDMA;
ZeroDMAstatus    stat; // DMA status codes returned by some functions

// The memory we'll be issuing to SPI:
#define DATA_LENGTH 2048
uint8_t source_memory[DATA_LENGTH];

volatile bool transfer_is_done = false; // Done yet?

// Callback for end-of-DMA-transfer
void dma_callback(Adafruit_ZeroDMA *dma) {
  transfer_is_done = true;
}

void setup() {
  uint32_t t;
  pinMode(LED_BUILTIN, OUTPUT);   // Onboard LED can be used for precise
  digitalWrite(LED_BUILTIN, LOW); // benchmarking with an oscilloscope
  Serial.begin(115200);
  while(!Serial);                 // Wait for Serial monitor before continuing

  Serial.println("DMA test: SPI data out");

  SPI.begin();

  Serial.println("Configuring DMA trigger");
#ifdef __SAMD51__
  // SERCOM2 is the 'native' SPI SERCOM on Metro M4
  myDMA.setTrigger(SERCOM2_DMAC_ID_TX);
#else
  // SERCOM4 is the 'native' SPI SERCOM on most M0 boards
  myDMA.setTrigger(SERCOM4_DMAC_ID_TX);
#endif
  myDMA.setAction(DMA_TRIGGER_ACTON_BEAT);

  Serial.print("Allocating DMA channel...");
  stat = myDMA.allocate();
  myDMA.printStatus(stat);

  Serial.println("Setting up transfer");
  myDMA.addDescriptor(
    source_memory,                    // move data from here
#ifdef __SAMD51__
    (void *)(&SERCOM2->SPI.DATA.reg), // to here (M4)
#else
    (void *)(&SERCOM4->SPI.DATA.reg), // to here (M0)
#endif
    DATA_LENGTH,                      // this many...
    DMA_BEAT_SIZE_BYTE,               // bytes/hword/words
    true,                             // increment source addr?
    false);                           // increment dest addr?

  Serial.println("Adding callback");
  // register_callback() can optionally take a second argument
  // (callback type), default is DMA_CALLBACK_TRANSFER_DONE
  myDMA.setCallback(dma_callback);

  // Fill the source buffer with incrementing bytes
  for(uint32_t i=0; i<DATA_LENGTH; i++) source_memory[i] = i;

  // Start SPI transaction at 12 MHz
  SPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));

  Serial.println("Starting transfer job");
  t = micros();
  digitalWrite(LED_BUILTIN, HIGH);

  // Because we've configured a peripheral trigger (SPI), there's
  // no need to manually trigger transfer, it starts up on its own.
  stat = myDMA.startJob();

  // Your code could do other things here while SPI write happens!

  while(!transfer_is_done); // Chill until DMA transfer completes

  digitalWrite(LED_BUILTIN, LOW);
  t = micros() - t; // Elapsed time

  SPI.endTransaction();
  myDMA.printStatus(stat); // Results of start_transfer_job()

  Serial.print("Done! ");
  Serial.print(t);
  Serial.println(" microseconds");
}

void loop() { }
