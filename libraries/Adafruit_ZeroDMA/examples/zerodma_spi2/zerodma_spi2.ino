// Double-buffered DMA on auxiliary SPI peripheral on pins 11/12/13.
// Continuously alternates between two data buffers...one is filled
// with new data as the other is being transmitted.

#include <SPI.h>
#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"
#include "wiring_private.h" // pinPeripheral() function

// Declare our own SPI peripheral 'mySPI' on pins 11/12/13:
// (Do not call this SPI1; Arduino Zero and Metro M0 already
// have an SPI1 (the EDBG interface) and it won't compile.)
SPIClass mySPI(
  &sercom1,         // -> Sercom peripheral
  34,               // MISO pin (also digital pin 12)
  37,               // SCK pin  (also digital pin 13)
  35,               // MOSI pin (also digital pin 11)
  SPI_PAD_0_SCK_1,  // TX pad (MOSI, SCK pads)
  SERCOM_RX_PAD_3); // RX pad (MISO pad)

Adafruit_ZeroDMA myDMA;
ZeroDMAstatus    stat; // DMA status codes returned by some functions

// Data we'll issue to mySPI.  There are TWO buffers; one being
// filled with new data while the other's being transmitted in
// the background.
#define DATA_LENGTH 512
uint8_t source_memory[2][DATA_LENGTH],
        buffer_being_filled = 0, // Index of 'filling' buffer
        buffer_value        = 0; // Value of fill

volatile bool transfer_is_done = true; // Done yet?

// Callback for end-of-DMA-transfer
void dma_callback(Adafruit_ZeroDMA *dma) {
  transfer_is_done = true;
}

DmacDescriptor *desc; // DMA descriptor address (so we can change contents)

void setup() {
  Serial.begin(115200);
  while(!Serial); // Wait for Serial monitor before continuing

  Serial.println("DMA test: SPI data out");

  mySPI.begin();
  // Assign pins 11, 12, 13 to SERCOM functionality
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);

  // Configure DMA for SERCOM1 (our 'mySPI' port on 11/12/13)
  Serial.println("Configuring DMA trigger");
  myDMA.setTrigger(SERCOM1_DMAC_ID_TX);
  myDMA.setAction(DMA_TRIGGER_ACTON_BEAT);

  Serial.print("Allocating DMA channel...");
  stat = myDMA.allocate();
  myDMA.printStatus(stat);

  desc = myDMA.addDescriptor(
    source_memory[buffer_being_filled], // move data from here
    (void *)(&SERCOM1->SPI.DATA.reg),   // to here
    DATA_LENGTH,                        // this many...
    DMA_BEAT_SIZE_BYTE,                 // bytes/hword/words
    true,                               // increment source addr?
    false);                             // increment dest addr?

  Serial.println("Adding callback");
  // register_callback() can optionally take a second argument
  // (callback type), default is DMA_CALLBACK_TRANSFER_DONE
  myDMA.setCallback(dma_callback);
}

void loop() {
  // Fill buffer with new data.  The other buffer might
  // still be transmitting in the background via DMA.
  memset(source_memory[buffer_being_filled], buffer_value, DATA_LENGTH);

  // Wait for prior transfer to complete before starting new one...
  Serial.print("Waiting on prior transfer...");
  while(!transfer_is_done) Serial.write('.');
  mySPI.endTransaction();
  Serial.println("Done!");

  // Modify the DMA descriptor using the newly-filled buffer as source...
  myDMA.changeDescriptor(desc,           // DMA descriptor address
    source_memory[buffer_being_filled]); // New src; dst & count don't change

  // Begin new transfer...
  Serial.println("Starting new transfer job");
  mySPI.beginTransaction(SPISettings(12000000, MSBFIRST, SPI_MODE0));
  transfer_is_done = false;            // Reset 'done' flag
  stat             = myDMA.startJob(); // Go!
  myDMA.printStatus(stat);

  // Switch buffer indices so the alternate buffer is filled/xfer'd
  // on the next pass.
  buffer_being_filled = 1 - buffer_being_filled;
  buffer_value++;
}

