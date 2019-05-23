// Simple ZeroDMA example -- an equivalent to the memcpy() function.
// Decause it uses DMA, unlike memcpy(), your code could be doing other
// things simultaneously while the copy operation runs.

#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"

Adafruit_ZeroDMA myDMA;
ZeroDMAstatus    stat; // DMA status codes returned by some functions

// The memory we'll be moving:
#define DATA_LENGTH 1024
uint8_t source_memory[DATA_LENGTH],
        destination_memory[DATA_LENGTH];

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

  Serial.println("DMA test: memory copy");

  Serial.print("Allocating DMA channel...");
  stat = myDMA.allocate();
  myDMA.printStatus(stat);

  Serial.println("Setting up transfer");
  myDMA.addDescriptor(source_memory, destination_memory, DATA_LENGTH);

  Serial.println("Adding callback");
  // register_callback() can optionally take a second argument
  // (callback type), default is DMA_CALLBACK_TRANSFER_DONE
  myDMA.setCallback(dma_callback);

  // Fill the source buffer with incrementing bytes, dest buf with 0's
  for(uint32_t i=0; i<DATA_LENGTH; i++) source_memory[i] = i;
  memset(destination_memory, 0, DATA_LENGTH);

  // Show the destination buffer is empty before transfer
  Serial.println("Destination buffer before transfer:");
  dump();

  Serial.println("Starting transfer job");
  stat = myDMA.startJob();
  myDMA.printStatus(stat);

  Serial.println("Triggering DMA transfer...");
  t = micros();
  digitalWrite(LED_BUILTIN, HIGH);
  myDMA.trigger();

  // Your code could do other things here while copy happens!

  while(!transfer_is_done); // Chill until DMA transfer completes

  digitalWrite(LED_BUILTIN, LOW);
  t = micros() - t; // Elapsed time

  Serial.print("Done! ");
  Serial.print(t);
  Serial.println(" microseconds");

  Serial.println("Destination buffer after transfer:");
  dump();

  // Now repeat the same operation, but 'manually' using memcpy() (not DMA):
  t = micros();
  digitalWrite(LED_BUILTIN, HIGH);
  memcpy(destination_memory, source_memory,  DATA_LENGTH);
  digitalWrite(LED_BUILTIN, LOW);
  t = micros() - t; // Elapsed time
  Serial.print("Same operation without DMA: ");
  Serial.print(t);
  Serial.println(" microseconds");
}

// Show contents of destination_memory[] array
void dump() {
  for(uint32_t i=0; i<DATA_LENGTH; i++) {
    Serial.print(destination_memory[i], HEX); Serial.print(' ');
    if ((i & 15) == 15) Serial.println();
  }
}

void loop() { }
