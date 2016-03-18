/*
  Reading a serial ASCII-encoded string.

 This sketch demonstrates the Serial parseInt() function.
 It looks for an ASCII string of comma-separated values.
 It parses them into ints, and uses those to fade an RGB LED.

 Circuit: Common-anode RGB LED wired like so:
 * Red cathode: digital pin 3
 * Green cathode: digital pin 5
 * blue cathode: digital pin 6
 * anode: +5V

 created 13 Apr 2012
 by Tom Igoe

 modified 3 Mar 2016
 by Femto.io for use w/ FemtoUSB (ARM) r1.0.1

 This example code is in the public domain.
 */

/*
 * Arduino Pin  | Chip port   | Color channel
 * -------------+-------------+--------------
 * D4~          | PA08        | Red
 * D3~          | PA09        | Green
 * D10~         | PA17        | Blue
 * 
 * 
 * Once loaded, open up the Serial Window.
 * Make sure to select "Newline" for line endings.
 * 
 * Try sending the following:
 * 
 * To get Red, send:
 *     255,0,0
 *     
 * To get Yellow, send:
 *     255,128,0
 *     
 * To get Green, send:
 *     0,255,0
 *     
 * To get Cyan, send:
 *     0,128,255
 * 
 * To get Blue, send:
 *     0,0,255
 * 
 * To get Violet, send:
 *     128,0,255
 * 
 * To get Magenta, send:
 *     255,0,255
 * 
 */
// pins for the LEDs:
const int redPin = 4;
const int greenPin = 3;
const int bluePin = 10;

bool isRedOn = true;
bool isGreenOn = true;
bool isBlueOn = true;

void setup() {
  // initialize serial:
  while(!SerialUSB);
  SerialUSB.begin(115200); // Required baud rate arg, though it's disregarded internally
  SerialUSB.println("Starting!");
  
  // make the pins outputs:
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  analogWrite(redPin, 0);
  analogWrite(greenPin, 255);
  analogWrite(bluePin, 0);

}

void loop() {
  // if there's any serial available, read it:
  while (SerialUSB.available()) {

    // look for the next valid integer in the incoming serial stream:
    int red = SerialUSB.parseInt();
    // do it again:
    int green = SerialUSB.parseInt();
    // do it again:
    int blue = SerialUSB.parseInt();

    // look for the newline. That's the end of your
    // sentence:
    if (SerialUSB.read() == '\n') {
      // constrain the values to 0 - 255 and invert
      // if you're using a common-cathode LED, just use "constrain(color, 0, 255);"
      red = 255 - constrain(red, 0, 255);
      green = 255 - constrain(green, 0, 255);
      blue = 255 - constrain(blue, 0, 255);

      // fade the red, green, and blue legs of the LED:
      if (red < 255) {
        if(!isRedOn) {
          isRedOn = true;
          pinMode(redPin, OUTPUT);
        }
        
        analogWrite(redPin, red);
      } else {
        if (isRedOn) {
          isRedOn = false;
          pinMode(redPin, INPUT);
        }
      }

      if (green < 255) {
        if(!isGreenOn) {
          isGreenOn = true;
          pinMode(greenPin, OUTPUT);
        }
        analogWrite(greenPin, green);
      } else {
        if(isGreenOn) {
          isGreenOn = false;
          pinMode(greenPin, INPUT);
        }
      }

      if (blue < 255) {
        if(!isBlueOn) {
          isBlueOn = true;
          pinMode(bluePin, OUTPUT);
        }
        analogWrite(bluePin, blue);
      } else {
        if(isBlueOn) {
          isBlueOn = false;
          pinMode(bluePin, INPUT);
        }
      }
      
      // print the three numbers in one string as hexadecimal:
      SerialUSB.print(red, HEX);
      SerialUSB.print(green, HEX);
      SerialUSB.println(blue, HEX);
    }
  }
}


