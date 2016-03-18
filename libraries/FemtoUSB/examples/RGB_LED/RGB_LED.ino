/**
 * RGB LED sketch - By Femto.io
 * 
 * Uses the FemtoUSB (Atmel SAM D21, E variant) with onboard 
 * RGB LED (OSRAM LRTB R48G) to fade between different colours.
 * 
 * @license Public Domain, 2016 - You are free to copy, modify, whatevs.
 * 
 * How the LED is wired up to the ATSAMD21E18A chip:
 * 
 * Arduino Pin  | Chip port   | Color channel
 * -------------+-------------+--------------
 * D4~          | PA08        | Red
 * D3~          | PA09        | Green
 * D10~         | PA17        | Blue
 * 
 */

int LED_R = 4;
int LED_G = 3;
int LED_B = 10;

// Analog value per color channel
int VALUE_R = 0;
int VALUE_G = 85;
int VALUE_B = 170;

// How much to change per channel per cycle.
int AMT_R = 5;
int AMT_G = 5;
int AMT_B = 5;

void setup() {
  // Set up the RGB LED pins
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

}

void loop() {
  
  // Update our RGB values every time we loop.
  fadePinR();
  fadePinG();
  fadePinB();
  
  // Delay at least 30ms so we can perceive the color.
  delay(30);

  
}

void fadePinR() {
  // set the brightness of pin "R":
  analogWrite(LED_R, 255 - VALUE_R);

  // change the brightness for next time through the loop:
  VALUE_R = VALUE_R + AMT_R;

  // reverse the direction of the fading at the ends of the fade:
  if (VALUE_R == 0 || VALUE_R == 255) {
    AMT_R = -AMT_R ;
  }
}

void fadePinG() {
  // set the brightness of pin "G":
  analogWrite(LED_G, 255 - VALUE_G);

  // change the brightness for next time through the loop:
  VALUE_G = VALUE_G + AMT_G;

  // reverse the direction of the fading at the ends of the fade:
  if (VALUE_G == 0 || VALUE_G == 255) {
    AMT_G = -AMT_G ;
  }
}

void fadePinB() {
  // set the brightness of pin "B":
  analogWrite(LED_B, 255 - VALUE_B);

  // change the brightness for next time through the loop:
  VALUE_B = VALUE_B + AMT_B;

  // reverse the direction of the fading at the ends of the fade:
  if (VALUE_B == 0 || VALUE_B == 255) {
    AMT_B = -AMT_B ;
  }
}
