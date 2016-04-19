/**
 * RGB LED Pushbutton sketch - By Femto.io
 * 
 * Uses the FemtoUSB (Atmel SAM D21, E variant) with onboard 
 * RGB LED (OSRAM LRTB R48G) to fade between different colours.
 * 
 * See http://stackoverflow.com/questions/15803986/fading-arduino-rgb-led-from-one-color-to-the-other
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

/*
 * LedBrightness sketch
 * controls the brightness of LEDs on "analog" (PWM) output ports.
 * 
 */
#define Serial SerialUSB

class rgb_color {

  private:
    int my_r;
    int my_g;
    int my_b;

  public:
    rgb_color (int red, int green, int blue)
      :
        my_r(red),
        my_g(green),
        my_b(blue)
    {
    }

    int r() const {return my_r;}
    int b() const {return my_b;}
    int g() const {return my_g;}
};

/*instances of fader can fade between two colors*/
class fader {

  private:
    int r_pin;
    int g_pin;
    int b_pin;
    
  public:
    int current_step = 0;
    long last_check = 0;
    bool is_on = true;
    bool is_complete = false;
    /* construct the fader for the pins to manipulate.
     * make sure these are pins that support Pulse
     * width modulation (PWM), these are the digital pins
     * denoted with a tilde(~) common are ~3, ~5, ~6, ~9, ~10 
     * and ~11 but check this on your type of arduino. 
     */ 
    fader( int red_pin, int green_pin, int blue_pin)
      :
        r_pin(red_pin),
        g_pin(green_pin),
        b_pin(blue_pin)
    {
    }

    void off()
    {
      is_on = false;
      analogWrite( r_pin, 256 );
      analogWrite( g_pin, 256 );
      analogWrite( b_pin, 256 );  
    }

    /*fade from rgb_in to rgb_out*/
    void fade( const rgb_color& in,
               const rgb_color& out,
               unsigned n_steps = 256,  //default take 256 steps
               long time    = 10)   //wait 10 ms per step
    {
      
      
      if (last_check == 0)
      {
        last_check = millis();
      }

      if (millis() > last_check + time)
      {
        int red_diff   = out.r() - in.r();
        int green_diff = out.g() - in.g();
        int blue_diff  = out.b() - in.b();
        
        current_step++;
        last_check = millis();
        
        if (current_step < n_steps)
        {
  
          rgb_color output ( in.r() + current_step * red_diff / n_steps,
                             in.g() + current_step * green_diff / n_steps,
                             in.b() + current_step * blue_diff/ n_steps);
          /*put the analog pins to the proper output.*/
          if (output.r() > 0) {
            analogWrite( r_pin, 255 - output.r() );
          }
  
          if (output.g() > 0) {
            analogWrite( g_pin, 255 - output.g() );
          }
  
          if (output.b() > 0) {
            analogWrite( b_pin, 255 - output.b() );
          }
        } else {
          current_step = 0;
          is_complete = true;
        } 
      }
    }

};

// Use RGB LED Pins D4, D3, D10
fader f (4, 3, 10);
/*colors*/
rgb_color yellow( 255, 128,   0 );
rgb_color orange( 255,  32,   0 );
rgb_color red   ( 255,   0,   0 );
rgb_color cyan  (   0, 128, 255 );
rgb_color blue  (   0,   0, 255 );
rgb_color purple( 128,   0, 255 );
rgb_color pink  ( 255,   0, 255 );
rgb_color green (   0, 255,   0 );

int colors[7] = {0, 1, 2, 3, 4, 5, 6};
int color_steps[3] = {256, 128, 64};
int color_index = 0;
int color_step_index = 0;


bool button_pin = 5; // Pin D5 (PA27)
int button_pin_status = 0;
int button_pressed = 0;

bool play_mode = true;

void setup()
{
  while(!Serial);
  Serial.begin(9600);
  Serial.println("WHAT HUH!");
  pinMode(button_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(button_pin), buttonEvent, CHANGE);
  
  //pins driven by analogWrite do not need to be declared as outputs
}

void buttonEvent()
{
  // Is the button pressed/released?
  play_mode = !play_mode;
}

void loop()
{
  
  if (play_mode)
  { 
    f.is_on = true;
    // fade colors
    switch(color_index)
    {
      case 6:
        f.fade( pink,   red,      color_steps[color_step_index], 5);
        break;
      case 5:
        f.fade( blue,   pink,     color_steps[color_step_index], 5);
        break;
      case 4:
        f.fade( cyan,   blue,     color_steps[color_step_index], 5);
        break;
      case 3:
        f.fade( green,  cyan,     color_steps[color_step_index], 5);
        break;
      case 2:
        f.fade( yellow, green,    color_steps[color_step_index], 5);
        break;
      case 1:
        f.fade( orange, yellow,   color_steps[color_step_index], 5);
        break;
      default:
        f.fade( red,    orange,   color_steps[color_step_index], 5);
        break;
    }
  
    // After the fade animation is complete, go to the next fade.
    if (f.is_complete)
    {
      f.is_complete = false;
      color_index++;
  
      if (color_index > 6)
      {
        color_index = 0;
      }
    }
  } else {
    if (f.is_on) {
      f.off();
    }
  }
}
