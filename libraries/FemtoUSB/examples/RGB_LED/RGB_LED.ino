/**
 * RGB LED sketch - By Femto.io
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
 */

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

    /*fade from rgb_in to rgb_out*/
    void fade( const rgb_color& in,
               const rgb_color& out,
               unsigned n_steps = 256,  //default take 256 steps
               unsigned time    = 10)   //wait 10 ms per step
    {
      int red_diff   = out.r() - in.r();
      int green_diff = out.g() - in.g();
      int blue_diff  = out.b() - in.b();
      for ( unsigned i = 0; i < n_steps; ++i){
        /* output is the color that is actually written to the pins
         * and output nicely fades from in to out.
         */
        rgb_color output ( in.r() + i * red_diff / n_steps,
                           in.g() + i * green_diff / n_steps,
                           in.b() + i * blue_diff/ n_steps);
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
        
        delay(time);
      }
    }

};

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

void setup()
{
  //pins driven by analogWrite do not need to be declared as outputs
}

void loop()
{
  

  /*fade colors*/
  f.fade( red,    orange,   256, 5);
  f.fade( orange, yellow,   256, 5);
  f.fade( yellow, green,    256, 5);
  f.fade( green,  cyan,     256, 5);
  f.fade( cyan,   blue,     256, 5);
  f.fade( blue,   pink,     256, 5);
  f.fade( pink,   red,      256, 5);
  
}

