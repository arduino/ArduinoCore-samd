/*
* Nunchuck functions -- Talk to a Wii Nunchuck
*
* This library is from the Bionic Arduino course :
* http://todbot.com/blog/bionicarduino/
*
* 2007-11 Tod E. Kurt, http://todbot.com/blog/
*
* The Wii Nunchuck reading code originally from Windmeadow Labs
* http://www.windmeadow.com/node/42
*/

#include <Arduino.h>

static uint8_t nunchuck_buf[6]; // array to store nunchuck data,

// Uses port C (analog in) pins as power & ground for Nunchuck
static void nunchuck_setpowerpins()
{
/*  
  #define pwrpin PORTC3
  #define gndpin PORTC2
  DDRC |= _BV(pwrpin) | _BV(gndpin);
  PORTC &=~ _BV(gndpin);
  PORTC |= _BV(pwrpin);
  delay(100); // wait for things to stabilize
*/
}

// initialize the I2C system, join the I2C bus,
// and tell the nunchuck we're talking to it
static void nunchuck_init()
{
  Wire.begin(); // join i2c bus as master
  Wire.beginTransmission(0x52);// transmit to device 0x52
  Wire.write((uint8_t)0x40);// sends memory address
  Wire.write((uint8_t)0x00);// sends sent a zero.
  Wire.endTransmission();// stop transmitting
}

// Send a request for data to the nunchuck
// was "send_zero()"
static void nunchuck_send_request()
{
  Wire.beginTransmission(0x52);// transmit to device 0x52
  Wire.write((uint8_t)0x00);// sends one byte
  Wire.endTransmission();// stop transmitting
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
static char nunchuk_decode_byte (char x)
{
  x = (x ^ 0x17) + 0x17;
  return x;
}

// Receive data back from the nunchuck,
// returns 1 on successful read. returns 0 on failure
static int nunchuck_get_data()
{
  int cnt=0;
  Wire.requestFrom (0x52, 6);// request data from nunchuck
  while (Wire.available ()) {
    // receive byte as an integer
    nunchuck_buf[cnt] = nunchuk_decode_byte( Wire.read() );
    cnt++;
  }
  nunchuck_send_request(); // send request for next data payload
  // If we recieved the 6 bytes, then go print them
  if (cnt >= 5) {
    return 1; // success
  }
  return 0; //failure
}

// Print the input data we have recieved
// accel data is 10 bits long
// so we read 8 bits, then we have to add
// on the last 2 bits. That is why I
// multiply them by 2 * 2
static void nunchuck_print_data()
{
  static int i=0;
  int joy_x_axis = nunchuck_buf[0];
  int joy_y_axis = nunchuck_buf[1];
  int accel_x_axis = nunchuck_buf[2]; // * 2 * 2;
  int accel_y_axis = nunchuck_buf[3]; // * 2 * 2;
  int accel_z_axis = nunchuck_buf[4]; // * 2 * 2;

  int z_button = 0;
  int c_button = 0;

  // byte nunchuck_buf[5] contains bits for z and c buttons
  // it also contains the least significant bits for the accelerometer data
  // so we have to check each bit of byte outbuf[5]
  if ((nunchuck_buf[5] >> 0) & 1)
  z_button = 1;
  if ((nunchuck_buf[5] >> 1) & 1)
  c_button = 1;

  if ((nunchuck_buf[5] >> 2) & 1)
  accel_x_axis += 1;
  if ((nunchuck_buf[5] >> 3) & 1)
  accel_x_axis += 2;

  if ((nunchuck_buf[5] >> 4) & 1)
  accel_y_axis += 1;
  if ((nunchuck_buf[5] >> 5) & 1)
  accel_y_axis += 2;

  if ((nunchuck_buf[5] >> 6) & 1)
  accel_z_axis += 1;
  if ((nunchuck_buf[5] >> 7) & 1)
  accel_z_axis += 2;

  Serial5.print(i,DEC);
  Serial5.print("\t");

  Serial5.print("joy:");
  Serial5.print(joy_x_axis,DEC);
  Serial5.print(",");
  Serial5.print(joy_y_axis, DEC);
  Serial5.print(" \t");

  Serial5.print("acc:");
  Serial5.print(accel_x_axis, DEC);
  Serial5.print(",");
  Serial5.print(accel_y_axis, DEC);
  Serial5.print(",");
  Serial5.print(accel_z_axis, DEC);
  Serial5.print("\t");

  Serial5.print("but:");
  Serial5.print(z_button, DEC);
  Serial5.print(",");
  Serial5.print(c_button, DEC);

  Serial5.print("\r\n"); // newline
  i++;
}

// returns zbutton state: 1=pressed, 0=notpressed
static int nunchuck_zbutton()
{
  return ((nunchuck_buf[5] >> 0) & 1) ? 0 : 1; // voodoo
}

// returns zbutton state: 1=pressed, 0=notpressed
static int nunchuck_cbutton()
{
  return ((nunchuck_buf[5] >> 1) & 1) ? 0 : 1; // voodoo
}

// returns value of x-axis joystick
static int nunchuck_joyx()
{
  return nunchuck_buf[0];
}

// returns value of y-axis joystick
static int nunchuck_joyy()
{
  return nunchuck_buf[1];
}

// returns value of x-axis accelerometer
static int nunchuck_accelx()
{
  return nunchuck_buf[2]; // FIXME: this leaves out 2-bits of the data
}

// returns value of y-axis accelerometer
static int nunchuck_accely()
{
  return nunchuck_buf[3]; // FIXME: this leaves out 2-bits of the data
}

// returns value of z-axis accelerometer
static int nunchuck_accelz()
{
  return nunchuck_buf[4]; // FIXME: this leaves out 2-bits of the data
}
