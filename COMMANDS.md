# FemtoCore COMMANDS

**FemtoCore 1.0.0**

Here is a list of available commands on nodes with ENABLE_SERIAL defined (see libraries/FemtoCore/FemtoCore.h)

## How to run commands
Commands issued as-is will run on the node where the command was issued (likely via SerialUSB port).

Prepending a command with ':' will send the command to the currently set destination node address.
Prepending a command with '>' will broadcast the command to all connected nodes on the same PAN ID address.

## FreeIMU library commands

| Command | Where      | Params         | Info                                                           |
|:-------:| ---------- |:--------------:| -------------------------------------------------------------- |
| v       | Coin only  |                | Returns version information about FemtoCore and FreeIMU        |
| 1       | Coin only  |                | Initializes the FreeIMU library                                |
| 2       | Coin only  |                | Resets the FreeIMU library.                                    |
| g       | Coin only  |                | Initialize FreeIMU gyroscope(s).                               |
| t       | Coin only  |                | Set FreeIMU temperature calibration.                           |
| p       | Coin only  | int pressure   | Set the sea level pressure. Example: Set to 100 `p100`         |
| r       | Coin only  | int count      | Read *FreeIMU raw data* `count` times as printable data. `r1`  |
| b       | Coin only  | int count      | Read *FreeIMU raw data* `count` times as binary data. `b1`     |
| q       | Coin only  | int count      | Read *FreeIMU quaternions* `count` times. `q1`                 |
| z       | Coin only  | int count      | Read *FreeIMU quaternions + values* `count` times. `z1`        |
| a       | Coin only  | int count      | Read *FreeIMU kalman filtered quats + vals* `count` times. `a1`|
| C       | Coin only  |                | Read/Check calibration values. `C`                             |
| d       | Coin only  |                | Read FreeIMU debug outputs. `d`                                |
| D       | Coin only  |                | Read *FemtoBeacon data*. `D`                                   |

*FreeIMU raw data* - accel X, accel Y, accel Z, gyro X, gyro Y, gyro Z, mag X, mag Y, mag Z, temperature

*FreeIMU quaternions* (IMU orientation with respect to Earth) - quaternion 1, quaternion 2, quaternion 3, quaternion 4

*FreeIMU quaternion + values* (IMU orientation with respect to Earth). Values are rad/sec. - quaternion 1, quaternion 2, quaternion 3, quaternion 4, accel X, accel Y, accel Z, gyro X, gyro Y, gyro Z, mag X, mag Y, mag Z, temperature (from barometer), pressure, sample frequency, mag heading, estimated altitude, motion detect value.

*FreeIMU kalman filtered quats + vals* (IMU orientation with respect to Earth). Values are rad/sec. Quaternions are Kalman filtered. - quaternion 1, quaternion 2, quaternion 3, quaternion 4, accel X, accel Y, accel Z, gyro X, gyro Y, gyro Z, mag X, mag Y, mag Z, temperature (from barometer), pressure, sample frequency, mag heading, estimated altitude, motion detect value.

*FemtoBeacon data* (YPR is 180deg based. Euler angles are 360deg based.) - current MS, yaw, pitch, roll, euler angle 1, euler angle 2, euler angle 3, accel X, accel Y, accel Z.


## Networking library commands

You must issue a `SET_CONFIG` command after you are done issuing networking library commands.

| Command (comm)        | Where | Params       | Info                                                    |
|:---------------------:| ----- |:------------:| ------------------------------------------------------- |
| SET_REPEAT:flw:comm   | All  | int flow, comm| Repeat send. flw 0x00 = off, 0x01 = on. Comm is optional|
| SET_NODE_ID:node_id   | All   | int node_id  | Sets internal node ID to `node_id` (0x0001-0xfffe)      |
| SET_DEST_ID:node_id   | All   | int node_id  | Sets internal dest ID to `node_id` (0x0001-0xfffe)      |
| SET_PAN_ID:pan_id     | All   | int pan_id   | Sets internal PAN ID address to `pan_id` (0x0001-0xfffe)|
| SET_CHANNEL:channel   | All   | int channel  | Sets internal channel to `channel` (0x0b-0x1a)          |
| SET_ENDPOINT:endpoint | All   | int endpoint | Sets internal channel to `endpoint` (0x01-0x0f)         |
| SET_ANTENNA:antenna   | All   | int antenna  | Set to 0x01 for SMD chip, 0x02 for u.Fl. (0x01-0x02)    |
| GET_ANTENNA           | All   |              | Gets the antenna config setting.                        |
| SET_TX_POWER:power    | All   | int power    | Sets the *Transmission Power Level*. (See below)        |
| GET_TX_POWER          | All   |              | Gets the *Transmission Power Level*. (See below)        |
| SET_CONFIG            | All   |              | Updates the internal radio config with new settings     |

### Transmission Power Level

Set TX Power for internal at86rf233 RF module. Default is 0x0 (+4 dbm)

 * TX_PWR  0x0 ( +4   dBm)
 * TX_PWR  0x1 ( +3.7 dBm)
 * TX_PWR  0x2 ( +3.4 dBm)
 * TX_PWR  0x3 ( +3   dBm)
 * TX_PWR  0x4 ( +2.5 dBm)
 * TX_PWR  0x5 ( +2   dBm)
 * TX_PWR  0x6 ( +1   dBm)
 * TX_PWR  0x7 (  0   dBm)
 * TX_PWR  0x8 ( -1   dBm)
 * TX_PWR  0x9 ( -2   dBm)
 * TX_PWR  0xA ( -3   dBm)
 * TX_PWR  0xB ( -4   dBm)
 * TX_PWR  0xC ( -6   dBm)
 * TX_PWR  0xD ( -8   dBm)
 * TX_PWR  0xE (-12   dBm)
 * TX_PwR  0xF (-17   dBm)

## Peripheral library commands - RGB LED

| Command                 | Where | Params               | Info                                        |
|:-----------------------:| ----- |:--------------------:| ------------------------------------------- |
| SET_RGB:0x00:0x00:0x00  | All   | int r, int g, int b  | Set RGB LED value. Off is 0x00, On is 0xff. |
| SET_HSV:0x000:0x00:0x00 | All   | int h, int s, int v  | H(0x000-0x167), S(0x00-0x64), V(0x00-0x64). |
| TEST_RGB                | All   |                      | Cycles LED through red, green, and blue     |
| TEST_HSV                | All   |                      | Cycles LED through all hues at full S and V |


## Peripheral library commands - Real Time Clock (RTC)

You will need to call `START_RTC` before using any of these commands. 

| Command                                  | Where | Params                 | Info                  |
|:----------------------------------------:| ----- |:----------------------:| --------------------- |
| START_RTC                                | All   | *                      | Enables RTC clock     |
| SET_CLOCK:0x000:0x00:0x00-0x00:0x00:0x00 | All   | *SET_CLOCK params*     | Set RTC 24hr clock    |
| GET_CLOCK                                | All   |                        | Gets RTC clock        |
| SET_SLEEP_MS:0x00000001                  | All   | int sleep_time_ms      | How long to sleep (ms)|
| GET_SLEEP_MS                             | All   |                        | Get sleep (ms) value  |

*START_RTC NOTE* KNOWN ISSUE: After starting RTC clock, USB sketch upload fails.
WORK AROUND: Either short the RESET pads for 3 seconds (goes into bootloader mode), or power cycle your board before you upload a sketch.

*SET_CLOCK params* Year (0x000), Month (0x01-0x0c), Day (0x01-0x1F) - Hour (0x00-0x18), Minutes (0x00-0x3c), Seconds (0x00-0x3c)

Note: *SET_SLEEP_MS* min value is 1, max value is 2,147,483,647 milliseconds (0x7FFFFFFF), as that's as high as `int` variables on this micro controller go.