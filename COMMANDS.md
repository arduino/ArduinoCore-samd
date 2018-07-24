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

| Command               | Where | Params       | Info                                                    |
|:---------------------:| ----- |:------------:| ------------------------------------------------------- |
| SET_NODE_ID:node_id   | All   | int node_id  | Sets internal node ID to `node_id` (0x0001-0xfffe)      |
| SET_DEST_ID:node_id   | All   | int node_id  | Sets internal dest ID to `node_id` (0x0001-0xfffe)      |
| SET_PAN_ID:pan_id     | All   | int pan_id   | Sets internal PAN ID address to `pan_id` (0x0001-0xfffe)|
| SET_CHANNEL:channel   | All   | int channel  | Sets internal channel to `channel` (0x0b-0x1a)          |
| SET_ENDPOINT:endpoint | All   | int endpoint | Sets internal channel to `endpoint` (0x01-0x0f)         |
| SET_CONFIG            | All   |              | Updates the internal radio config with new settings     |

## Peripheral library commands

| Command                 | Where | Params               | Info                                        |
|:-----------------------:| ----- |:--------------------:| ------------------------------------------- |
| SET_RGB:0x00:0x00:0x00  | All   | int r, int g, int b  | Set RGB LED value. Off is 0x00, On is 0xff. |
| SET_HSV:0x00:0x00:0x00  | All   | int h, int s, int v  | H(0x0-0x167), S(0x0-0x64), V(0x0-0x64).     |
| TEST_RGB                | All   |                      | Cycles LED through red, green, and blue     |
| TEST_HSV                | All   |                      | Cycles LED through all hues at full S and V |