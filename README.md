## How to use MAVLink on Arduino ESP32 with PlatformIO

![MAVLink for Arduino ESP32](/How-to-use-MAVLink-on-ESP32.png)

## Configuration and Installation

Change the `creds.h` file with your WiFi credentials.

```
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASS "YOUR_WIFI_PASS"
```
It's recommanded to use a static IP address. You can change it in the `conf.h` file. Don't forget to set your Groundstation IP address.
```
#define LOCAL_IP 192, 168, 23, 23
#define SUBNET 255, 255, 255, 255
#define GATEWAY 192, 168, 23, 1
#define LOCAL_PORT 14500

#define GROUNGSTATION_IP "192.168.23.2"
#define GROUNDSTATION_PORT 14550
```
And change the status LED pins to your need.

```
#define LED_GREEN 16
#define LED_RED 17
```
## QGroundControl

After successfully flashing your ESP32 with the firmware, the serial monitor should output the WiFi connection status.

Open `QGroundControl` and go to `Application Settings` tick `SiK Radio` (Serial) and/or `UDP` under _AutoConnect the following devices_. QGroundControl should automatically connect to your ESP32 and show its position on the map. Optionally you can enable `Virtual Joystick` under _Fly View_.

You can also set connection type and settings manually under `Comm Links`.

Use `MAVLink Inspector` under `Analyze Tools` to get an insight into the MAVLink traffic between your ESP32 and Groundstation.

![ESP32 MAVLink QGroundControl](/ESP32-MAVLink-QGroundControl.jpg)

## Sources and Inspiration

- https://discuss.ardupilot.org/t/mavlink-step-by-step/9629
- https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566
- https://github.com/tmaxxdd/arduino-with-mavlink
- https://github.com/patrickelectric/minimal-arduino-mavlink-example
- https://github.com/danzimmerman/barebones_MAVLink
- https://github.com/pschatzmann/ArduinoMavlinkDrone

## More
Here is a list of projects that uses this sourcecode as a base (coming soon):

- ESP32 MAVLink Rover
- MAVLink Standalone Hardware Groundstation
- Pelco-D to MAVLink

---

✍️ Blog: https://www.technopolis.tv/How-to-use-MAVLink-on-ESP32/