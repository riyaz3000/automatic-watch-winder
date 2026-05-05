# 3-Watch Automatic Watch Winder Firmware

ESP32 firmware for three independent 28BYJ-48 watch winders with an SSD1315/SSD1306-compatible OLED, four direct GPIO buttons, WiFi dashboard, WiFi setup, and persistent settings in ESP32 Preferences.

## Build

This project is set up for PlatformIO:

```bash
pio run
pio run -t upload
pio device monitor
```

## ESP32-DevKitC-32E Pin Map

This pin map is for the ESP32-DevKitC / ESP32-DevKitC-32E style board from the linked Amazon listing. It uses the header labels printed in Espressif's DevKitC documentation where possible.

OLED I2C:

- SDA: `IO21` / GPIO 21
- SCL: `IO22` / GPIO 22
- Address: `0x3C`

Buttons:

- K1 Up / Next: `VP` / GPIO 36
- K2 Down / Previous: `VN` / GPIO 39
- K3 Select / Confirm: `IO34` / GPIO 34
- K4 Back / WiFi setup: `IO35` / GPIO 35

GPIO 34, 35, 36, and 39 are input-only and do not have internal pull-ups. Use 10k pull-ups to 3.3V unless the OLED board already provides stable active-LOW button outputs.

Motors via ULN2003:

| Winder | ULN2003 IN1 | ULN2003 IN2 | ULN2003 IN3 | ULN2003 IN4 |
| --- | --- | --- | --- | --- |
| 1 | `IO13` / GPIO 13 | `IO14` / GPIO 14 | `IO27` / GPIO 27 | `IO26` / GPIO 26 |
| 2 | `IO25` / GPIO 25 | `IO32` / GPIO 32 | `IO33` / GPIO 33 | `IO23` / GPIO 23 |
| 3 | `IO19` / GPIO 19 | `IO17` / GPIO 17 | `IO18` / GPIO 18 | `IO16` / GPIO 16 |

The firmware passes pins to AccelStepper in HALF4WIRE constructor order as `IN1, IN3, IN2, IN4`, so the code array is intentionally ordered differently from the ULN2003 table above.

Do not use GPIO 6-11. Espressif marks those pins as connected to the module's SPI flash interface.

Optional physical enable switches are supported in `PHYSICAL_ENABLE_SWITCH_PINS` in [src/main.cpp](src/main.cpp). They are disabled by default with `-1`; when fitted, each switch is active LOW and uses the ESP32 internal pull-up.

Power:

- Power the ESP32 from USB or the board 5V input, but not both external board power methods at the same time.
- Power the motors from the external 5V supply through the ULN2003 boards.
- Connect ESP32 GND, ULN2003 GND, and motor power supply GND together.

## Operation

- K3 opens the OLED menu from the status screen.
- K1/K2 scroll or adjust values.
- K3 selects, toggles enable, cycles direction, or saves TPD edits.
- K4 goes back.
- Hold K4 from the status screen to start WiFi scanning.

WiFi password entry uses the four buttons:

- K1/K2 changes the current character.
- K3 appends the current character.
- Hold K3 saves and connects.
- K4 deletes the last character, or returns to network selection if empty.

## WiFi Setup Hotspot

If saved WiFi is missing or connection fails, the ESP32 starts an open setup hotspot:

- SSID: `WatchWinder-Setup`
- Setup address: `http://192.168.4.1/`

Connect your phone to `WatchWinder-Setup`. Most phones will open the setup page automatically; if not, open `http://192.168.4.1/` in a browser. Enter your home WiFi SSID and password, then save.

While joining your home WiFi, the setup hotspot stays active long enough for the form submission to complete. After the ESP32 connects successfully, it shuts down the setup hotspot and shows the home-network IP address on the OLED status screen and menu. Use that IP address from any device on the same WiFi network to adjust winder settings in the web GUI.

The web GUI remains available in both modes:

- Setup hotspot mode: use `http://192.168.4.1/`
- Home WiFi mode: use the IP address shown on the OLED

## Motor Timing

Each enabled winder runs a short non-blocking burst every 30 seconds. Firmware converts TPD to steps per burst using `4096` steps per full 28BYJ-48 output shaft rotation and carries fractional steps forward so low and odd TPD values average correctly over the day.
