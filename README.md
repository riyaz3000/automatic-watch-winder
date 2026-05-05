# 3-Watch Automatic Watch Winder Firmware

ESP32 firmware for three independent 28BYJ-48 watch winders with an SSD1315/SSD1306-compatible OLED, four direct GPIO buttons, WiFi dashboard, WiFi setup, and persistent settings in ESP32 Preferences.

## Build

This project is set up for PlatformIO:

```bash
pio run
pio run -t upload
pio device monitor
```

## Default Pin Map

OLED I2C:

- SDA: GPIO 21
- SCL: GPIO 22
- Address: `0x3C`

Buttons:

- K1 Up / Next: GPIO 34
- K2 Down / Previous: GPIO 35
- K3 Select / Confirm: GPIO 36
- K4 Back / WiFi setup: GPIO 39

GPIO 34-39 are input-only and do not have internal pull-ups. Use 10k pull-ups to 3.3V unless the OLED board already provides stable active-LOW button outputs.

Motors via ULN2003:

- Winder 1: GPIO 13, 14, 16, 17
- Winder 2: GPIO 18, 19, 23, 25
- Winder 3: GPIO 26, 27, 32, 33

The firmware passes pins to AccelStepper in HALF4WIRE order as `IN1, IN3, IN2, IN4`, which is why each motor row appears reordered in code.

Optional physical enable switches are supported in `PHYSICAL_ENABLE_SWITCH_PINS` in [src/main.cpp](src/main.cpp). They are disabled by default with `-1`; when fitted, each switch is active LOW and uses the ESP32 internal pull-up.

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

If saved WiFi is missing or connection fails, the ESP32 starts `WatchWinder-Setup` access point mode. The web dashboard is available at the shown IP address.

## Motor Timing

Each enabled winder runs a short non-blocking burst every 30 seconds. Firmware converts TPD to steps per burst using `4096` steps per full 28BYJ-48 output shaft rotation and carries fractional steps forward so low and odd TPD values average correctly over the day.
