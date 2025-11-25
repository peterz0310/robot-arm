# Robot Arm Firmware (ESP32)

ESP32 firmware for controlling a 6-axis 3D printed robotic arm over WebSocket. Receives joint angle commands from a mobile app and drives hobby servos with slew rate limiting for smooth motion.

## Credits

This project is based on the excellent mechanical design by **Emre Kalem**:

🔗 **Original Design:** [Robotic Arm with Servo Arduino on MakerWorld](https://makerworld.com/en/models/1134925-robotic-arm-with-servo-arduino)

The original design used an Arduino with a different control scheme. This firmware is a complete rewrite for the ESP32 platform with WebSocket-based control, enabling wireless operation from a mobile app.

## Related Repository

📱 **Mobile App:** [robot-arm-app](https://github.com/peterz0310/robot-arm-app) — Expo/React Native app for controlling the arm from your phone.

## Features

- **WebSocket server** for real-time bidirectional communication
- **Slew rate limiting** — Smooth motion with configurable max degrees/step (prevents jerky movements)
- **Per-joint clamping** — Safety limits for each servo
- **Program playback** — Execute waypoint sequences with linear interpolation
- **Lead-in waypoints** — Automatically interpolates from current position to first waypoint
- **Emergency stop** — Immediately halts all motion

## Hardware

- **MCU:** ESP32 (tested on ESP32-WROOM-32)
- **Servos:** 7x hobby servos (6 joints, with Arm A using 2 mirrored servos)
- **Power:** External 5V supply for servos (do not power from ESP32)

### GPIO Pin Mapping

| Joint   | GPIO | Notes                   |
| ------- | ---- | ----------------------- |
| Base    | 4    |                         |
| Arm A1  | 5    |                         |
| Arm A2  | 12   | Mirrored (180° - angle) |
| Arm B   | 13   |                         |
| Wrist A | 14   |                         |
| Wrist B | 18   |                         |
| Gripper | 19   |                         |

## Configuration

Edit `src/main.cpp` to configure:

```cpp
// WiFi credentials
constexpr char WIFI_SSID[] = "YourNetwork";
constexpr char WIFI_PASSWORD[] = "YourPassword";

// Joint limits (adjust for your build)
constexpr float HOME_ANGLES[6] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
constexpr float MIN_ANGLES[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
constexpr float MAX_ANGLES[6] = {180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f};

// Motion control
constexpr float MAX_DEG_PER_STEP = 10.0f;      // Max degrees per 20ms tick
constexpr unsigned long SLEW_INTERVAL_MS = 20; // Update rate
```

## Building & Flashing

This is a PlatformIO project:

```bash
# Build
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor -b 115200
```

## WebSocket Protocol

The ESP32 hosts a WebSocket server at `ws://<IP>:80/arm`.

### Manual Joint Control

```json
{
  "base": 90,
  "armA": 45,
  "armB": 120,
  "wristA": 90,
  "wristB": 90,
  "gripper": 60
}
```

### Program Execution

```json
{
  "type": "program",
  "program": {
    "waypoints": [
      { "t": 0, "joints": { "base": 90, "armA": 90, ... } },
      { "t": 2000, "joints": { "base": 45, "armA": 120, ... } }
    ]
  }
}
```

### Emergency Stop

```json
{ "emergencyStop": true }
```

## Important: Home Position

**Always power on the arm with joints at their home positions.** Hobby servos don't provide position feedback, so the firmware assumes the arm starts at home. If powered on in a different position, the arm will immediately try to move to home angles.

## Dependencies

Managed by PlatformIO (see `platformio.ini`):

- ESP32Servo
- ArduinoJson
- ESPAsyncWebServer
- AsyncTCP

## License

MIT — See original mechanical design license on MakerWorld.
