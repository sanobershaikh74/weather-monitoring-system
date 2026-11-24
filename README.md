# Task-Based Weather Monitoring Station

An ESP32-based environmental monitoring project developed for **BTEC13701 – Embedded System Design**. The application uses ESP-IDF and FreeRTOS tasks to collect temperature, humidity, and gas-sensor readings, display them on an SSD1306 OLED, and upload readings to ThingSpeak over Wi-Fi.

## Features

- DHT11 temperature and relative-humidity sampling.
- MQ-2 and MQ-135 analog sensor sampling through ESP32 ADC1.
- SSD1306 OLED output over I²C.
- Independent FreeRTOS tasks for sensor acquisition, display refresh, and cloud upload.
- Mutex-protected shared sensor data.
- Wi-Fi event handling and reconnection attempt.
- ThingSpeak HTTP updates at a configurable interval.
- Serial logging for sensor readings and task status.

> **Measurement limitation:** MQ-2 and MQ-135 values are raw ADC counts. They are not calibrated gas concentrations, ppm values, or certified air-quality measurements. The project is an educational prototype, not a safety alarm or certified environmental instrument.

## Hardware and pin mapping

| Module / signal | ESP32 pin | Notes |
|---|---:|---|
| DHT11 DATA | GPIO 4 | Use a suitable pull-up resistor (commonly 4.7–10 kΩ) |
| MQ-2 analog output | GPIO 34 / ADC1 channel 6 | Scale voltage so it never exceeds 3.3 V |
| MQ-135 analog output | GPIO 35 / ADC1 channel 7 | Scale voltage so it never exceeds 3.3 V |
| SSD1306 SDA | GPIO 21 | I²C |
| SSD1306 SCL | GPIO 22 | I²C |
| OLED I²C address | 0x3C | Confirm your module address |

<img width="804" height="557" alt="image" src="https://github.com/user-attachments/assets/1f339cd7-d578-4121-bdea-0acdde26c2fa" />


### Electrical safety

- **Never apply more than 3.3 V to an ESP32 GPIO or ADC input.**
- Many MQ sensor breakout boards are powered at 5 V and may produce analog outputs above 3.3 V. Use a correctly designed voltage divider or level-conditioning circuit and verify the voltage with a meter before connecting it to the ESP32.
- Check the sensor module's pinout and power requirements before wiring.
- MQ sensors require warm-up and calibration for meaningful measurements.

## Software requirements

- ESP-IDF 5.x (the project manifest specifies ESP-IDF 5.0 or newer).
- A supported ESP32 target (the pin assumptions are for a classic ESP32).
- USB/UART drivers and the ESP-IDF command-line environment.
- A ThingSpeak channel with fields configured as:
  - Field 1: Temperature (°C)
  - Field 2: Relative humidity (%)
  - Field 3: MQ-2 raw ADC count
  - Field 4: MQ-135 raw ADC count

The code uses ESP-IDF APIs and may need small compatibility adjustments for a particular ESP-IDF 6.x release. Verify against the exact version installed on your machine.

## Configure credentials

1. Copy `main/secrets.example.h` to `main/secrets.h`.
2. Edit `main/secrets.h` with your own Wi-Fi SSID, Wi-Fi password, and ThingSpeak Write API key.
3. Keep `main/secrets.h` local. It is ignored by Git.

Never publish Wi-Fi passwords or ThingSpeak write keys. If a real key or password has already been committed or shared publicly, rotate it.

## Build and flash

From the project root, with ESP-IDF exported in your terminal:

```bash
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p PORT flash monitor
```

Replace `PORT` with your serial port (for example, `COM5` on Windows or `/dev/ttyUSB0` on Linux).

## FreeRTOS task design

| Task | Priority | Intended period | Responsibility |
|---|---:|---:|---|
| `dht_task` | 5 | 2 seconds | Read DHT11 and update temperature/humidity |
| `display_task` | 4 | 1 second | Refresh OLED using a consistent data snapshot |
| `gas_task` | 3 | 1 second | Read MQ-2 and MQ-135 ADC channels |
| `thingspeak_task` | 2 | 20 seconds | Upload the latest snapshot when Wi-Fi is connected |

A FreeRTOS mutex protects the shared reading structure. An event group signals Wi-Fi connectivity.

## ThingSpeak channel setup

Create a ThingSpeak channel with four fields matching the mapping above. Copy the channel's **Write API Key** into your local `main/secrets.h`. The upload period defaults to 20 seconds; ThingSpeak free accounts generally enforce a minimum update interval, so check the current limits of your account.

## Project structure

```text
Task-Based-Weather-Monitoring-Station/
├── CMakeLists.txt
├── README.md
├── LICENSE
├── .gitignore
└── main/
    ├── CMakeLists.txt
    ├── idf_component.yml
    ├── main.c
    ├── secrets.example.h
    └── secrets.h             # local only; create yourself
```

## Libraries and ESP-IDF components

- **ESP-IDF:** official development framework for ESP32.
- **FreeRTOS:** task scheduling, mutexes, and event groups.
- **GPIO driver:** DHT11 signal pin configuration.
- **ADC oneshot driver:** MQ sensor analog acquisition.
- **I²C driver:** SSD1306 OLED communication.
- **ESP Timer / ROM delay:** microsecond timing utilities.
- **Wi-Fi, ESP Event, and ESP Netif:** station-mode network connection and event handling.
- **ESP HTTP Client:** ThingSpeak HTTP requests.
- **NVS:** non-volatile storage initialization required by the Wi-Fi stack.
- **ESP logging:** diagnostics in the serial monitor.
- **C standard library:** formatted output, memory operations, and fixed-width types.

The supplied design uses a custom DHT11 GPIO bit-banging implementation and a minimal SSD1306 driver. The `esp-idf-lib/dht` component was mentioned in the original project notes but is not required by this implementation.

## Academic details

- **Subject code:** BTEC13701
- **Subject name:** Embedded System Design
- **Project:** Task-Based Weather Monitoring Station

Update or remove personal academic details before making the repository public.

## Limitations and future improvements

- Replace the custom DHT11 bit-banging routine with a maintained sensor driver or RMT-based implementation for more robust timing.
- Add ADC calibration and sensor-specific calibration curves before interpreting MQ sensor output.
- Use HTTPS where supported and protect API credentials.
- Add retry/backoff and response-body validation for cloud uploads.
- Add a watchdog-safe design and explicit handling for task-creation failures.
- Consider a dedicated OLED library and a richer font if more display text is needed.
