# Plant Monitoring System

## Overview

This project implements a **Plant Monitoring System** using the Zephyr RTOS. It integrates multiple sensors and actuators to monitor environmental conditions, log measurements, and provide visual feedback. The system is modular, thread-safe, and supports different operating modes.

### Key Features

- Periodic acquisition of environmental sensor data:
  - Ambient light (phototransistor)
  - Soil moisture
  - Accelerometer (XYZ axes)
  - Temperature & humidity (Si7021)
  - Color detection (TCS34725)
  - GPS positioning
- Multi-threaded architecture using Zephyr threads:
  - **Sensors thread**: acquires ADC and I2C sensor data
  - **GPS thread**: reads GPS data periodically or on demand
- Thread-safe shared structures using atomic variables
- Mode-dependent behavior:
  - **TEST_MODE**: fast sampling, RGB LED shows dominant color
  - **NORMAL_MODE**: standard periodic sampling with alerts
  - **ADVANCED_MODE**: minimal visual feedback, threads idle until triggered
- Synchronization using semaphores and timers
- Scaled integer representation of sensor values for atomic storage

---

## Architecture

The system is organized as a **multi-threaded embedded application** running on Zephyr RTOS, with clear separation between data acquisition, processing, and main control.  

### Threads

- **Main Thread**  
  - Responsible for system initialization and mode management.  
  - Triggers sensor and GPS measurements via semaphores.  
  - Reads the latest measurements from the shared `system_measurement` structure.

- **Sensors Thread**  
  - Periodically reads ADC and I²C sensors according to the system mode.  
  - Sensors include:  
    - Phototransistor (ambient brightness)  
    - Soil moisture  
    - Accelerometer (XYZ axes)  
    - Temperature and humidity (Si7021)  
    - Color sensor (TCS34725)  
  - Uses a timer semaphore and a manual trigger semaphore.  
  - Stores readings atomically in `system_measurement`.

- **GPS Thread**  
  - Periodically reads GPS data in TEST_MODE and NORMAL_MODE.  
  - In ADVANCED_MODE, waits for a manual semaphore trigger.  
  - Parses NMEA sentences and updates `system_measurement` with scaled integer values.  
  - Uses a timer semaphore and a manual trigger semaphore.

### Shared Structures

- **`system_context`**  
  - Contains configuration pointers for all sensors and GPS.  
  - Stores semaphores for thread synchronization.  
  - Holds the current operating mode (`system_mode_t`) as an atomic variable.

- **`system_measurement`**  
  - Stores the latest measurements from all sensors.  
  - Values are stored as atomic variables for safe access across threads.  
  - Includes ADC percentages, accelerometer readings, temperature & humidity, color sensor values, and GPS coordinates.

### Synchronization

- **Timers** control measurement cadence depending on the current mode.  
- **Semaphores** allow threads to signal completion or trigger immediate measurement:  
  - `main_sensors_sem` and `main_gps_sem`: signal the main thread when new data is ready.  
  - `sensors_sem` and `gps_sem`: manually trigger sensor/GPS threads.  
- **Atomic variables** ensure safe concurrent access to shared measurements.


---

## Operating Modes

| Mode           | Description                                                                 |
|----------------|-----------------------------------------------------------------------------|
| **TEST_MODE**   | Fast sensor updates, RGB LED shows dominant color                            |
| **NORMAL_MODE** | Periodic measurement, alerts on out-of-range values                          |
| **ADVANCED_MODE** | Minimal visual feedback, threads idle until re-triggered                  |

Mode is stored atomically and can be changed at runtime.

---

## Sensor Interfaces

### ADC Sensors

- Phototransistor (ambient brightness)
- Soil moisture
- Values are read via `adc_read_voltage()` and scaled to 0–100%

### I2C Sensors

#### Accelerometer (e.g., MMA8451)

- Registers read via I2C
- Supports full-scale ranges: 2G, 4G, 8G
- Functions:
  - `accel_init()`
  - `accel_read_xyz()`
  - `accel_convert_to_g()`
  - `accel_convert_to_ms2()`

#### Temperature & Humidity (Si7021)

- Provides %RH and °C
- Functions:
  - `temp_hum_init()`
  - `temp_hum_read_humidity()`
  - `temp_hum_read_temperature()`

#### Color Sensor (TCS34725)

- Reads raw RGB and clear channel
- Supports gain and integration time configuration
- Functions:
  - `color_init()`
  - `color_wake_up()`
  - `color_sleep()`
  - `color_set_gain()`
  - `color_set_integration()`
  - `color_read_rgb()`

---

## GPS Interface

- GPS data parsed from NMEA GGA sentences
- Fields stored in `system_measurement`:
  - Latitude / Longitude (degrees ×1e6)
  - Altitude (meters ×100)
  - Satellites in view
  - UTC time encoded as HHMMSS integer
- Thread uses a periodic timer in TEST/NORMAL mode, and waits for semaphore in ADVANCED mode
- Functions:
  - `start_gps_thread()`
  - `gps_wait_for_gga()`

---

## Thread Design

### Sensors Thread

- **Stack size**: 1024 bytes  
- **Priority**: 5  
- Periodically reads:
  - ADC sensors (brightness & moisture)
  - Accelerometer (XYZ)
  - Temperature & humidity
  - Color sensor (RGB + clear)
- Synchronization:
  - Uses a timer semaphore
  - Polls for timer expiration or manual trigger
- Stores data atomically for thread-safe access

### GPS Thread

- **Stack size**: 1024 bytes  
- **Priority**: 5  
- Periodically reads GPS data in TEST/NORMAL modes
- In ADVANCED mode, waits for external trigger semaphore
- Uses a timer for periodic updates
- Converts GPS floating point values to scaled integers for atomic storage

---

## Synchronization

- **Semaphores**:
  - `main_sensors_sem`: main thread waits for sensor update
  - `main_gps_sem`: main thread waits for GPS update
  - `sensors_sem`: manual trigger for sensor thread
  - `gps_sem`: manual trigger for GPS thread
- **Timers**:
  - Control measurement cadence depending on operating mode
- **Atomic variables**:
  - All sensor measurements stored atomically for thread-safe reads/writes

---

## Usage

1. Initialize sensors and GPS:

```c
system_context ctx;
system_measurement measure;

start_sensors_thread(&ctx, &measure);
start_gps_thread(&ctx, &measure);
```


2. Main loop waits for semaphore events and reads shared measurements:

```c
k_sem_take(&ctx.main_sensors_sem, K_FOREVER);
int brightness = atomic_get(&measure.brightness);
```

3. Switch system modes by updating ctx.mode atomically:

```c
atomic_set(&ctx.mode, NORMAL_MODE);
```

## Notes
- All sensor readings are scaled appropriately to allow atomic storage.
- I2C devices are checked for readiness before communication.
- Accelerometer and color sensors are set to standby or active modes as needed.
- GPS UTC time is stored as an integer HHMMSS for atomic safety.
- RGB LED can be used to display dominant color in TEST_MODE.
- ADC readings are converted to percentages ×10 for precision.

## Conclusion
This Plant Monitoring System is a modular, multi-threaded system designed for embedded platforms using Zephyr. It integrates multiple sensors, synchronizes data safely between threads, and supports different operating modes with adaptive behavior.