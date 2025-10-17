# Simple Brightness Control System

## Problem statement

Two working modes: **NORMAL** and **BLUE** modes. The **USER button**
switches the working mode.

A **brightness measure** must be taken every **2 seconds** in **NORMAL
mode**. Depending on the read value, the **RGB LED**:
    - turns **RED** if brightness is below 33%.
    - turns **YELLOW** if brightness is between 33 and 66%.
    - turns **GREEN** if brightness is above 66%.

The **RGB LED** may be controlled either using digital or bus outputs.

In **BLUE mode** the **RGB LED** is **BLUE** and **no brightness measures** are taken until back in **NORMAL mode**.

In both modes the information is displayed on the Zephyr terminal.

To control **brightness measure** and **serial connection** with an independent thread (using a new file for it).

If a long press (1 second long) is detected in the **USER button**, the system enters **OFF mode**. In such case, **RGB LED** turns OFF and **serial connection** sends the message “System OFF”.

---

## Overview

This system reads ambient brightness using a **phototransistor** and controls an **RGB LED** based on the measured light. A **user button** allows switching between different modes and turning the system ON/OFF. The system is built using **Zephyr RTOS**, with a separate thread for brightness measurement.

---

## Components

### 1. RGB LED
- Connected via three GPIO pins (Red, Green, Blue).  
- Can display colors like **Red, Yellow, Green, Blue** depending on mode and brightness.  
- Controlled via functions in `rgb_led.c`:
  - `rgb_red()`, `rgb_green()`, `rgb_blue()`, `rgb_yellow()`, etc.  
  - `rgb_led_write()` writes a bitmask to the pins.
- Can be controlled using either digital or bus outputs.

### 2. Phototransistor (Light Sensor)
- Connected to an ADC channel.  
- Measures ambient light voltage.  
- Functions in `adc.c`:
  - `adc_read_voltage()` → returns measured voltage in millivolts.
  - `adc_read_normalized()` → returns normalized brightness (0–1).

### 3. User Button
- Connected via a GPIO pin with interrupt capability.  
- Short press → switches between **NORMAL** and **BLUE** modes.  
- Long press (1 second) → toggles **OFF mode**.  
- Handled via `user_button.c`:
  - `button_init()` → initialize GPIO.
  - `button_set_callback()` → attach ISR.
  - `button_was_pressed()` → check if pressed.

### 4. Brightness Thread
- Runs independently from the main loop.  
- Periodically reads the ADC and converts voltage to **brightness percentage**.  
- Does **not decide mode**; it only updates the shared context with the measured brightness.
- Serially prints brightness information to the Zephyr terminal for debugging.

---

## System Modes

| Mode        | Behavior                                                                 |
|------------|--------------------------------------------------------------------------|
| **OFF**     | RGB LED is off. No brightness measurement. Zephyr terminal prints "System OFF".  |
| **NORMAL**  | RGB LED color depends on brightness: <br> - < 33% → Red <br> - 33–66% → Yellow <br> - > 66% → Green <br> Brightness measurements occur every 2 seconds. |
| **BLUE**    | RGB LED turns blue. No brightness measurements are performed until returning to NORMAL mode. |

---

## System Operation

1. **Initialization**
    - Initialize RGB LED, ADC, and User Button.  
    - Create a **shared context** (`struct system_context`) containing:
        - Current mode (`ctx.mode`)  
        - Last brightness value (`ctx.brightness`)  
        - ADC configuration (`ctx.adc`)  
        - Mutex for thread-safe access (`ctx.lock`)  
    - Start the **brightness thread**, which updates `ctx.brightness` periodically.

2. **User Button Handling (Main Loop)**
    - Short press → toggles **NORMAL ↔ BLUE** mode.  
    - Long press → toggles **OFF ↔ NORMAL** mode.  
    - Uses GPIO reads and timestamps to detect press duration.

3. **LED Control (Main Loop)**
    - Main loop reads:
        - Current system mode (`ctx.mode`)  
        - Last brightness value (`ctx.brightness`)  
    - Sets the RGB LED accordingly:
        - `OFF_MODE` → LED off  
        - `BLUE_MODE` → LED blue  
        - `NORMAL_MODE` → LED color based on brightness
            - < 33% → RED
            - 33–66% → YELLOW
            - > 66% → GREEN

4. **Brightness Measurement (Thread)**
    - Runs every 2 seconds **only if mode is NORMAL**.  
    - Reads ADC voltage → converts to percentage:  
      ```c
      percent = (mv / vref_mv) * 100
      ```
    - Updates `ctx.brightness` with mutex protection.  
    - Prints value to console for debugging.

5. **Serial Output**
    - The brightness thread sends measured brightness values to the Zephyr terminal.  
    - Mode changes (NORMAL, BLUE, OFF) are also printed.

---

## Two Working Modes (Summary)

- **NORMAL mode**: brightness is measured every 2 seconds. LED changes color based on brightness:
    - **RED** if brightness < 33%
    - **YELLOW** if brightness 33–66%
    - **GREEN** if brightness > 66%
- **BLUE mode**: LED is **BLUE**, no brightness measurements are taken until returning to NORMAL.
- **OFF mode** (activated by long press): LED turns off and terminal prints “System OFF”.
- User button switches modes:
    - Short press: NORMAL ↔ BLUE
    - Long press:
		- If in NORMAL or BLUE → OFF
		- If in OFF → NORMAL
- The system initializes all components and starts in OFF mode.

---

## Thread & Main Separation

- **Main thread**:
  - Handles system mode logic and LED control.  
  - Handles user input (button presses).  

- **Brightness thread**:
  - Only reads ADC and updates brightness in context.  
  - Serially prints brightness information.  

This separation ensures:
- Responsive button handling and LED updates.  
- Independent brightness measurement without blocking the main loop.
