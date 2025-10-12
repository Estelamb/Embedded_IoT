# Blink Example modified
This is the modified version of the Blink example.

## RGB LED
1. Modify the overlay file to include the selected pin as a digital output:

In confs/prj_nucleo_wl55jc.conf, add:
```
CONFIG_GPIO=y # Enable GPIO
```

In boards/nucleo_wl55jc.overlay, add:
```

```

2. Design the electrical circuit required to drive the RGB LED (each of its three colors), considering the polarity of the common terminal:


  |      |      |
100 Ω   100 Ω  100 Ω
  |      |      |
 RED   GREEN   BLUE
  |      |      |
 GND    GND    GND



3. Incorporate the corresponding modifications into the previous code (Main.c). It is recommended to distribute the code into separate libraries:

In src/sensors, add the rgb_led folder with rgb_led.c and rgb_led.h files.

In CMakeLists.txt, add:
```
target_sources(app PRIVATE 
    src/main.c
    src/sensors/rgb_led/rgb_led.c
)

target_include_directories(app PRIVATE
    src/sensors/rgb_led
)
```

Develop the rgb_led.c and rgb_led.h files to manage the RGB LED functionality and modify main.c to utilize these functions.


## Phototransistor
1. Modify the overlay file to include the selected pin as a digital output.
In confs/prj_nucleo_wl55jc.conf, add:
```
CONFIG_ADC=y # Enable ADC
```

In boards/nucleo_wl55jc.overlay, add:
```

```




2. Design the electrical circuit required to drive the phototransistor, considering the polarity of the terminals.

  Vcc = 5V
      |
Phototransistor
      |
     Vout ---------> ADC pin
      |
     1 kΩ
      |
     GND


3. Incorporate the corresponding modifications into the previous code (Main.c). It is recommended to distribute the code into separate libraries.

In src/sensors, add the phototransistor folder with phototransistor.c and phototransistor.h files.

In CMakeLists.txt, add:
```
target_sources(app PRIVATE 
    src/main.c
    src/sensors/rgb_led/rgb_led.c
    src/sensors/phototransistor/phototransistor.c
)

target_include_directories(app PRIVATE
    src/sensors/rgb_led
    src/sensors/phototransistor
)
```

Develop the phototransistor.c and phototransistor.h files to manage the phototransistor functionality and modify main.c to utilize these functions.