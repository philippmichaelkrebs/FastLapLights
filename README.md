# FastLapLights

**"Gentlemen, start your engines!"** 🏁  

Bring FIA-style lighting to your slot car track with this high-tech lighting system! Featuring WS2812B Start & Pit Lane Lights, RGB Track Flags, and I2C Data Handling! 

Following Models are supported:
- Carrera Digital 124
- Carrera Digital 132

Preview:  
![](https://github.com/philippmichaelkrebs/FastLapLights/blob/main/images/signal_start_proc_start_light.gif?raw=true)


# Features

- Start Lights: Perfect race countdown sequence with WS2812B LEDs 🚦
- Pit Lane Lights: Manage pit stops like a real motorsport pro
- Track Flag System: RGB LEDs indicate race conditions (Green, Yellow, Red, Blue) 🚥
- Real-Time Data Capture: I2C interface collects race data via track interrupts 
- Customizable & Expandable: Tweak lights, colors, and behavior for your track setup! 

# Get Started

- Order the microcontroller from jlcpcb
- Flash the firmware on your STM32 microcontroller
- Connect WS2812B & RGB LEDs according to the purpose.
- Plug in I2C Sensors for track data (optional but cool 😎).
- Solder power wires to the track
- Power up & lights out!


# How It Works

- Start Lights: Uses WS2812B LEDs to signal the race start countdown.
- Pit Lane Signals: Indicate pit status, access control, and penalties.
- Track Flags: RGB LEDs for race status (Green: go, Yellow: caution, Red: stop).
- I2C Data Handling: Reads digital track data from interrupts, perfect for lap timing & automation.

# CU Data
The data that is transmitted by the cu is an asynchronous serial protocol. The chunks are of various size. We got no stop bit. But fortunetly the messages are of different size by at least 2 bits. Accordingly to this behaviour it's sufficient to listen only to the falling edges. If the message size is between two known message types, we ceil the size by adding a 0 to the end of the message.

## Content
We don't listen to all different message types. Let's choose a few messages wisely :)
So we got:
1. Reset of positions and round counter. Occurs at pressing start for first time. (hitting start for the second time starts the countdown)
2. Car with ID finishes the race
3. Car with ID crosses the lap and makes a new fastest lap
4. Car with ID crosses the lap
5. Car with ID jumps the start
6. Fuel level of car with ID
7. Accelerator pedal position of car with ID
8. Number of light were lit on the starting lights. If the lights out, the race begin
9. Total reset

## Documentation
- [How the Lights Work](../docs/how_the_lights_work.md)
- [Getting Started](../docs/getting_started.md)

# Contribute & Support

Got ideas? Found a bug? Want to make it even cooler? 
Open an issue, submit a PR, or just say hi!

Developer: Philipp Krebs
Contact: pmge.krebs@gmail.com
Repo: https://github.com/philippmichaelkrebs
