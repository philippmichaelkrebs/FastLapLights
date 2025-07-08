# FastLapLights

**"Gentlemen, start your engines!"** 🏁  

Bring FIA-style lighting to your slot car track with FastLapLights - a high-tech lighting and telemetry system for Carrera Digital tracks! 
Control WS2812b Start Lights, Pit Lane Lights and Track Flags with stunning LED animations that bring your races to life. Want more? FastLapLights als taps into the real-time data stream from the Carrera Control Unit, filters the most relevant messages, and outputs the via UART - perfect for powering external displays, mechanical 7-segment scoring pylons, or even fuel and pit-stop management systems. 
With support for 3.3V and 5V output, you've got everything you need to bring custom hardware into the mix.
Whether you're racing for fun or building the ultimate home track, FastLapLights adds a whole new level of immersion, control and creativity.

Following Models are supported:
- Carrera Digital 124
- Carrera Digital 132

Preview:  
![](https://github.com/philippmichaelkrebs/FastLapLights/blob/main/images/signal_start_proc_start_light.gif?raw=true)


# Features

## Start Lights
Perfect race countdown sequence with WS2812B LEDs 🚦

You can switch between horizontal and vertical alignment for the start lights WS2812B led strips. Vertical alignment requires more soldering but looks more realistic, giving you the option to physically seperate the  lights - just like in real motorsport.
The system is designed to handle 40 leds. So both the front and the back of the lights illuminated. You nearly have a 360 degree visibility of the start lights.

## Pit Lane Lights
Manage pit stops like a real motorsport pro!

Instead of start loghts, you can choose to illuminate pit lane lights. This is an either-or option - controlled via a solder bridge on the back of the pcb. Simpy close the appropriate bridge to switch the functionality.

## Track Flag System
RGB LEDs indicate race conditions (Green, Yellow, Red) 🚥

With a red and green led, you can animate track lights. It's also possible to use a rgb led with r and g connected only. Race status indicated by flags:
- Green: go, 
- Yellow: caution, 
- Red: stop
If you plan to use multiple RGB LEDs, you should use a mosfet to avoid overloading the uCs GPIOs.

## Real-Time Data Capture
Tap into live control unit telemetry!

The STM32 continuously processes data from the Carrera Control Unit. A filtered stream of the most relevant messages is available via the UART interface. This gives you a powerful foundation to build custom applications and add unique behavior to your racetrack setup. 
Whether it's a mechanical 7-segment scoring pylon or a smart pit stop prediction system, FastLapLights gives your cicuit the edge it deserves.

# Get Started

- Order the microcontroller from jlcpcb (see section documentation)
- Flash the firmware on your STM32 microcontroller (see section documentation)
- Connect WS2812B & RGB LEDs according to the purpose.
- Solder power wires to the track
- Power up & lights out!


# Pinouts
![](https://github.com/philippmichaelkrebs/FastLapLights/blob/main/images/pcb_pinout_lq.jpg?raw=true)

# UART
The data that is transmitted by the Control Unit is an asynchronous serial protocol. The chunks are of various size. Notably, the protocol doesn't use a traditional stop bit. The cu transmits data in a structured, continouos stream every 75 milli secs. Each transmission cycle contains 10 distinct messages sent in a strict and predictable sequence.

The controller processes this incomimg data and outputs it via UART, using a **32-byte single-buffered TX DMA system**. Each UART transmission contains a snapshot of the current race state. TX only, RX is disabled.

## UART Configuration
- Baud rate: 115200
- Data format: 8 data bits, no parity, 1 stop bit (8N1)
## Application Behavior
- Message Size: Each UART message is exactly 32 bytes in length.
- Contents: The message includes all relevant application data (e.g., race state, positions, fuel levels, etc.) and is a snapshot of the current race state.
- Transmission Rate: A new message is transmitted every 750 milliseconds.

|Byte Index|Field|Description|
|---|---|---|
|{0,4,...}|`Driver X - Fuel`|Remaining fuel of Driver 0–5|
|{1,5,...}|`Driver X - Position`|Current position of Driver 0–5|
|{2,6,...}|`Driver X - Finished`|Driver 0–5 has finished the race (1 = finished)|
|{3,7,...}|`Driver X - Laps`|Laps completed by Driver 0–5|
|24|`Fastest Lap`|Driver index with the fastest lap (0–5, or 0xFF if none)|
|25|`Start Lights Count`|Number of red lights currently lit|
|26|`Lap Count`|Total laps completed by the leader|
|27|`Reset Event`|Non-zero if a global reset has occurred|
|28|`Race State`|Current state of the race (see `RaceState` enum)|
|29|`Session Type`|Race or Qualifying session (see `SessionType` enum)|
|30 & 31|`Track Time`|16-bit track time counter, synced with track (13.33 Hz)|

## Message Decoding Example

The following C code snippet demonstrates how to decode the message and populate internal state structures:
```c 
typedef enum {
	RACE,
	QUALIFYING_TRAINING
}S essionType;

typedef enum {
	RACE_STATE_STARTUP,
	RACE_STATE_RED_FLAG,
	RACE_STATE_YELLOW_FLAG,
	RACE_STATE_GREEN_FLAG,
	RACE_STATE_START_PROC,
	RACE_STATE_JUMP_START,
	RACE_STATE_OPEN
} RaceState;

typedef struct {
	uint8_t fuel;
	uint8_t position;
	uint8_t finished;
	uint8_t laps;
} DriverData;

typedef struct {
	DriverData 	      driver_data[6];
	uint8_t		fastest_lap_driver;
	uint8_t		start_lights;
	uint8_t		lap_count;
	uint8_t		lap_count_nibble_completed;
	uint8_t		reset_event;
	RaceState	      race_state;
	SessionType	      session_type;
} RaceInfo;

// Global structure updated via UART
volatile RaceInfo	uart_race_info = {
		.driver_data = {
			{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 },
			{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 },
			{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 },
			{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 },
			{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 },
			{ .fuel = 0, .position = 0, .finished = 0, .laps = 0 }
		},
		.fastest_lap_driver = 0xFF,       // 0xFF no fastest lap yet
		.start_lights = 0,
		.lap_count = 0,
		.reset_event = 0,
		.race_state = RACE_STATE_STARTUP,
		.session_type = RACE
	};
	
volatile uint8_t rx_buf[32];

void uart_interrupt(void){
	uint8_t buf_idx = 0;
	for (uint8_t _driver = 0; _driver < 6; _driver++){
		uart_race_info.driver_data[_driver].position = 
			rx_buf[buf_idx++];
		uart_race_info.driver_data[_driver].fuel = 
			rx_buf[buf_idx++];
		uart_race_info.driver_data[_driver].laps = 
			rx_buf[buf_idx++];
		uart_race_info.driver_data[_driver].finished = 
			rx_buf[buf_idx++];
	}

	uart_race_info.fastest_lap_driver = rx_buf[buf_idx++];
	uart_race_info.start_lights = rx_buf[buf_idx++];
	uart_race_info.lap_count = rx_buf[buf_idx++];
	uart_race_info.reset_event = rx_buf[buf_idx++];
	uart_race_info.race_state = (RaceState)rx_buf[buf_idx++];
	uart_race_info.session_type = (SessionType)rx_buf[buf_idx++];

	track_time = 0xFF00 & (rx_buf[buf_idx++] << 8);
	track_time |= rx_buf[buf_idx++];
}
```

# Documentation
- [How the Lights Work](/docs/how_the_lights_work.md)
- [Getting Started](/docs/getting_started.md)

# Contribute & Support

Got ideas? Found a bug? Want to make it even cooler? 
Open an issue, submit a PR, or just say hi!

Developer: Philipp Krebs  
Contact: pmge.krebs@gmail.com  
Repo: https://github.com/philippmichaelkrebs  