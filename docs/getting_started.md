# 🏎️ FastLapLight – FIA Style Lights for Slot Car Racing

Bring high-speed realism to your slot car track with FastLapLight – a customizable lighting system that emulates real FIA race lights for start sequences, pit lane, and track sectors.

## What You Need

    - Access to this repository
    - PCB from JLCPCB (with assembly option)
    - WS2812B LED strip (60 LEDs/m)
    - 5mm RGB LEDs (for track lights)
    - 3D printer for enclosures (STL files included)
    - Basic soldering tools and wire

## Step-by-Step Setup
### 1. Clone the Repo via: 
 
git clone https://github.com/philippmichaelkrebs/FastLapLights.git

    - Make sure to clone all files, including the STL and assembly folders.
    - Alternativly download the repository as zip.

### 2. Order PCB from JLCPCB

- Go to jlcpcb.com
- Upload the Gerber files located in the /pcb folder
- Select Assembly option
- Upload the required BOM and Pick & Place files from /assembly

Once the PCB arrives, all surface-mounted components will already be placed for you.

### 3. Print the Enclosures

    Navigate to the STL/ folder

    Choose the appropriate housing:
    
        - generic_light_base.stl
        - starting_light_jib.stl
        - starting_light_faceplate.stl
        - pit_lane_light.stl

    Use your favorite slicer and print using black PLA or ABS for best realism

### 4. Choose Your Light Setup

 | Light Type   | LED Type |    LED Count |
 | -------------|----------|--------------|
|Starting Light |WS2812B (60/m)|    40 LEDs|
|Pit Lane Light |WS2812B (60/m) | 12 LEDs|
|Track Sector   |5mm RGB LEDs   | As needed|

    - Cut the WS2812B strip to the appropriate lengths
    - For track lights, solder your own 5mm RGB LEDs directly to the PCB

### 5. Solder & Assemble

Once your PCB arrives:

    - Solder pin headers or wires as needed
    - Connect the WS2812B strips to the correct pads
    - Use 2 wires to connect the light system to your slot car track (power and signal)

Make sure the polarity and orientation match your layout!

## 🚦 Light Modes (Optional Preview)

    FIA-accurate start sequence
    Pit lane entry/exit animations
    Yellow/Green/Red flag logic
    DMA-driven WS2812B control

## Coming Soon

    - Full firmware flashing guide
    - Full hardware wirering and soldering guide