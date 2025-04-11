# 🏁 How the Lights Work

## 📘 Introduction to FIA Light Signals

The FIA (Fédération Internationale de l'Automobile) defines several key light systems in their [recommendations for motor racing](https://argent.fia.com/web/fia-public.nsf/1330F0769A59327AC125736200478FB9/$FILE/03__Recommended_light_signals.pdf). These signals help ensure safety and structure during a race. Here's a simplified breakdown:

- **Start Light System**
  - Indicates the track status: open, closed, or released
  - Controls the start procedure
  - Signals a jump start or aborted start

- **Pit Lane Lights**
  - Indicates whether the pit lane is open or closed

- **Track Light Signals**
  - **Yellow flag**: caution, danger ahead
  - **Red flag**: race is stopped or paused
  - **Green flag**: track is clear and race continues

These are the most relevant signals for our slot car adaptation.

---

## 🟥 Red Flag – Race Pause

In FIA terms, a red flag means race stoppage. For us slot car/Carrera fans, it’s more like a **race pause**. All cars stop immediately — often because a car flies off the track and needs rescue. This can happen:

- **Before the start procedure begins**, or
- **Mid-race**, due to track incidents

The control unit sends a **reset** followed by a **5x red light** signal. If the previous state wasn't a start sequence, it's treated as a red flag.

### Light Behavior

- **Start Light**: All red lights ON. Yellow and green OFF.  
- **Pit Lane**: Red light ON – pit lane is closed.  
- **Track Lights**: All lights RED – red flag signal.

### How The Start Light Looks
 
![](https://github.com/philippmichaelkrebs/FastLapLights/blob/main/images/signal_red_flag_start_light.gif?raw=true)

### How The Track Light Looks
 
![](https://github.com/philippmichaelkrebs/FastLapLights/blob/main/images/signal_red_flag_track.gif?raw=true)


---

## 🟨 Yellow Flag / Safety Car Phase

A yellow flag indicates caution on the track – there's an incident or obstacle. You can optionally deploy a **safety car**. Without one, drivers are simply advised to slow down (though that's more a gentleman’s agreement than a rule 😉).

You trigger this phase by pressing the **Safety Car button**.

### Light Behavior

- **Start Light**: Yellow lights BLINK at 5 Hz.  
- **Pit Lane**: Green light ON – pit lane is open.  
- **Track Lights**: Yellow lights BLINK at 5 Hz.

### How The Start Light Looks
 
![](https://github.com/philippmichaelkrebs/FastLapLights/blob/main/images/signal_yellow_flag_start_light.gif?raw=true)

### How The Track Light Looks
 
![](https://github.com/philippmichaelkrebs/FastLapLights/blob/main/images/signal_yellow_flag_track.gif?raw=true)

---

## 🟩 Green Flag – Race Resumes

A green flag means the track is clear and the race is back on.

There’s **no direct button** to start a green flag phase. Instead, it's shown **automatically for 10 seconds** after ending a red flag or yellow flag phase.

### Light Behavior

- **Start Light**: Green lights BLINK at 1 Hz (for 10 seconds).  
- **Pit Lane**: Green light ON – track and pit lane are open.  
- **Track Lights**: Green lights BLINK at 1 Hz – track is clear.

---

## 🚦 Start Procedure

Pressing the **Start button** during a red flag triggers the official **start sequence**. According to FIA rules, only the **start light system** is active during this sequence — track lights remain off, assuming the track is already clear.

### Light Behavior

- **Start Light**: Red lights turn on in sequence (typical countdown).  
- **Pit Lane**: Red light ON during countdown; turns green at race start.  
- **Track Lights**: All OFF – the track is considered ready.

### How The Start Light Looks
 
![](https://github.com/philippmichaelkrebs/FastLapLights/blob/main/images/signal_start_proc_start_light.gif?raw=true)

---

## 🚫 Jump Start

If a car jumps the start by hitting the throttle before the lights go out, it’s a **jump start**. In slot car logic, this acts like a **start abort**. So we follow the FIA approach:

### Light Behavior

- **Start Light**: Red lights stay ON; Yellow lights FLASH at 1 Hz.  
- **Pit Lane**: Red light ON – pit lane closed.  
- **Track Lights**: All OFF – track assumed clear.

### How The Start Light Looks
 
![](https://github.com/philippmichaelkrebs/FastLapLights/blob/main/images/signal_jump_start.gif?raw=true)