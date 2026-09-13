# Adaptive Software-Defined Sonar Transmitter Payload

**Team Fennekin &middot; Smart India Hackathon 2026 &middot; PS ID SIH26058**
Ministry of Earth Sciences (MoES) / NIOT &middot; NIT Warangal

> Development of a Low-Power, Real-Time Adaptive Software-Defined Sonar
> Transmitter Payload for Autonomous Underwater Vehicles (AUVs)

---


## What this is

An AUV's sonar faces a trade-off it cannot avoid: **high frequency** gives a
sharp image but gets absorbed quickly by water and sediment; **low frequency**
travels much further but blurs the picture. A fixed-frequency sonar has to pick
one setting and live with it everywhere. This project doesn't pick - it
**measures the water it's in and works out the right waveform for that water,
every single pulse**, on an ESP32.

Every pulse, the firmware:
1. Reads the current water conditions (turbidity and depth - see below).
2. Runs **all 12 candidate waveforms** through a real underwater-acoustics
   physics model and scores each one.
3. Picks the waveform that gives the sharpest image for the least battery
   energy, **while still meeting the minimum SNR needed to detect anything at
   all**.
4. Synthesises that exact waveform as DAC samples and streams them out over
   I2S + DMA - the CPU is not involved sample-by-sample.
5. Reports the decision, the reasoning in plain English, and a live
   comparison against a traditional fixed-frequency sonar in the same water -
   all over WiFi, in real time.

## Repository contents

| File | What it is |
|---|---|
| `AdaptiveSonarPayload.ino` | The firmware. Flash this to an ESP32. This is where every decision is actually made. |
| `AdaptiveSonarPayload_Dashboard.py` | A desktop viewer. Connects to the ESP32 over WiFi and displays what it decided. Does no physics and makes no decisions itself - see "What runs where" below. |
| `*.mp4` | A short demo clip of the system running. |

---

## What runs where (read this before judging it)

This distinction is the whole point of the project, so it's stated up front
rather than left for someone to discover:

**On the ESP32 (the actual payload):**
- The ocean-acoustics physics engine - Francois & Garrison (1982) seawater
  absorption, plus a sediment-scattering term for suspended turbidity
- The full active sonar equation (source level, two-way transmission loss,
  target strength, noise level, array directivity, pulse-compression gain)
- Evaluating all 12 waveforms against the current environment and picking the
  best one, with a plain-English reason built from the actual numbers
- Synthesising the winning waveform into real DAC sample arrays
- Streaming those samples out through the internal DAC via I2S + DMA
- Computing the same metrics for a legacy fixed-frequency sonar, so every
  "improvement" figure is the ESP32's own number, not asserted afterwards

**On the laptop (the Python dashboard):**
- Only draws what the ESP32 already sent it over `/api/frame`
- Redraws the waveform trace at a higher frame rate than the ESP32's update
  interval, by interpolating the *same public waveform equations* the
  firmware uses, purely so the scroll looks smooth - it never changes which
  waveform is shown or why
- If the ESP32 is unplugged, the display freezes on its last known state.
  That's deliberate: it's the easiest way to prove the ESP32 is the one
  actually deciding.

## The 12-waveform library

Twelve LFM chirps, indexed from the setting they're built for:

| | Built for | Centre freq. | Bandwidth | Pulse |
|---|---|---|---|---|
| W1 | Muddy estuary, worst case | 90 kHz | 20 kHz | 400 µs |
| ⋮ | ⋮ | ⋮ | ⋮ | ⋮ |
| W12 | Clear reef, best case | 350 kHz | 100 kHz | 120 µs |

Every pulse, all 12 are scored and the best one wins - this is a real
optimisation over the library each time, not a fixed if/else ladder. Two
resolutions are tracked independently, because they come from different
waveform properties and that tension is the actual engineering problem:

- **Range resolution** `δR = c / 2B` - set by **bandwidth**
- **Along-track resolution** `δY = (λ/D)·R` - set by **centre frequency**

A higher centre frequency narrows the beam and sharpens the image but
attenuates faster. The optimiser is choosing that trade-off, not guessing it.

## Running it

**Firmware**
1. Arduino IDE, board package **esp32 by Espressif, version 2.x** (not 3.x -
   the built-in I2S DAC driver used here was removed in 3.x).
2. Board: **ESP32 Dev Module**.
3. Open and upload `AdaptiveSonarPayload.ino`.
4. Open Serial Monitor at 115200 baud to see the WiFi credentials it prints.

**Control & view**
- By default the ESP32 creates its own hotspot (`AP_SSID` / `AP_PASS` at the
  top of the `.ino`). Join it, then open `http://192.168.4.1` in any browser
  for the built-in control page with two sliders - **turbidity** (suspended
  sediment) and **depth**.
- For a richer view, run the Python dashboard on a laptop joined to the same
  network:
  ```
  pip install requests matplotlib numpy
  python AdaptiveSonarPayload_Dashboard.py            # ESP32 in AP mode
  python AdaptiveSonarPayload_Dashboard.py 192.168.1.42  # ESP32 joined your WiFi
  ```

No potentiometers, transducer, or extra hardware are required to see the
system make real decisions - the two environment sliders stand in for sensor
input, and everything downstream of them is real.

## Honesty about what's simulated

The **decision-making is entirely real** and runs on the ESP32. What is
*not* real is the ocean itself - there's no seawater tank or transducer in
this demo, so turbidity and depth are supplied as slider values rather than
measured acoustically. The physics that turns those two numbers into an SNR,
a resolution, and a waveform choice is the same physics a real payload would
use with real sensor input in place of the sliders.

## One-sentence principle

**Don't pick one frequency and hope - measure the water, and let the
waveform follow the physics.**
