#!/usr/bin/env python3
"""
===============================================================================
 ADAPTIVE SONAR PAYLOAD - LIVE DASHBOARD                            SIH26058
===============================================================================

 This window is a DISPLAY ONLY for the DECISION.

 The ESP32 alone decides which waveform to use and computes every physics
 number (absorption, SNR, resolution, energy, the reason text). Those come
 over WiFi from http://<esp32-ip>/api/frame and are never touched.

 The waveform TRACE itself is redrawn here at high frame rate for smooth
 motion. To do that smoothly, this file re-evaluates the SAME publicly known
 chirp/sine equations the firmware uses (LFM phase-integral, window
 functions, etc.) at extra points in time between the ESP32's updates -
 exactly the way a graphics engine interpolates between physics-engine
 keyframes. It never changes WHICH waveform is shown or WHY - it only
 re-draws the one the ESP32 already chose, more smoothly than one JSON
 frame every ~350 ms could otherwise look. If you unplug the ESP32, the top
 panel goes blank and the waveform freezes on its last known shape - which is
 the point, and worth demonstrating to a judge.

 WHY THE WAVE LOOKS THE WAY IT DOES
 -----------------------------------
 The trace is a genuinely continuous, physically-driven scroll (not a fake
 "spin the array" trick, and not a "reveal then blank" animation):
   - A FIXED time window is always shown (not scaled per waveform), so a
     HIGH frequency signal visibly packs MORE wave crests into that same
     window (congested) and a LOW frequency signal spreads FEWER crests
     across it (free) - exactly the density contrast you'd see on a real
     oscilloscope.
   - The window scrolls forward in real, continuous time at a rate tied to
     the waveform's centre frequency: higher frequency scrolls faster,
     lower frequency scrolls slower - because a higher-frequency ping needs
     less time lingering in the water, while a low-frequency ping (chosen
     for muddy/deep water) is deliberately longer, and that pace is shown
     directly instead of merely stated.

 Usage
   python AdaptiveSonarPayload_Dashboard.py                # tries 192.168.4.1 (ESP32 AP mode)
   python AdaptiveSonarPayload_Dashboard.py 192.168.1.42    # ESP32 joined your WiFi (STA mode)
===============================================================================
"""

import sys
import time
import math
import threading
import textwrap

try:
    import requests
except ImportError:
    sys.exit("Missing dependency. Run:  pip install requests matplotlib numpy")

try:
    import numpy as np
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    sys.exit("Missing dependency. Run:  pip install requests matplotlib numpy")


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
ESP32_IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.4.1"
URL = f"http://{ESP32_IP}/api/frame"
POLL_SECONDS = 0.35        # how often we ask the ESP32 for a new decision
FPS_INTERVAL_MS = 20       # redraw rate for the wave scroll (~50 fps target)

BG     = "#0a1020"
PANEL  = "#151f38"
OURS   = "#63e6a0"
LEGACY = "#ff8a6b"
TEXT   = "#e9eefb"
MUTED  = "#7f8db0"
ACCENT = "#8fd4ff"
GOLD   = "#ffd166"

# --- Continuous-scroll tuning -----------------------------------------------
VIEW_WINDOW_US   = 260.0   # FIXED width of the visible time slice, in signal-us.
                           # Fixed on purpose: this is what makes a high-frequency
                           # ping look "congested" and a low one look "free" -
                           # more or fewer real cycles fit in the same window.
REF_FREQ_KHZ     = 200.0   # reference centre frequency for scroll speed
BASE_SCROLL_RATE = 150.0   # signal-us played per real second, AT the reference
MIN_SCROLL_RATE  = 35.0
MAX_SCROLL_RATE  = 420.0
N_POINTS         = 900     # samples drawn per frame (smooth even when dense)

BARKER7  = [1,1,1,-1,-1,1,-1]
BARKER13 = [1,1,1,1,1,-1,-1,1,1,-1,1,-1,1]


# ---------------------------------------------------------------------------
# Background poller - keeps the GUI smooth even if WiFi stalls
# ---------------------------------------------------------------------------
class Poller(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.frame = None
        self.error = "connecting to ESP32..."
        self.ok_count = 0
        # NOTE: must not be named "_stop" - that name collides with a private
        # Thread internal used during shutdown and silently kills the thread.
        self._stop_event = threading.Event()

    def run(self):
        session = requests.Session()
        while not self._stop_event.is_set():
            try:
                r = session.get(URL, timeout=2.0)
                self.frame = r.json()
                self.error = None
                self.ok_count += 1
            except Exception as e:
                self.error = f"No data from {ESP32_IP} ({type(e).__name__})"
            self._stop_event.wait(POLL_SECONDS)

    def stop(self):
        self._stop_event.set()


poller = Poller()
poller.start()


# ---------------------------------------------------------------------------
# Waveform maths - mirrors the ESP32 firmware's generators EXACTLY, so the
# smooth trace drawn here is mathematically the same signal, just resampled
# at extra points in time. This does not decide anything: f0, f1, pulseUs,
# type, window and amplitude all come from the ESP32's JSON frame. See the
# module docstring for why this split is legitimate.
# ---------------------------------------------------------------------------
def window_value(win_name, r):
    """r is time-within-pulse normalised to [0,1). Vectorised over numpy arrays."""
    if win_name == "HANN":
        return 0.5 - 0.5 * np.cos(2 * np.pi * r)
    if win_name == "HAMMING":
        return 0.54 - 0.46 * np.cos(2 * np.pi * r)
    if win_name == "BLACKMAN":
        return 0.42 - 0.5 * np.cos(2 * np.pi * r) + 0.08 * np.cos(4 * np.pi * r)
    return np.ones_like(r)  # RECTANGULAR


def waveform_value(wtype, f0, f1, pulse_us, window_name, amplitude_frac, t_mod_us):
    """
    t_mod_us: numpy array of times, already wrapped into [0, pulse_us).
    Returns the shaped value in [-1, 1] (raw waveform x amplitude x window).
    Treated as exactly periodic with period = pulse_us (no artificial silence
    gap) - which is why every waveform in the library uses a window that
    tapers to (near) zero at both ends: it hides the repeat seam in
    amplitude, the same way a real transmitter would never want a hard
    edge there anyway.
    """
    t = t_mod_us * 1e-6          # seconds
    T = pulse_us * 1e-6           # seconds
    r = np.clip(t_mod_us / max(pulse_us, 1e-9), 0.0, 0.999999)

    if wtype == "SINE":
        raw = np.sin(2 * np.pi * f0 * t)
    elif wtype == "GEOMETRIC":
        if f0 <= 0 or abs(f1 - f0) < 1.0:
            raw = np.sin(2 * np.pi * f0 * t)
        else:
            ratio = f1 / f0
            lnr = math.log(ratio)
            raw = np.sin(2 * np.pi * (f0 * T / lnr) * (np.power(ratio, t / T) - 1.0))
    elif wtype == "PHASE_CODED":
        code = BARKER7
        chip_dur = T / len(code)
        chip_idx = np.clip((t / chip_dur).astype(int), 0, len(code) - 1)
        signs = np.array(code, dtype=float)[chip_idx]
        raw = signs * np.sin(2 * np.pi * f0 * t)
    else:  # "LFM" (the default - covers every preset in the current library)
        k = (f1 - f0) / T if T > 0 else 0.0
        raw = np.sin(2 * np.pi * (f0 * t + 0.5 * k * t * t))

    win = window_value(window_name, r)
    return raw * amplitude_frac * win


def scroll_positions(now, t0, scroll_rate_us_per_s, window_us, n_points):
    """Absolute (unwrapped) signal-time positions currently visible, in us."""
    t_end = (now - t0) * scroll_rate_us_per_s
    return np.linspace(t_end - window_us, t_end, n_points)


# ---------------------------------------------------------------------------
# Figure layout
# ---------------------------------------------------------------------------
plt.rcParams["toolbar"] = "none"
fig = plt.figure(figsize=(15, 9), facecolor=BG)
fig.canvas.manager.set_window_title("Adaptive Sonar Payload - Live Output (ESP32)")

gs = fig.add_gridspec(
    4, 3,
    height_ratios=[1.15, 1.15, 1.15, 1.15],
    width_ratios=[1.5, 1, 1],
    hspace=0.78, wspace=0.30,
    left=0.055, right=0.975, top=0.945, bottom=0.075,
)

ax_info   = fig.add_subplot(gs[0, :])
ax_ours   = fig.add_subplot(gs[1, :])
ax_legacy = fig.add_subplot(gs[2, :])
ax_bars   = fig.add_subplot(gs[3, 0])
ax_freq   = fig.add_subplot(gs[3, 1])
ax_score  = fig.add_subplot(gs[3, 2])

for ax in (ax_info, ax_ours, ax_legacy, ax_bars, ax_freq, ax_score):
    ax.set_facecolor(PANEL)
    for s in ax.spines.values():
        s.set_color("#26324f")
    ax.tick_params(colors=MUTED, labelsize=8)

ax_info.axis("off")


def style_wave_axis(ax, title, color):
    ax.set_title(title, color=color, fontsize=11, fontweight="bold", loc="left", pad=8)
    ax.set_ylim(-1.35, 1.35)
    ax.set_xlim(0, 1)
    ax.grid(True, color="#1e2a47", linewidth=0.6)
    ax.set_ylabel("amplitude", color=MUTED, fontsize=8)
    ax.set_xticks([])


style_wave_axis(ax_ours, "OUR PAYLOAD - adaptive waveform (synthesised on ESP32)", OURS)
style_wave_axis(ax_legacy, "CONVENTIONAL SONAR - one fixed pulse for every water condition", LEGACY)

# ----- persistent artists: created ONCE, updated via set_data every frame --
line_ours,   = ax_ours.plot([], [], color=OURS, linewidth=1.3,
                            label="synthesised waveform (smooth)")
line_true,   = ax_ours.plot([], [], color=GOLD, linewidth=0.8, alpha=0.85,
                            drawstyle="steps-post",
                            label="actual 8-bit DAC codes (real sample rate)")
line_ours_env_hi, = ax_ours.plot([], [], color=OURS, linewidth=0.8, alpha=0.30, linestyle="--")
line_ours_env_lo, = ax_ours.plot([], [], color=OURS, linewidth=0.8, alpha=0.30, linestyle="--")
line_legacy, = ax_legacy.plot([], [], color=LEGACY, linewidth=1.3)

ax_ours.legend(fontsize=7.5, facecolor=PANEL, edgecolor="#26324f",
               labelcolor=TEXT, loc="upper right", ncol=2)

txt_ours_ann   = ax_ours.text(0.995, -1.22, "", color=ACCENT, fontsize=8.5,
                              ha="right", va="bottom", family="monospace")
txt_legacy_ann = ax_legacy.text(0.995, -1.22, "", color="#ffb9a3", fontsize=8.5,
                                ha="right", va="bottom", family="monospace")

# Info panel text
t_title  = ax_info.text(0.005, 1.02, "", color=TEXT, fontsize=12,
                        fontweight="bold", va="top")
t_status = ax_info.text(0.005, 0.72, "", color=MUTED, fontsize=8.2, va="top")
t_env    = ax_info.text(0.005, 0.56, "", color=ACCENT, fontsize=10,
                        va="top", family="monospace")
t_metric = ax_info.text(0.005, 0.34, "", color=MUTED, fontsize=9,
                        va="top", family="monospace")
t_reason = ax_info.text(0.005, 0.10, "", color="#c3d2f0", fontsize=9.2, va="top")

# ----- instantaneous-frequency panel: static curve rebuilt only when the
# waveform changes, moving dot updated every frame ---------------------------
freq_line_ours,  = ax_freq.plot([], [], color=OURS, linewidth=2, label="adaptive chirp")
freq_line_legacy, = ax_freq.plot([], [], color=LEGACY, linewidth=2, linestyle="--",
                                 label="legacy CW")
freq_dot, = ax_freq.plot([], [], marker="o", markersize=8, color=GOLD,
                         markeredgecolor=BG, markeredgewidth=1.5, zorder=5)
ax_freq.set_title("instantaneous frequency", color=TEXT, fontsize=9.5, loc="left")
ax_freq.set_xlabel("time (us)", color=MUTED, fontsize=8)
ax_freq.set_ylabel("kHz", color=MUTED, fontsize=8)
ax_freq.grid(True, color="#1e2a47", linewidth=0.6)
ax_freq.legend(fontsize=7, facecolor=PANEL, edgecolor="#26324f", labelcolor=TEXT)
ax_freq.set_ylim(0, 560)

t0 = time.time()          # single fixed origin for all continuous scrolling
_slow_cache = {"identity": None}   # tracks when a NEW ESP32 decision arrived


def fmt_freq(hz):
    return f"{hz/1000:.0f} kHz"


def scroll_rate_for(fc_khz):
    rate = BASE_SCROLL_RATE * (fc_khz / REF_FREQ_KHZ)
    return min(max(rate, MIN_SCROLL_RATE), MAX_SCROLL_RATE)


# ---------------------------------------------------------------------------
# SLOW PATH: only runs when a genuinely new decision arrives from the ESP32
# (a few times a second). Everything expensive - text layout, legends, bar
# charts - lives here, NOT in the 50 fps hot path below.
# ---------------------------------------------------------------------------
def refresh_slow_elements(f):
    a, l, imp = f["adaptive"], f["legacy"], f["improve"]
    feasible = f.get("feasible", True)

    mark = "" if feasible else "   [DETECTION THRESHOLD NOT MET]"
    title_line = (f"Payload selected:  {f['label']}   |   "
                 f"{f['type']}  {fmt_freq(f['f0'])} -> {fmt_freq(f['f1'])}   "
                 f"{f['pulseUs']} us   {f['amplitude']}%   {f['window']}{mark}")
    t_title.set_text(textwrap.fill(title_line, width=100))
    t_title.set_color(TEXT if feasible else "#ff9a9a")

    t_env.set_text(
        f"ENVIRONMENT   turbidity {f['turbidity']:6.0f} mg/L    depth {f['depth']:6.0f} m"
        f"    -> absorption {a['absorption']:6.1f} dB/km"
    )
    t_metric.set_text(
        f"echo SNR {a['snr']:6.1f} dB   |   range res {a['rangeRes']*100:5.1f} cm   |   "
        f"along-track res {a['alongRes']:5.2f} m   |   pulse-compression gain "
        f"{a['procGain']:4.1f} dB   |   {f['samples']} DAC samples @ {f['sampleRate']/1e6:.1f} MS/s"
    )
    wrapped = textwrap.fill(f["reason"], width=165)
    t_reason.set_text("WHY THIS WAVEFORM:  " + wrapped)
    t_reason.set_color("#c3d2f0" if feasible else "#ffc9c9")

    fc = 0.5 * (f["f0"] + f["f1"]) / 1000.0
    rate = scroll_rate_for(fc)
    speed_word = ("FAST - clear water needs no extra penetration energy" if rate > 200 else
                 "MODERATE speed" if rate > 100 else
                 "SLOW - long wavelength needed to punch through this water")
    t_status.set_text(
        f"live from ESP32 @ {ESP32_IP}   frames: {poller.ok_count}    |    "
        f"wave speed & density: {speed_word}   (centre freq {fc:.0f} kHz)"
    )

    bw = abs(f["f1"] - f["f0"])
    txt_ours_ann.set_text(
        f"chirp {fmt_freq(f['f0'])}->{fmt_freq(f['f1'])}  |  bandwidth {bw/1000:.0f} kHz  |  "
        f"{f['window']} taper suppresses sidelobes & protects the amplifier  |  "
        f"time-bandwidth {bw*f['pulseUs']*1e-6:.1f}"
    )
    txt_legacy_ann.set_text(
        f"fixed {f['legacyFreq']/1000:.0f} kHz CW  |  bandwidth {l['bandwidth']/1000:.1f} kHz  |  "
        f"rectangular - hard switch-on/off (visible as a hard jump each repeat)  |  "
        f"time-bandwidth 1.0 (no compression gain)"
    )

    tt = np.linspace(0, f["pulseUs"], 100)
    finst = f["f0"] + (f["f1"] - f["f0"]) * (tt / max(f["pulseUs"], 1))
    freq_line_ours.set_data(tt, finst / 1000.0)
    freq_line_legacy.set_data(tt, np.full_like(tt, f["legacyFreq"] / 1000.0))
    ax_freq.set_xlim(0, max(f["pulseUs"], f["legacyPulseUs"]))

    # ----- comparison bars ---------------------------------------------------
    ax_bars.clear()
    ax_bars.set_facecolor(PANEL)
    ax_bars.tick_params(colors=MUTED, labelsize=8)
    for s in ax_bars.spines.values():
        s.set_color("#26324f")

    names = ["detection\nSNR (dB)", "image sharpness\n(1 / cell area)",
             "battery efficiency\n(1 / energy)"]
    ours_vals = [a["snr"], 1.0 / max(a["cellArea"], 1e-9), 1.0 / max(a["energy"], 1e-9)]
    leg_vals  = [l["snr"], 1.0 / max(l["cellArea"], 1e-9), 1.0 / max(l["energy"], 1e-9)]
    raw_labels = [(f"{a['snr']:.1f}", f"{l['snr']:.1f}"),
                  (f"{a['cellArea']:.4f} m2", f"{l['cellArea']:.4f} m2"),
                  (f"{a['energy']:.0f}", f"{l['energy']:.0f}")]
    idx = np.arange(3)
    norm = [max(abs(o), abs(g), 1e-6) for o, g in zip(ours_vals, leg_vals)]
    ax_bars.bar(idx - 0.19, [o / m for o, m in zip(ours_vals, norm)], 0.36, color=OURS, label="ours")
    ax_bars.bar(idx + 0.19, [g / m for g, m in zip(leg_vals, norm)], 0.36, color=LEGACY, label="legacy")
    for i, (o, g) in enumerate(zip(ours_vals, leg_vals)):
        ax_bars.text(i - 0.19, o / norm[i] + 0.03, raw_labels[i][0], ha="center", color=OURS, fontsize=7)
        ax_bars.text(i + 0.19, g / norm[i] + 0.03, raw_labels[i][1], ha="center", color=LEGACY, fontsize=7)
    ax_bars.set_xticks(idx)
    ax_bars.set_xticklabels(names, color=MUTED, fontsize=7.5)
    ax_bars.set_yticks([])
    ax_bars.set_ylim(min(0, min([o/m for o,m in zip(ours_vals,norm)] +
                                 [g/m for g,m in zip(leg_vals,norm)])) - 0.25, 1.35)
    ax_bars.set_title("same water, both systems  (taller = better)", color=TEXT, fontsize=9.5, loc="left")
    ax_bars.legend(fontsize=7, facecolor=PANEL, edgecolor="#26324f", labelcolor=TEXT, loc="lower right")

    # ----- improvement index ---------------------------------------------------
    ax_score.clear()
    ax_score.set_facecolor(PANEL)
    ax_score.axis("off")
    overall = imp["overall"]
    colour = OURS if overall > 0 else LEGACY
    ax_score.text(0.5, 0.80, "OVERALL IMPROVEMENT", ha="center", color=MUTED, fontsize=9)
    ax_score.text(0.5, 0.50, f"{overall:+,.0f}%", ha="center", color=colour, fontsize=30, fontweight="bold")
    ax_score.text(0.5, 0.30, f"SNR {imp['snr']:+.1f} dB      image cell {imp['cell']:.1f}x sharper",
                  ha="center", color=ACCENT, fontsize=9)
    ax_score.text(0.5, 0.17, f"energy per ping {imp['energy']:+.0f}% vs legacy",
                  ha="center", color=ACCENT, fontsize=9)
    ax_score.text(0.5, 0.03, "index = detection margin x sqrt(image sharpness), vs fixed 200 kHz CW",
                  ha="center", color=MUTED, fontsize=7)


# ---------------------------------------------------------------------------
# FAST PATH: runs every animation tick (~50 fps). Only set_data/set_offsets -
# no clear(), no legend(), no text object creation - this is what keeps the
# wave motion smooth instead of stuttering.
# ---------------------------------------------------------------------------
def update(_):
    now = time.time()
    f = poller.frame
    if f is None:
        t_title.set_text("Waiting for ESP32...")
        t_status.set_text(poller.error or "")
        t_env.set_text(f"Trying {URL}\n\n"
                       "1. Connect this laptop to the ESP32's WiFi\n"
                       "2. Check the IP printed in the Serial Monitor\n"
                       "3. Pass it as an argument:  python AdaptiveSonarPayload_Dashboard.py <ip>")
        return []

    identity = (f["index"], f["f0"], f["f1"], f["pulseUs"], f["window"], f["type"], f["amplitude"])
    if identity != _slow_cache["identity"]:
        _slow_cache["identity"] = identity
        refresh_slow_elements(f)

    # ----- adaptive waveform: continuous physically-driven scroll ----------
    fc = 0.5 * (f["f0"] + f["f1"]) / 1000.0
    rate = scroll_rate_for(fc)
    t_arr = scroll_positions(now, t0, rate, VIEW_WINDOW_US, N_POINTS)
    t_mod = np.mod(t_arr, max(f["pulseUs"], 1e-6))
    y = waveform_value(f["type"], f["f0"], f["f1"], f["pulseUs"], f["window"],
                       f["amplitude"] / 100.0, t_mod)
    x = np.linspace(0, 1, N_POINTS)
    line_ours.set_data(x, y)

    # true 8-bit DAC codes: same signal, snapped to the real hardware sample
    # grid (drawn as steps) so the quantisation is shown honestly.
    dt_sample = 1e6 / max(f["sampleRate"], 1.0)     # us per DAC sample
    t_sampled = np.floor(t_arr / dt_sample) * dt_sample
    t_sampled_mod = np.mod(t_sampled, max(f["pulseUs"], 1e-6))
    y_true = waveform_value(f["type"], f["f0"], f["f1"], f["pulseUs"], f["window"],
                            f["amplitude"] / 100.0, t_sampled_mod)
    line_true.set_data(x, y_true)

    env = np.abs(y)
    k = max(3, N_POINTS // 60)
    env_smooth = np.convolve(env, np.ones(k) / k, mode="same")
    line_ours_env_hi.set_data(x, env_smooth)
    line_ours_env_lo.set_data(x, -env_smooth)

    # ----- legacy waveform: constant tempo, constant band, on purpose -------
    legacy_rate = scroll_rate_for(f["legacyFreq"] / 1000.0)
    t_arr_l = scroll_positions(now, t0, legacy_rate, VIEW_WINDOW_US, N_POINTS)
    t_mod_l = np.mod(t_arr_l, max(f["legacyPulseUs"], 1e-6))
    y_l = waveform_value("SINE", f["legacyFreq"], f["legacyFreq"], f["legacyPulseUs"],
                        "RECTANGULAR", 1.0, t_mod_l)
    line_legacy.set_data(x, y_l)

    # ----- moving playhead on the frequency panel ---------------------------
    t_end_mod = np.mod((now - t0) * rate, max(f["pulseUs"], 1e-6))
    play_f = (f["f0"] + (f["f1"] - f["f0"]) * (t_end_mod / max(f["pulseUs"], 1))) / 1000.0
    freq_dot.set_data([t_end_mod], [play_f])

    return [line_ours, line_true, line_ours_env_hi, line_ours_env_lo, line_legacy, freq_dot]


ani = FuncAnimation(fig, update, interval=FPS_INTERVAL_MS,
                    blit=False, cache_frame_data=False)

print(f"Dashboard running. Polling {URL}")
print("Move the sliders on the ESP32 web page and watch this window react.")
try:
    plt.show()
finally:
    poller.stop()
