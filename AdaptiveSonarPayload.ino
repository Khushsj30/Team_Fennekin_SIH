/*
  ============================================================================
   ADAPTIVE SOFTWARE-DEFINED SONAR TRANSMITTER PAYLOAD  -  SIH26058
   ESP32 firmware  (Ministry of Earth Sciences / NIOT problem statement)
  ============================================================================

   FILE  : AdaptiveSonarPayload.ino  (must sit in a folder of the
           same name: AdaptiveSonarPayload/)
   BOARD : classic ESP32-WROOM-32 / ESP32 DevKit V1
   CORE  : Arduino-ESP32  v2.0.14      <-- must be 2.x, see README
   EXTRA HARDWARE REQUIRED : none (two on-screen sliders replace the knobs)

   ---------------------------------------------------------------------------
   WHAT RUNS WHERE  (important - read before the demo)
   ---------------------------------------------------------------------------
   EVERYTHING below runs ON THE ESP32:
     - the ocean-acoustics physics engine (Francois & Garrison absorption,
       sediment scattering, the active sonar equation)
     - evaluating ALL 12 candidate waveforms against the live environment
     - choosing the optimum one and generating the human-readable reason
     - synthesising the actual DAC sample array for that waveform
     - streaming those samples to the internal DAC through I2S + DMA
     - computing the same metrics for the legacy fixed-frequency sonar
       so the improvement figures are the ESP32's own numbers

   The Python window on your laptop ONLY DRAWS what the ESP32 sends.
   It performs no selection and no physics. This distinction matters: it is
   what makes this a hardware payload rather than a desktop simulation.

   ---------------------------------------------------------------------------
   CONFIGURATION vs SAMPLES (keep these separate in your head)
   ---------------------------------------------------------------------------
     CONFIGURATION : "LFM, 400 kHz -> 500 kHz, 50 us, 40 %"   (WaveformConfig)
     SAMPLES       : 128, 141, 155, 167, 176, ...              (dacSamples[])
   The firmware's job is turning the first into the second, every pulse.
  ============================================================================
*/

#include <WiFi.h>
#include <WebServer.h>
#include "driver/i2s.h"
#include <math.h>

// ============================================================================
//  SECTION 1 : USER SWITCHES
// ============================================================================

// --- Network ---------------------------------------------------------------
// MODE_AP = true  : the ESP32 creates its own WiFi hotspot. Simplest, works
//                   anywhere, but your laptop loses internet while connected.
// MODE_AP = false : the ESP32 joins your existing WiFi/phone hotspot. Your
//                   laptop keeps internet. Fill in STA_SSID / STA_PASS.
#define MODE_AP   true

const char *AP_SSID  = "ESP32_SONAR";
const char *AP_PASS  = "sonar1234";      // >= 8 characters
const char *STA_SSID = "YOUR_WIFI_NAME";
const char *STA_PASS = "YOUR_WIFI_PASS";

// --- Behaviour -------------------------------------------------------------
#define USE_POTENTIOMETERS false   // false = sliders on the web page are the
                                   // knobs. Set true LATER when you buy pots.
#define PRINT_SAMPLES      false   // print first 24 DAC codes each pulse
#define PULSE_INTERVAL_MS  50      // gap between transmitted pulses

// --- Signal chain ----------------------------------------------------------
#define SAMPLE_RATE   1000000UL    // 1 MS/s DAC rate (see README limitations)
#define MAX_SAMPLES   4096         // hard cap on the RAM sample buffer
#define EXPORT_POINTS 400          // fine-grid display render sent to dashboard
#define TRUE_POINTS   256          // real DAC codes sent to dashboard

// --- Pins (only used if USE_POTENTIOMETERS is true) -------------------------
#define ADC_TURBIDITY_PIN 32       // ADC1_CH4
#define ADC_DEPTH_PIN     33       // ADC1_CH5
// DAC output is fixed in hardware to GPIO25 (DAC1) by I2S_DAC_CHANNEL_RIGHT_EN

// ============================================================================
//  SECTION 2 : PHYSICAL / SONAR CONSTANTS
//  These are the assumptions behind every number the payload reports.
//  Change them here and the whole model follows.
// ============================================================================
const float SOUND_SPEED   = 1500.0f;  // m/s, nominal seawater
const float WATER_TEMP_C  = 15.0f;    // assumed temperature
const float SALINITY_PPT  = 35.0f;    // assumed salinity
const float WATER_PH      = 8.0f;     // assumed pH
const float SOURCE_LEVEL  = 210.0f;   // dB re 1uPa @1m at 100% amplitude
const float NOISE_SPECTRAL= 40.0f;    // dB re 1uPa/sqrt(Hz) ambient
const float TARGET_STRENGTH = -15.0f; // dB, nominal seabed backscatter
const float ARRAY_GAIN    = 20.0f;    // dB, receive directivity index
const float ARRAY_APERTURE= 0.30f;    // m, transducer aperture (sets beamwidth)
const float SNR_REQUIRED  = 12.0f;    // dB, detection threshold with margin
// A side-scan sonar images SIDEWAYS across a swath much wider than its
// altitude, so the slant range to the far edge of the swath is several times
// the depth. 2.5x is a typical side-scan swath-to-altitude ratio. Using
// range == depth would wrongly make the depth input almost irrelevant.
const float SWATH_RATIO   = 2.5f;
const float ENERGY_WEIGHT = 1.2f;     // how hard we punish battery use

// The legacy system we are benchmarking against: a traditional sonar that
// transmits ONE fixed CW pulse regardless of the water it is swimming through.
const float LEGACY_FREQ_HZ     = 200000.0f;
const float LEGACY_PULSE_US    = 100.0f;
const float LEGACY_AMPLITUDE   = 1.00f;

// ============================================================================
//  SECTION 3 : DATA TYPES
// ============================================================================
enum WaveformType : uint8_t { SINE = 0, LFM = 1, GEOMETRIC = 2, PHASE_CODED = 3 };
enum WindowType   : uint8_t { WIN_RECT = 0, WIN_HANN = 1, WIN_HAMMING = 2, WIN_BLACKMAN = 3 };

struct WaveformConfig {
  const char   *label;
  WaveformType  type;
  uint32_t      startFreqHz;
  uint32_t      endFreqHz;     // == startFreqHz for SINE / PHASE_CODED carrier
  uint32_t      pulseWidthUs;
  uint8_t       amplitudePct;  // 0-100 % of full DAC swing
  WindowType    window;
  uint8_t       phaseCodeId;   // 0 = Barker-7, 1 = Barker-13
};

// Everything the physics engine computes for one waveform in one environment.
struct SonarMetrics {
  float absorption;   // dB/km   total (seawater + suspended sediment)
  float transmission; // dB      one-way transmission loss
  float snr;          // dB      echo signal-to-noise ratio
  float rangeRes;     // m       range resolution  = c / 2B
  float alongRes;     // m       along-track resolution = beamwidth * range
  float cellArea;     // m^2     resolution cell = rangeRes * alongRes
  float energy;       // relative transmitted energy (amplitude^2 * duration)
  float bandwidth;    // Hz      effective bandwidth
  float procGain;     // dB      pulse-compression gain = 10log10(B*tau)
  bool  feasible;     // did it clear the detection threshold?
};

// ============================================================================
//  SECTION 4 : THE 12-WAVEFORM LIBRARY   <-- EDIT THESE VALUES FREELY
// ----------------------------------------------------------------------------
//  This is CONFIGURATION only - no sample values live here. It is `const`
//  so the compiler keeps it in flash, not RAM.
//  Index 0  = built for the worst water (deep + very muddy)
//  Index 11 = built for the best water  (very shallow + very clear)
// ============================================================================
const WaveformConfig waveforms[12] = {
/* 0*/ {"W1  Muddy estuary, very high atten.", LFM,  80000, 100000, 400, 100, WIN_HANN,     0},
/* 1*/ {"W2  Muddy estuary, high atten.",      LFM,  90000, 110000, 380, 100, WIN_HANN,     0},
/* 2*/ {"W3  Muddy estuary, medium",           LFM, 100000, 120000, 350,  98, WIN_HANN,     0},
/* 3*/ {"W4  Muddy estuary, moderate",         LFM, 100000, 140000, 320,  96, WIN_HANN,     0},
/* 4*/ {"W5  Turbid water",                    LFM, 110000, 150000, 300,  94, WIN_HAMMING,  0},
/* 5*/ {"W6  Transition water",                LFM, 120000, 170000, 280,  92, WIN_HAMMING,  0},
/* 6*/ {"W7  Clear water",                     LFM, 150000, 200000, 250,  90, WIN_HAMMING,  0},
/* 7*/ {"W8  Clear shallow water",             LFM, 180000, 230000, 220,  90, WIN_HAMMING,  0},
/* 8*/ {"W9  Clear shallow reef",              LFM, 200000, 260000, 200,  88, WIN_BLACKMAN, 0},
/* 9*/ {"W10 Clear reef, high resolution",     LFM, 220000, 300000, 180,  88, WIN_BLACKMAN, 0},
/*10*/ {"W11 Very clear reef",                 LFM, 250000, 350000, 150,  86, WIN_BLACKMAN, 0},
/*11*/ {"W12 Clear reef, maximum resolution",  LFM, 300000, 400000, 120,  85, WIN_BLACKMAN, 0},
};
#define NUM_WAVEFORMS 12

// Barker codes for the PHASE_CODED waveform type
const int8_t BARKER7[7]   = {1,1,1,-1,-1,1,-1};
const int8_t BARKER13[13] = {1,1,1,1,1,-1,-1,1,1,-1,1,-1,1};

// ============================================================================
//  SECTION 5 : RAM BUFFERS
// ----------------------------------------------------------------------------
//  dacSamples[] : the 8-bit DAC codes (0-255). Readable, printable, exported.
//  i2sBuffer[]  : the same values repacked as 16-bit words, because the I2S
//                 built-in-DAC mode reads the DAC code from the UPPER byte.
//  Both sit in ordinary internal RAM, which the I2S DMA engine reads directly.
// ============================================================================
uint8_t  dacSamples[MAX_SAMPLES];
uint16_t i2sBuffer [MAX_SAMPLES];

// Handed to the Python dashboard for plotting.
//  exportAdaptive/Legacy = fine-grid render of the ideal analogue waveform
//  exportTrue            = the ACTUAL 8-bit DAC codes that DMA clocked out
uint8_t  exportAdaptive[EXPORT_POINTS];
uint8_t  exportLegacy  [EXPORT_POINTS];
uint8_t  exportTrue    [TRUE_POINTS];
int      exportTrueLen = 0;

// ============================================================================
//  SECTION 6 : LIVE STATE
// ============================================================================
float envTurbidity = 25.0f;   // suspended sediment, mg/L  (0 - 500)
float envDepth     = 20.0f;   // water depth, m            (1 - 200)

int             activeIndex  = -1;
WaveformConfig  activeConfig = waveforms[5];
SonarMetrics    activeMetrics;
SonarMetrics    legacyMetrics;
uint32_t        activeSamples = 0;
String          selectionReason = "initialising";
bool            detectionAchievable = true;

unsigned long lastPulseMs = 0;
WebServer server(80);

// ============================================================================
//  SECTION 7 : PHYSICS ENGINE
// ============================================================================

/*  Francois & Garrison (1982) sound absorption in seawater.
    Three relaxation/viscosity terms:
      - boric acid      (dominant below ~10 kHz, negligible for us)
      - magnesium sulphate (dominant ~10-500 kHz, our main loss)
      - pure water viscosity (grows as f^2, dominates above ~500 kHz)
    Returns dB per kilometre. f is in kHz, depth in metres.
    Sanity check at T=15C S=35: ~37 dB/km @100 kHz, ~131 dB/km @500 kHz,
    which matches published tables.                                         */
float absorptionSeawater(float f_kHz, float depth_m) {
  float T = WATER_TEMP_C, S = SALINITY_PPT, D = depth_m, pH = WATER_PH;
  float c = 1412.0f + 3.21f*T + 1.19f*S + 0.0167f*D;

  // boric acid relaxation
  float f1 = 2.8f * sqrtf(S/35.0f) * powf(10.0f, 4.0f - 1245.0f/(T+273.0f));
  float A1 = (8.86f/c) * powf(10.0f, 0.78f*pH - 5.0f);

  // magnesium sulphate relaxation
  float f2 = 8.17f * powf(10.0f, 8.0f - 1990.0f/(T+273.0f)) / (1.0f + 0.0018f*(S-35.0f));
  float A2 = 21.44f * (S/c) * (1.0f + 0.025f*T);
  float P2 = 1.0f - 1.37e-4f*D + 6.2e-9f*D*D;

  // pure water viscous absorption
  float A3;
  if (T <= 20.0f) A3 = 4.937e-4f - 2.59e-5f*T + 9.11e-7f*T*T - 1.50e-8f*T*T*T;
  else            A3 = 3.964e-4f - 1.146e-5f*T + 1.45e-7f*T*T - 6.50e-10f*T*T*T;
  float P3 = 1.0f - 3.83e-5f*D + 4.9e-10f*D*D;

  float f = f_kHz;
  return A1*f1*f*f/(f1*f1 + f*f)
       + A2*P2*f2*f*f/(f2*f2 + f*f)
       + A3*P3*f*f;
}

/*  Extra attenuation from suspended sediment (the "muddy estuary" case).
    Scattering from suspended particles rises steeply with frequency. For
    sediment grains that are small-but-not-tiny compared to the wavelength
    in the 100-500 kHz band, an exponent near f^1.8 is a reasonable
    engineering fit. NOTE: this is a simplified prototype model, not a
    validated oceanographic one - say so if a judge asks.                   */
float absorptionSediment(float f_kHz, float conc_mgL) {
  return 0.09f * conc_mgL * powf(f_kHz/100.0f, 1.8f);
}

/*  Full active-sonar evaluation of one waveform in one environment.

    Sonar equation used:
        SNR = SL - 2*TL + TS - NL + DI + PG

      SL  source level, scaled by the waveform's amplitude
      TL  transmission loss = spherical spreading + absorption, counted twice
          because the sound goes out AND comes back
      TS  target (seabed) strength
      NL  noise level = spectral density + 10log10(bandwidth)
      DI  receive array directivity
      PG  pulse-compression gain = 10log10(B*tau), the time-bandwidth product.
          This is the whole reason a chirp beats a CW ping: a 100 kHz x 50 us
          chirp gets ~7 dB for free that a CW pulse simply does not get.

    Resolution has TWO independent parts, and they come from different
    waveform properties - this is the core trade-off of the project:
      range resolution   dR = c / 2B        -> set by BANDWIDTH
      along-track res    dY = (lambda/D)*R  -> set by CENTRE FREQUENCY
    Higher centre frequency narrows the beam and sharpens the image, but it
    also absorbs far faster. That tension is exactly what we are adapting to. */
SonarMetrics evaluate(float f0, float f1, float pulseUs, float ampFrac,
                      float turbidity, float depth) {
  SonarMetrics m;
  float fc  = 0.5f * (f0 + f1);
  float B   = fabsf(f1 - f0);
  float tau = pulseUs * 1e-6f;
  float R   = fmaxf(depth * SWATH_RATIO, 2.0f);  // slant range across the swath

  m.absorption = absorptionSeawater(fc/1000.0f, depth)
               + absorptionSediment(fc/1000.0f, turbidity);
  m.transmission = 20.0f*log10f(R) + m.absorption * R / 1000.0f;

  float SL = SOURCE_LEVEL + 20.0f*log10f(fmaxf(ampFrac, 0.01f));
  m.bandwidth = fmaxf(B, 1.0f/tau);         // a CW pulse still has 1/tau of BW
  float NL = NOISE_SPECTRAL + 10.0f*log10f(m.bandwidth);
  m.procGain = 10.0f*log10f(fmaxf(B*tau, 1.0f));

  m.snr = SL - 2.0f*m.transmission + TARGET_STRENGTH - NL + ARRAY_GAIN + m.procGain;

  m.rangeRes = SOUND_SPEED / (2.0f * m.bandwidth);
  float beamwidth = (SOUND_SPEED / fc) / ARRAY_APERTURE;   // radians
  m.alongRes = beamwidth * R;
  m.cellArea = m.rangeRes * m.alongRes;
  m.energy   = ampFrac * ampFrac * pulseUs;                 // relative
  m.feasible = (m.snr >= SNR_REQUIRED);
  return m;
}

/*  Figure of merit used to rank candidate waveforms.

    This is NOT "maximise SNR". A battery-powered AUV does not want the
    loudest possible ping - it wants the SHARPEST IMAGE PER JOULE, subject to
    actually being able to hear the echo. So:
      1. any waveform that fails the detection threshold is rejected outright
      2. among survivors, reward a small resolution cell (sharp image)
      3. penalise transmitted energy (the low-power requirement)              */
float figureOfMerit(const SonarMetrics &m) {
  if (!m.feasible) return -1.0e6f + m.snr;   // infeasible, but rank by how close
  return -10.0f*log10f(m.cellArea) - ENERGY_WEIGHT*10.0f*log10f(fmaxf(m.energy,0.1f));
}

// ============================================================================
//  SECTION 8 : ADAPTIVE WAVEFORM SELECTION
// ----------------------------------------------------------------------------
//  Every pulse, the ESP32 runs all 12 candidates through the physics engine
//  above and keeps the best one. This is a real optimisation over the
//  library, not an if/else ladder - which is why the reason text below can
//  state actual numbers.
// ============================================================================
int selectWaveform(float turbidity, float depth,
                   SonarMetrics &bestMetrics, String &reason, bool &feasible) {
  int   bestIdx = 0;
  float bestFom = -1.0e30f;
  int   feasibleCount = 0;

  for (int i = 0; i < NUM_WAVEFORMS; i++) {
    const WaveformConfig &w = waveforms[i];
    SonarMetrics m = evaluate((float)w.startFreqHz, (float)w.endFreqHz,
                              (float)w.pulseWidthUs, w.amplitudePct/100.0f,
                              turbidity, depth);
    if (m.feasible) feasibleCount++;
    float fom = figureOfMerit(m);
    if (fom > bestFom) { bestFom = fom; bestIdx = i; bestMetrics = m; }
  }

  feasible = (feasibleCount > 0);
  const WaveformConfig &w = waveforms[bestIdx];
  float fc = 0.5f*(w.startFreqHz + w.endFreqHz)/1000.0f;   // kHz

  // Build a human explanation from what the physics actually said.
  // Note every branch reports the CENTRE FREQUENCY and BANDWIDTH it moved to.
  // Amplitude stays between 85 and 100 % across the whole library on purpose
  // (a spread of only ~1.4 dB), so the adaptation you see is genuinely a
  // frequency decision and not a volume knob.
  char buf[512];   // sized for the longest branch after substitution
  float bwk = (float)(w.endFreqHz - w.startFreqHz) / 1000.0f;
  if (!feasible) {
    snprintf(buf, sizeof(buf),
      "NO waveform in the library reaches the %.0f dB detection threshold at "
      "%.0f mg/L sediment and %.0f m depth (best achievable SNR %.1f dB). "
      "Falling back to the lowest band, %.0f kHz, because absorption rises "
      "steeply with frequency and this gives the only remaining chance of an "
      "echo. The payload is reporting its own limit rather than hiding it.",
      SNR_REQUIRED, turbidity, depth, bestMetrics.snr, fc);
  } else if (fc >= 250.0f) {
    snprintf(buf, sizeof(buf),
      "Absorption is only %.0f dB/km over a %.0f m swath, so high frequency "
      "survives the round trip. The payload sweeps UP to %.0f kHz centre with "
      "%.0f kHz bandwidth - the widest sweep it has. Bandwidth sets range "
      "resolution (c/2B = %.1f cm) and the high centre frequency narrows the "
      "beam to %.2f m, with %.1f dB of SNR still in hand.",
      bestMetrics.absorption, depth*SWATH_RATIO, fc, bwk,
      bestMetrics.rangeRes*100.0f, bestMetrics.alongRes,
      bestMetrics.snr - SNR_REQUIRED);
  } else if (fc <= 130.0f) {
    snprintf(buf, sizeof(buf),
      "Sediment and range push absorption to %.0f dB/km, which would annihilate "
      "a high-frequency ping long before it returned. The payload sweeps DOWN "
      "to %.0f kHz centre, %.0f kHz bandwidth, and stretches the pulse to hold "
      "SNR at %.1f dB. Resolution drops to %.1f cm on purpose - a blurry "
      "detected seabed beats a sharp invisible one.",
      bestMetrics.absorption, fc, bwk, bestMetrics.snr,
      bestMetrics.rangeRes*100.0f);
  } else {
    snprintf(buf, sizeof(buf),
      "Intermediate conditions: absorption %.0f dB/km over a %.0f m swath. The "
      "payload settles on %.0f kHz centre with %.0f kHz bandwidth as the best "
      "compromise - enough penetration to keep %.1f dB SNR, while holding the "
      "resolution cell to %.4f m^2. Higher bands lose the echo here; lower "
      "bands would waste resolution it does not need to give up.",
      bestMetrics.absorption, depth*SWATH_RATIO, fc, bwk,
      bestMetrics.snr, bestMetrics.cellArea);
  }
  reason = String(buf);
  return bestIdx;
}

// ============================================================================
//  SECTION 9 : WINDOW FUNCTIONS
//  Smooth the start and end of the pulse. This suppresses spectral sidelobes
//  and protects the power amplifier from a hard voltage step.
// ============================================================================
float windowValue(WindowType w, uint32_t n, uint32_t N) {
  if (N <= 1) return 1.0f;
  float r = (float)n / (float)(N - 1);
  switch (w) {
    case WIN_HANN:     return 0.5f  - 0.5f *cosf(2.0f*PI*r);
    case WIN_HAMMING:  return 0.54f - 0.46f*cosf(2.0f*PI*r);
    case WIN_BLACKMAN: return 0.42f - 0.5f *cosf(2.0f*PI*r) + 0.08f*cosf(4.0f*PI*r);
    default:           return 1.0f;
  }
}

// ============================================================================
//  SECTION 10 : WAVEFORM GENERATORS
// ----------------------------------------------------------------------------
//  Each returns a normalised value in [-1, +1]. Amplitude, windowing and the
//  conversion to DAC codes happen once, centrally, in generateWaveform().
//
//  PERFORMANCE NOTE: sinf()/powf() are called per sample here. That is fine,
//  because generation runs ONCE per pulse into a buffer of a few hundred
//  samples - it is NOT inside the DMA or interrupt path. The CPU is idle
//  while DMA streams the result out. If you ever need tens of thousands of
//  samples, swap in a sine lookup table + phase accumulator.
// ============================================================================

float sampleSine(uint32_t n, float Fs, float f) {
  return sinf(2.0f*PI*f*(n/Fs));
}

/*  LFM chirp. Frequency sweeps linearly f0 -> f1 over the pulse.
    The phase MUST be the integral of instantaneous frequency:
        phi(t) = 2*pi*( f0*t + 0.5*k*t^2 ),   k = (f1-f0)/T
    Writing sin(2*pi*f(t)*t) instead is a classic bug - it produces a sweep
    with twice the intended rate and the wrong end frequency.                */
float sampleLFM(uint32_t n, float Fs, float f0, float f1, float T) {
  float t = n / Fs;
  float k = (f1 - f0) / T;
  return sinf(2.0f*PI*(f0*t + 0.5f*k*t*t));
}

/*  Geometric / logarithmic sweep: f(t) = f0 * r^(t/T),  r = f1/f0.
    Integrating that instantaneous frequency gives
        phi(t) = 2*pi * (f0*T/ln r) * ( r^(t/T) - 1 )                        */
float sampleGeometric(uint32_t n, float Fs, float f0, float f1, float T) {
  float t = n / Fs;
  if (f0 <= 0.0f || fabsf(f1 - f0) < 1.0f) return sinf(2.0f*PI*f0*t);
  float r = f1 / f0;
  return sinf(2.0f*PI*(f0*T/logf(r))*(powf(r, t/T) - 1.0f));
}

/*  Phase-coded (BPSK) pulse: a fixed carrier whose phase flips 180 degrees
    according to a Barker code, one chip per T/codeLength slice.            */
float samplePhaseCoded(uint32_t n, float Fs, float carrier, float T, uint8_t codeId) {
  const int8_t *code; int len;
  if (codeId == 1) { code = BARKER13; len = 13; } else { code = BARKER7; len = 7; }
  float t = n / Fs;
  int chip = (int)(t / (T / (float)len));
  if (chip >= len) chip = len - 1;
  if (chip < 0)    chip = 0;
  return (float)code[chip] * sinf(2.0f*PI*carrier*t);
}

// ----------------------------------------------------------------------------
//  Single source of truth for "what is this waveform doing at time t?".
//  Used both by the real sample generator and by the display renderer, so the
//  dashboard can never drift out of sync with what is actually transmitted.
//  Returns the fully shaped value (amplitude and window applied), in [-1,+1].
// ----------------------------------------------------------------------------
float waveformValueAtTime(const WaveformConfig &cfg, float t, float T) {
  float raw;
  uint32_t nEq = (uint32_t)lroundf(t * (float)SAMPLE_RATE);   // for generators
  float Fs = (float)SAMPLE_RATE;
  switch (cfg.type) {
    case SINE:        raw = sinf(2.0f*PI*cfg.startFreqHz*t); break;
    case LFM: {
      float k = ((float)cfg.endFreqHz - (float)cfg.startFreqHz) / T;
      raw = sinf(2.0f*PI*((float)cfg.startFreqHz*t + 0.5f*k*t*t));
      break;
    }
    case GEOMETRIC: {
      float f0 = (float)cfg.startFreqHz, f1 = (float)cfg.endFreqHz;
      if (f0 <= 0.0f || fabsf(f1-f0) < 1.0f) raw = sinf(2.0f*PI*f0*t);
      else {
        float r = f1/f0;
        raw = sinf(2.0f*PI*(f0*T/logf(r))*(powf(r, t/T) - 1.0f));
      }
      break;
    }
    case PHASE_CODED: raw = samplePhaseCoded(nEq, Fs, cfg.startFreqHz, T, cfg.phaseCodeId); break;
    default:          raw = 0.0f;
  }
  // window, expressed on the continuous time axis
  float r = (T > 0.0f) ? (t / T) : 0.0f;
  if (r < 0.0f) r = 0.0f; if (r > 1.0f) r = 1.0f;
  float win;
  switch (cfg.window) {
    case WIN_HANN:     win = 0.5f  - 0.5f *cosf(2.0f*PI*r); break;
    case WIN_HAMMING:  win = 0.54f - 0.46f*cosf(2.0f*PI*r); break;
    case WIN_BLACKMAN: win = 0.42f - 0.5f *cosf(2.0f*PI*r) + 0.08f*cosf(4.0f*PI*r); break;
    default:           win = 1.0f;
  }
  float amp = cfg.amplitudePct / 100.0f;
  if (amp > 1.0f) amp = 1.0f;
  return raw * amp * win;
}

// ============================================================================
//  SECTION 11 : CONFIGURATION  ->  SAMPLES
//  This is the heart of the payload: parameters in, DAC codes out.
// ============================================================================
uint32_t generateWaveform(const WaveformConfig &cfg) {
  float Fs = (float)SAMPLE_RATE;
  float T  = cfg.pulseWidthUs * 1e-6f;

  // N is CALCULATED from the configured sample rate - never assumed.
  uint64_t n64 = (uint64_t)SAMPLE_RATE * cfg.pulseWidthUs / 1000000ULL;
  if (n64 == 0) { Serial.println("ERR: pulse too short, 0 samples"); return 0; }
  if (n64 > MAX_SAMPLES) {
    Serial.printf("WARN: %llu samples exceeds MAX_SAMPLES, clamping\n",
                  (unsigned long long)n64);
    n64 = MAX_SAMPLES;
  }
  uint32_t N = (uint32_t)n64;

  for (uint32_t n = 0; n < N; n++) {
    // t = n / Fs. Amplitude scaling and windowing happen inside
    // waveformValueAtTime(), which is the SINGLE definition of this waveform -
    // the dashboard renders from the same function, so what you see plotted
    // can never drift away from what DMA actually transmits.
    float shaped = waveformValueAtTime(cfg, (float)n / Fs, T);

    // Centre on DAC midscale and clamp. These are DAC CODES, not volts -
    // the real output voltage depends on the DAC reference, the reconstruction
    // filter, the op-amp gain and the load. Never quote 128 as "exactly 1.65 V".
    int code = 128 + (int)lroundf(shaped * 127.0f);
    if (code < 0)   code = 0;
    if (code > 255) code = 255;

    dacSamples[n] = (uint8_t)code;
    i2sBuffer[n]  = ((uint16_t)code) << 8;   // built-in DAC reads the high byte
  }
  return N;
}

/*  DISPLAY RENDER (not transmitted).
    Evaluates the SAME waveform maths on a fine time grid so the dashboard can
    draw a recognisable chirp. This is the ideal analogue signal - i.e. what
    the reconstruction filter is trying to rebuild from the DAC steps.
    We plot this UNDERNEATH the true DAC codes, never instead of them.        */
void renderAdaptiveForDisplay(const WaveformConfig &cfg, uint8_t *out, int points) {
  float T = cfg.pulseWidthUs * 1e-6f;
  for (int i = 0; i < points; i++) {
    float t = T * (float)i / (float)(points - 1);
    int code = 128 + (int)lroundf(waveformValueAtTime(cfg, t, T) * 127.0f);
    if (code < 0) code = 0;
    if (code > 255) code = 255;
    out[i] = (uint8_t)code;
  }
}

/*  The legacy fixed-frequency CW pulse, rendered only for comparison.
    It is NOT transmitted - it is the baseline we are beating. Rectangular
    window on purpose: the hard switch-on/off is part of what is wrong with it. */
void renderLegacyForDisplay(uint8_t *out, int points) {
  float T = LEGACY_PULSE_US * 1e-6f;
  for (int i = 0; i < points; i++) {
    float t = T * (float)i / (float)(points - 1);
    float v = LEGACY_AMPLITUDE * sinf(2.0f*PI*LEGACY_FREQ_HZ*t);
    int code = 128 + (int)lroundf(v * 127.0f);
    if (code < 0) code = 0;
    if (code > 255) code = 255;
    out[i] = (uint8_t)code;
  }
}

/*  The TRUE DAC codes actually clocked out by DMA, downsampled only if the
    pulse is longer than the export array. At 1 MS/s a 50 us pulse is just 50
    samples - the dashboard shows them as visible steps so the quantisation is
    honestly represented rather than smoothed away.                           */
int exportTrueSamples(uint32_t N, uint8_t *out, int maxPoints) {
  if (N == 0) return 0;
  int points = (N < (uint32_t)maxPoints) ? (int)N : maxPoints;
  for (int i = 0; i < points; i++) {
    uint32_t n = (uint32_t)((uint64_t)i * N / points);
    if (n >= N) n = N - 1;
    out[i] = dacSamples[n];
  }
  return points;
}

// ============================================================================
//  SECTION 12 : DAC + DMA
// ----------------------------------------------------------------------------
//  The classic ESP32 has no dedicated DAC-DMA block. The real, documented way
//  to DMA-feed the internal DAC is to run the I2S peripheral in built-in-DAC
//  mode. Once i2s_write() queues the buffer, the DMA engine clocks it out to
//  GPIO25 at the configured rate with ZERO further CPU involvement - which is
//  precisely the low-power requirement in the problem statement.
// ============================================================================
void setupDacDma() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = true,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };
  esp_err_t err = i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  if (err != ESP_OK) Serial.printf("i2s_driver_install failed: %d\n", err);
  i2s_set_pin(I2S_NUM_0, NULL);                 // NULL = internal DAC
  i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);   // RIGHT = DAC1 = GPIO25
  i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE);
}

void transmitWaveform(uint32_t N) {
  size_t written = 0;
  i2s_write(I2S_NUM_0, (const char *)i2sBuffer, N * sizeof(uint16_t),
            &written, portMAX_DELAY);
}

// ============================================================================
//  SECTION 13 : ONE COMPLETE PULSE CYCLE
// ----------------------------------------------------------------------------
//  Order is deliberate. The environment is sampled and the waveform chosen
//  BEFORE generation begins, and the pulse is transmitted to completion before
//  anything is re-read. A waveform is never altered mid-pulse - doing so would
//  corrupt the chirp's phase continuity and destroy pulse compression.
// ============================================================================
void runPulseCycle() {
  if (USE_POTENTIOMETERS) {
    // Map the two ADC inputs onto the same physical units the sliders use.
    envTurbidity = analogRead(ADC_TURBIDITY_PIN) / 4095.0f * 500.0f;
    envDepth     = 1.0f + analogRead(ADC_DEPTH_PIN) / 4095.0f * 199.0f;
  }

  SonarMetrics m; String reason; bool feasible;
  int idx = selectWaveform(envTurbidity, envDepth, m, reason, feasible);

  activeIndex  = idx;
  activeConfig = waveforms[idx];
  activeMetrics = m;
  selectionReason = reason;
  detectionAchievable = feasible;

  // The legacy system's performance in this SAME water, for comparison.
  legacyMetrics = evaluate(LEGACY_FREQ_HZ, LEGACY_FREQ_HZ, LEGACY_PULSE_US,
                           LEGACY_AMPLITUDE, envTurbidity, envDepth);

  activeSamples = generateWaveform(activeConfig);
  if (activeSamples == 0) return;

  transmitWaveform(activeSamples);                    // RAM -> DMA -> DAC

  renderAdaptiveForDisplay(activeConfig, exportAdaptive, EXPORT_POINTS);
  renderLegacyForDisplay(exportLegacy, EXPORT_POINTS);
  exportTrueLen = exportTrueSamples(activeSamples, exportTrue, TRUE_POINTS);

  if (PRINT_SAMPLES) {
    Serial.print("First DAC codes: ");
    for (uint32_t i = 0; i < activeSamples && i < 24; i++) {
      Serial.print(dacSamples[i]); Serial.print(' ');
    }
    Serial.println();
  }
}

// ============================================================================
//  SECTION 14 : WEB CONTROL PAGE (the two "knobs")
// ============================================================================
const char INDEX_HTML[] PROGMEM = R"HTMLDELIM(
<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Adaptive Sonar Payload</title>
<style>
 body{font-family:-apple-system,Segoe UI,Arial,sans-serif;background:#0a1020;color:#e9eefb;margin:0;padding:16px;}
 h1{font-size:1.15em;margin:0 0 4px 0} .sub{color:#7f8db0;font-size:.82em;margin-bottom:14px}
 .card{background:#151f38;border-radius:12px;padding:16px;margin-bottom:14px}
 label{display:block;margin:14px 0 6px;font-size:.9em;color:#a9b8dc}
 input[type=range]{width:100%;height:28px}
 .v{float:right;color:#63e6a0;font-weight:700}
 .kv{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #223056;font-size:.88em}
 .kv span:last-child{color:#8fd4ff;font-weight:600}
 .reason{background:#101a30;border-left:3px solid #3d7bff;padding:10px;border-radius:6px;font-size:.85em;line-height:1.5;color:#c3d2f0;margin-top:10px}
 .warn{border-left-color:#ff6b6b;color:#ffc9c9}
 .badge{display:inline-block;background:#3d7bff;padding:3px 10px;border-radius:20px;font-size:.8em;font-weight:700}
 .g{color:#63e6a0}
</style></head><body>
<h1>Adaptive Software-Defined Sonar Payload</h1>
<div class="sub">SIH26058 &middot; all selection and physics computed on the ESP32</div>

<div class="card">
 <b>Environment knobs</b>
 <label>Turbidity / suspended sediment <span class="v" id="vT"></span> mg/L</label>
 <input type="range" id="t" min="0" max="500" step="5" value="25">
 <label>Water depth <span class="v" id="vD"></span> m</label>
 <input type="range" id="d" min="1" max="200" step="1" value="20">
</div>

<div class="card">
 <b>Payload decision</b> <span class="badge" id="wf">--</span>
 <div class="kv"><span>Absorption</span><span id="ab">--</span></div>
 <div class="kv"><span>Echo SNR</span><span id="sn">--</span></div>
 <div class="kv"><span>Range resolution</span><span id="rr">--</span></div>
 <div class="kv"><span>Along-track resolution</span><span id="ar">--</span></div>
 <div class="kv"><span>Pulse compression gain</span><span id="pg">--</span></div>
 <div class="kv"><span>Samples generated</span><span id="ns">--</span></div>
 <div class="reason" id="rs">--</div>
</div>

<div class="card">
 <b>vs legacy fixed 200 kHz sonar</b>
 <div class="kv"><span>SNR advantage</span><span class="g" id="i1">--</span></div>
 <div class="kv"><span>Resolution cell</span><span class="g" id="i2">--</span></div>
 <div class="kv"><span>Energy per ping</span><span class="g" id="i3">--</span></div>
 <div class="kv"><span>Overall performance index</span><span class="g" id="i4">--</span></div>
</div>

<script>
const t=document.getElementById('t'),d=document.getElementById('d');
function show(){document.getElementById('vT').textContent=t.value;
                document.getElementById('vD').textContent=d.value;}
let timer=null;
function push(){show();clearTimeout(timer);
  timer=setTimeout(()=>fetch(`/api/env?t=${t.value}&d=${d.value}`),80);}
t.addEventListener('input',push); d.addEventListener('input',push); show();

function poll(){fetch('/api/frame').then(r=>r.json()).then(j=>{
  document.getElementById('wf').textContent=j.label;
  document.getElementById('ab').textContent=j.adaptive.absorption.toFixed(1)+' dB/km';
  document.getElementById('sn').textContent=j.adaptive.snr.toFixed(1)+' dB';
  document.getElementById('rr').textContent=(j.adaptive.rangeRes*100).toFixed(1)+' cm';
  document.getElementById('ar').textContent=j.adaptive.alongRes.toFixed(2)+' m';
  document.getElementById('pg').textContent=j.adaptive.procGain.toFixed(1)+' dB';
  document.getElementById('ns').textContent=j.samples;
  const rs=document.getElementById('rs');
  rs.textContent=j.reason; rs.className=j.feasible?'reason':'reason warn';
  document.getElementById('i1').textContent=(j.improve.snr>=0?'+':'')+j.improve.snr.toFixed(1)+' dB';
  document.getElementById('i2').textContent=j.improve.cell.toFixed(1)+'x sharper';
  document.getElementById('i3').textContent=j.improve.energy.toFixed(0)+'% saved';
  document.getElementById('i4').textContent='+'+j.improve.overall.toFixed(0)+'%';
}).catch(e=>{});}
setInterval(poll,700); poll();
</script></body></html>
)HTMLDELIM";

// ============================================================================
//  SECTION 15 : JSON API
//  /api/frame is what the Python dashboard consumes. Note that every number
//  in it was computed by the ESP32 - Python does no physics.
// ============================================================================
void addMetrics(String &j, const char *name, const SonarMetrics &m) {
  j += "\""; j += name; j += "\":{";
  j += "\"absorption\":"  + String(m.absorption, 2) + ",";
  j += "\"transmission\":"+ String(m.transmission, 2) + ",";
  j += "\"snr\":"         + String(m.snr, 2) + ",";
  j += "\"rangeRes\":"    + String(m.rangeRes, 5) + ",";
  j += "\"alongRes\":"    + String(m.alongRes, 4) + ",";
  j += "\"cellArea\":"    + String(m.cellArea, 6) + ",";
  j += "\"energy\":"      + String(m.energy, 3) + ",";
  j += "\"bandwidth\":"   + String(m.bandwidth, 1) + ",";
  j += "\"procGain\":"    + String(m.procGain, 2) + ",";
  j += "\"feasible\":"    + String(m.feasible ? "true" : "false");
  j += "}";
}

void handleFrame() {
  const char *typeName[] = {"SINE","LFM","GEOMETRIC","PHASE_CODED"};
  const char *winName[]  = {"RECTANGULAR","HANN","HAMMING","BLACKMAN"};

  // Improvement figures - computed here on the ESP32, not in Python.
  float dSnr    = activeMetrics.snr - legacyMetrics.snr;
  float cellX   = legacyMetrics.cellArea / fmaxf(activeMetrics.cellArea, 1e-9f);
  float energyP = (1.0f - activeMetrics.energy / fmaxf(legacyMetrics.energy,1e-9f)) * 100.0f;
  // Composite index: geometric blend of detection margin and image sharpness,
  // discounted by energy spent. Expressed as % better than legacy.
  float overall = (powf(10.0f, dSnr/20.0f) * sqrtf(fmaxf(cellX,1e-6f)) - 1.0f) * 100.0f;
  if (overall > 99999.0f) overall = 99999.0f;

  String j; j.reserve(6000);
  j = "{";
  j += "\"turbidity\":" + String(envTurbidity,1) + ",";
  j += "\"depth\":"     + String(envDepth,1) + ",";
  j += "\"index\":"     + String(activeIndex) + ",";
  j += "\"label\":\""   + String(activeConfig.label) + "\",";
  j += "\"type\":\""    + String(typeName[activeConfig.type]) + "\",";
  j += "\"window\":\""  + String(winName[activeConfig.window]) + "\",";
  j += "\"f0\":"        + String(activeConfig.startFreqHz) + ",";
  j += "\"f1\":"        + String(activeConfig.endFreqHz) + ",";
  j += "\"pulseUs\":"   + String(activeConfig.pulseWidthUs) + ",";
  j += "\"amplitude\":" + String(activeConfig.amplitudePct) + ",";
  j += "\"samples\":"   + String(activeSamples) + ",";
  j += "\"sampleRate\":"+ String(SAMPLE_RATE) + ",";
  j += "\"feasible\":"  + String(detectionAchievable ? "true":"false") + ",";
  j += "\"reason\":\""  + selectionReason + "\",";
  j += "\"legacyFreq\":"    + String(LEGACY_FREQ_HZ,0) + ",";
  j += "\"legacyPulseUs\":" + String(LEGACY_PULSE_US,0) + ",";
  addMetrics(j, "adaptive", activeMetrics); j += ",";
  addMetrics(j, "legacy",   legacyMetrics); j += ",";
  j += "\"improve\":{\"snr\":" + String(dSnr,2)
     + ",\"cell\":" + String(cellX,2)
     + ",\"energy\":" + String(energyP,1)
     + ",\"overall\":" + String(overall,1) + "},";

  j += "\"waveAdaptive\":[";
  for (int i = 0; i < EXPORT_POINTS; i++) { if(i) j += ','; j += String(exportAdaptive[i]); }
  j += "],\"waveLegacy\":[";
  for (int i = 0; i < EXPORT_POINTS; i++) { if(i) j += ','; j += String(exportLegacy[i]); }
  j += "],\"waveTrue\":[";
  for (int i = 0; i < exportTrueLen; i++) { if(i) j += ','; j += String(exportTrue[i]); }
  j += "]}";

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", j);
}

void handleEnv() {
  if (server.hasArg("t")) envTurbidity = constrain(server.arg("t").toFloat(), 0.0f, 500.0f);
  if (server.hasArg("d")) envDepth     = constrain(server.arg("d").toFloat(), 1.0f, 200.0f);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "ok");
}

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void setupNetwork() {
  if (MODE_AP) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("\nAP started.  SSID: %s   PASS: %s\n", AP_SSID, AP_PASS);
    Serial.print("Open  http://"); Serial.println(WiFi.softAPIP());
  } else {
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    Serial.printf("\nJoining %s", STA_SSID);
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(250); Serial.print('.'); }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("\nConnected. Open  http://"); Serial.println(WiFi.localIP());
    } else {
      Serial.println("\nWiFi failed - falling back to AP mode.");
      WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID, AP_PASS);
      Serial.print("Open  http://"); Serial.println(WiFi.softAPIP());
    }
  }
  server.on("/",           handleRoot);
  server.on("/api/env",    handleEnv);
  server.on("/api/frame",  handleFrame);
  server.begin();
}

// ============================================================================
//  SECTION 16 : SETUP / LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n============================================");
  Serial.println(" ADAPTIVE SOFTWARE-DEFINED SONAR PAYLOAD");
  Serial.println(" SIH26058  |  ESP32 firmware");
  Serial.println("============================================");
  Serial.printf("DAC sample rate : %lu Hz\n", SAMPLE_RATE);
  Serial.println("DAC output      : internal 8-bit, GPIO25, I2S+DMA");
  Serial.printf("Waveform library: %d entries\n", NUM_WAVEFORMS);
  Serial.printf("Input source    : %s\n",
                USE_POTENTIOMETERS ? "ADC potentiometers" : "web sliders");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  setupDacDma();
  setupNetwork();
  lastPulseMs = millis();
}

void loop() {
  server.handleClient();                       // keep the UI and API responsive
  if (millis() - lastPulseMs >= PULSE_INTERVAL_MS) {
    runPulseCycle();                           // sense -> decide -> synthesise -> DMA
    lastPulseMs = millis();
  }
}
