#include <Arduino.h>
#include <avr/pgmspace.h>

// ---- Gamma table (from the published code) ----
const uint8_t PROGMEM gamma8[] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2,
  2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5,
  5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 9, 9, 9, 10,
  10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
  17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
  25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
  37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
  51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
  69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
  90, 92, 93, 95, 96, 98, 99, 101, 102, 104, 105, 107, 109, 110, 112, 114,
  115, 117, 119, 120, 122, 124, 126, 127, 129, 131, 133, 135, 137, 138, 140, 142,
  144, 146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 167, 169, 171, 173, 175,
  177, 180, 182, 184, 186, 189, 191, 193, 196, 198, 200, 203, 205, 208, 210, 213,
  215, 218, 220, 223, 225, 228, 231, 233, 236, 239, 241, 244, 247, 249, 252, 255
};

// ---- PWM pins ----
static const uint8_t LED_PIN_1 = 3; // D3  (Timer2 OC2B)  ~1.95kHz
static const uint8_t LED_PIN_2 = 9; // D9  (Timer1 OC1A)  2.0kHz (configured below)

// ---- Candle parameters ----
static float value = 0.10f;

static const int   max_value     = 255;
static const int   dimming_range = 50;   // 90 -> larger => stronger contrast (try 140..200)
static const float threshold     = 0.065f;

// Update interval jitter to avoid "regular rhythm"
static uint32_t nextUpdateMs = 0;

// ---- Timer1 settings (2.0kHz PWM) ----
// 16MHz / prescaler 8 = 2MHz timer clock
// 2kHz => period 0.5ms => 2MHz * 0.5ms = 1000 counts
// Use Fast PWM with TOP=ICR1=999 -> 1000 counts
static const uint16_t T1_TOP = 999;

// Map 0..255 duty (after gamma) to 0..T1_TOP
static inline uint16_t map255ToT1(uint8_t d) {
  return (uint32_t)d * T1_TOP / 255u;
}

// ---- Setup Timer2 on D3 to ~1.95kHz ----
static void setupTimer2PwmOnD3() {
  pinMode(LED_PIN_1, OUTPUT);

  TCCR2A = 0;
  TCCR2B = 0;

  // Fast PWM 8-bit, non-inverting on OC2B (D3)
  TCCR2A |= (1 << WGM21) | (1 << WGM20);
  TCCR2A |= (1 << COM2B1);

  // prescaler 32 -> ~1953 Hz
  TCCR2B |= (1 << CS21) | (1 << CS20);

  OCR2B = 0;
}

// ---- Setup Timer1 on D9 to exactly 2.0kHz ----
static void setupTimer1PwmOnD9() {
  pinMode(LED_PIN_2, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;

  // Fast PWM, TOP = ICR1 (Mode 14: WGM13:0 = 1110)
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // Non-inverting on OC1A (D9)
  TCCR1A |= (1 << COM1A1);

  // Prescaler = 8
  TCCR1B |= (1 << CS11);

  ICR1  = T1_TOP;
  OCR1A = 0;
}

static inline void writeCh1(uint8_t duty255) {
  // D3 uses OCR2B 0..255
  OCR2B = duty255;
}

static inline void writeCh2(uint8_t duty255) {
  // D9 uses OCR1A 0..999
  OCR1A = map255ToT1(duty255);
}

void setup() {
  setupTimer2PwmOnD3();
  setupTimer1PwmOnD9();

  // Seed RNG (A0 floating is OK; if tied, move to another analog pin)
  randomSeed(analogRead(A0));
  nextUpdateMs = millis();
}

void loop() {
  uint32_t now = millis();
  if ((int32_t)(now - nextUpdateMs) < 0) return;

  // ---- folded quadratic mapping (as published) ----
  if (value < 0.5f) {
    value = value + 2.0f * value * value;
  } else {
    float t = 1.0f - value;
    value = value - 2.0f * t * t;
  }

  // ---- edge re-injection (breaks periodicity) ----
  if (value <= (0.0f + threshold) || (1.0f - threshold) <= value) {
    long r = random((long)(threshold * 1000.0f), (long)(1000.0f - threshold * 1000.0f));
    value = (float)r / 1000.0f;
  }

  // ---- two channels, opposite phase (as published) ----
  int value_1 = max_value - dimming_range + (int)(value * dimming_range); // 255-range .. 255
  int value_2 = max_value - (int)(value * dimming_range);                 // 255 .. 255-range

  if (value_1 < 0) value_1 = 0; if (value_1 > 255) value_1 = 255;
  if (value_2 < 0) value_2 = 0; if (value_2 > 255) value_2 = 255;

  uint8_t duty1 = pgm_read_byte(&(gamma8[value_1]));
  uint8_t duty2 = pgm_read_byte(&(gamma8[value_2]));

  writeCh1(duty1);
  writeCh2(duty2);

  // ---- jitter update interval (reduces artificial rhythm) ----
  // Base 50..95ms + rare longer pause
  uint16_t dt = 80 + (uint16_t)random(0, 46); // 50..95
  if (random(0, 3) == 0) {                   // ~1/28 chance
    dt += (uint16_t)random(40, 140);          // add 40..139
  }
  nextUpdateMs = now + dt;
}
