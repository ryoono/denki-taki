/*
 * ボード：Amazonで購入した「KEYESTUDIO 5V Pro Micro Atmega32U4」
 * コンパイル時のボード設定：Arduino Leonardo
 * 
 * LEDの揺らぎ表現は、以下のWeb記事で紹介されている手法を元に実装している。
 * アルゴリズム自体はオリジナルではなく、実装方法を参考にしている。
 * https://www.creativity-ape.com/entry/2020/12/16/232928
 * 
 * ソレノイドは500~10,000msのランダム秒ごとに駆動する。
 * ただし、過電流を防ぐため片方ずつしか駆動しないようソフトウェアでブロックしている。
 * 
 * D5：
 *   LED揺らぎ用PWM出力（CH1）
 *   - Timer3 / OC3A
 *   - Fast PWM（TOP=999, prescaler=8）
 *   - gamma補正後のdutyを出力
 * 
 * D9：
 *   LED揺らぎ用PWM出力（CH2）
 *   - Timer1 / OC1A
 *   - Fast PWM（TOP=999, prescaler=8）
 *   - D5と逆位相の揺らぎ表現
 * 
 * D2：
 *   デジタル出力（パルス制御）
 *   - 500～10000msのランダム待ち時間後にON
 *   - ON時間は40ms固定
 *   - 過電流防止のため、D4と同時ONしない
 * 
 * D4：
 *   デジタル出力（パルス制御）
 *   - 動作条件はD6と同じ
 *   - D2がON中の場合は、D2がOFFになるまでONを待機
*/

#include <Arduino.h>
#include <avr/pgmspace.h>

// ---- Gamma table ----
// 人間の目の明るさ感覚に合わせるための gamma 補正テーブル
// 0–255 の duty 値を、見た目が自然になるように変換する
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
// PWM出力に使用するピン
// D5 : Timer3 / OC3A
// D9 : Timer1 / OC1A
static const uint8_t LED_PIN_1 = 5; // D5  (Timer3 OC3A)
static const uint8_t LED_PIN_2 = 9; // D9  (Timer1 OC1A)

// ---- Digital output pins ----
// 単純な digital output 用ピン（PWMは使わない）
static const uint8_t DOUT_PIN_6 = 2; // D2
static const uint8_t DOUT_PIN_7 = 4; // D4

// ---- Digital output timing parameters ----
// デジタル出力のON時間と待ち時間の条件
static const uint16_t DOUT_ON_MS      = 40;     // ONする時間 [ms]
static const uint16_t DOUT_WAIT_MINMS = 500;    // 待ち時間の最小値 [ms]
static const uint16_t DOUT_WAIT_MAXMS = 10000;  // 待ち時間の最大値 [ms]

// ---- Digital output states ----
// D6 / D7 それぞれの状態管理用変数
static uint32_t d6NextMs = 0;
static uint32_t d6OffMs  = 0;
static bool     d6On     = false;

static uint32_t d7NextMs = 0;
static uint32_t d7OffMs  = 0;
static bool     d7On     = false;

// 待ち時間をランダムに生成するヘルパ関数
static inline uint16_t randWaitMs() {
  return (uint16_t)random((long)DOUT_WAIT_MINMS, (long)DOUT_WAIT_MAXMS + 1);
}

// ---- Candle parameters ----
// 揺らぎ計算用の内部状態
static float value = 0.10f;

static const int   max_value     = 255;
static const int   dimming_range = 50;   // 値を大きくするとコントラストが強くなる
static const float threshold     = 0.065f;

// 揺らぎ更新タイミング管理用
static uint32_t nextUpdateMs = 0;

// ---- Timer1 settings ----
// Timer1 を使って PWM 周波数を正確に設定する
// prescaler = 8
// TOP = 999 → 約2kHz
static const uint16_t T1_TOP = 999;

// gamma 補正後の 0–255 を Timer1 用のカウント値に変換
static inline uint16_t map255ToT1(uint8_t d) {
  return (uint32_t)d * T1_TOP / 255u;
}

// ---- Setup PWM on D5 (Timer3) ----
// Timer3 を使った PWM 出力設定
static void setupTimer2PwmOnD3() {
  pinMode(LED_PIN_1, OUTPUT);

  TCCR3A = 0;
  TCCR3B = 0;

  // Fast PWM, TOP = ICR3
  TCCR3A |= (1 << WGM31);
  TCCR3B |= (1 << WGM33) | (1 << WGM32);

  // Non-inverting mode on OC3A
  TCCR3A |= (1 << COM3A1);

  // prescaler = 8
  TCCR3B |= (1 << CS31);

  ICR3  = T1_TOP;
  OCR3A = 0;
}

// ---- Setup PWM on D9 (Timer1) ----
// Timer1 を使った PWM 出力設定
static void setupTimer1PwmOnD9() {
  pinMode(LED_PIN_2, OUTPUT);

  TCCR1A = 0;
  TCCR1B = 0;

  // Fast PWM, TOP = ICR1
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // Non-inverting mode on OC1A
  TCCR1A |= (1 << COM1A1);

  // prescaler = 8
  TCCR1B |= (1 << CS11);

  ICR1  = T1_TOP;
  OCR1A = 0;
}

// PWM duty 更新（D5）
static inline void writeCh1(uint8_t duty255) {
  OCR3A = map255ToT1(duty255);
}

// PWM duty 更新（D9）
static inline void writeCh2(uint8_t duty255) {
  OCR1A = map255ToT1(duty255);
}

void setup() {
  setupTimer2PwmOnD3();
  setupTimer1PwmOnD9();

  pinMode(DOUT_PIN_6, OUTPUT);
  pinMode(DOUT_PIN_7, OUTPUT);
  digitalWrite(DOUT_PIN_6, LOW);
  digitalWrite(DOUT_PIN_7, LOW);

  // A0 を使って乱数の初期化（未接続前提）
  randomSeed(analogRead(A0));
  nextUpdateMs = millis();

  d6NextMs = nextUpdateMs + (uint32_t)randWaitMs();
  d7NextMs = nextUpdateMs + (uint32_t)randWaitMs();
}

void loop() {
  uint32_t now = millis();

  // ---- D6 control ----
  // ランダム時間待ち → 40msだけON
  if (d6On) {
    if ((int32_t)(now - d6OffMs) >= 0) {
      digitalWrite(DOUT_PIN_6, LOW);
      d6On = false;
      d6NextMs = now + (uint32_t)randWaitMs();
    }
  } else {
    if ((int32_t)(now - d6NextMs) >= 0) {
      digitalWrite(DOUT_PIN_6, HIGH);
      d6On = true;
      d6OffMs = now + (uint32_t)DOUT_ON_MS;
    }
  }

  // ---- D7 control ----
  // D6 と同時に ON しないようにソフトウェアでブロックする
  if (d7On) {
    if ((int32_t)(now - d7OffMs) >= 0) {
      digitalWrite(DOUT_PIN_7, LOW);
      d7On = false;
      d7NextMs = now + (uint32_t)randWaitMs();
    }
  } else {
    if ((int32_t)(now - d7NextMs) >= 0) {
      if (d6On) {
        d7NextMs = d6OffMs;
      } else {
        digitalWrite(DOUT_PIN_7, HIGH);
        d7On = true;
        d7OffMs = now + (uint32_t)DOUT_ON_MS;
      }
    }
  }

  if ((int32_t)(now - nextUpdateMs) < 0) return;

  // ---- 揺らぎ計算 ----
  if (value < 0.5f) {
    value = value + 2.0f * value * value;
  } else {
    float t = 1.0f - value;
    value = value - 2.0f * t * t;
  }

  // 周期性を壊すため、端に来たら値を再注入
  if (value <= (0.0f + threshold) || (1.0f - threshold) <= value) {
    long r = random((long)(threshold * 1000.0f), (long)(1000.0f - threshold * 1000.0f));
    value = (float)r / 1000.0f;
  }

  // 2チャンネルを逆位相で駆動
  int value_1 = max_value - dimming_range + (int)(value * dimming_range);
  int value_2 = max_value - (int)(value * dimming_range);

  if (value_1 < 0) value_1 = 0; if (value_1 > 255) value_1 = 255;
  if (value_2 < 0) value_2 = 0; if (value_2 > 255) value_2 = 255;

  uint8_t duty1 = pgm_read_byte(&(gamma8[value_1]));
  uint8_t duty2 = pgm_read_byte(&(gamma8[value_2]));

  writeCh1(duty1);
  writeCh2(duty2);

  // 次回更新タイミングをランダムにずらす
  uint16_t dt = 80 + (uint16_t)random(0, 46);
  if (random(0, 3) == 0) {
    dt += (uint16_t)random(40, 140);
  }
  nextUpdateMs = now + dt;
}
