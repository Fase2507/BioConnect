#include <BleKeyboard.h>

BleKeyboard bleKeyboard("EMG Controller", "DIY", 100);

#define EMG_PIN 35
#define LO_PLUS 32
#define LO_MINUS 33

// ---- Calibration values (auto-set at startup, keep arm relaxed during calibration) ----
int baseline = 2048;

// ---- Thresholds — TUNE THESE after watching Serial output ----
int lowThreshold = 40;   // above this = light flex -> LEFT
int highThreshold = 100;  // above this = strong flex -> RIGHT

// ---- Envelope smoothing ----
const int SAMPLES = 15;
int buffer[SAMPLES];
int bufIndex = 0;

// ---- State tracking ----
enum State { IDLE,
             LEFT,
             RIGHT };
State currentState = IDLE;

unsigned long lastTrigger = 0;
const unsigned long DEBOUNCE_MS = 300;

void setup() {
  Serial.begin(115200);
  pinMode(LO_PLUS, INPUT);
  pinMode(LO_MINUS, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(EMG_PIN, ADC_11db);

  bleKeyboard.begin();

  Serial.println("Calibrating... keep arm RELAXED");
  delay(1000);
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    sum += analogRead(EMG_PIN);
    delay(5);
  }
  baseline = sum / 200;
  Serial.print("Baseline set to: ");
  Serial.println(baseline);
}

int getSmoothedEnvelope() {
  int raw = analogRead(EMG_PIN);
  int rectified = abs(raw - baseline);

  buffer[bufIndex] = rectified;
  bufIndex = (bufIndex + 1) % SAMPLES;

  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) sum += buffer[i];
  return sum / SAMPLES;
}

void releaseAllKeys() {
  bleKeyboard.release('a');
  bleKeyboard.release('d');
}

void loop() {
  if ((digitalRead(LO_PLUS) == 1) || (digitalRead(LO_MINUS) == 1)) {
    delay(5);
    return;
  }

  int envelope = getSmoothedEnvelope();
  Serial.println(envelope);  // watch this in Serial Plotter to set thresholds

  unsigned long now = millis();

  if (!bleKeyboard.isConnected()) {
    delay(5);
    return;
  }

  // Decide gesture based on intensity
  if (envelope > highThreshold) {
    if (currentState != RIGHT && (now - lastTrigger > DEBOUNCE_MS)) {
      releaseAllKeys();
      bleKeyboard.press('d');  // RIGHT
      currentState = RIGHT;
      lastTrigger = now;
      Serial.println(">>> RIGHT");
    }
  } else if (envelope > lowThreshold) {
    if (currentState != LEFT && (now - lastTrigger > DEBOUNCE_MS)) {
      releaseAllKeys();
      bleKeyboard.press('a');  // LEFT
      currentState = LEFT;
      lastTrigger = now;
      Serial.println(">>> LEFT");
    }
  } else {
    if (currentState != IDLE) {
      releaseAllKeys();
      currentState = IDLE;
      Serial.println(">>> IDLE");
    }
  }

  delay(2);
}