#include <BleKeyboard.h>
#include <math.h>

BleKeyboard bleKeyboard("BioConnect EMG", "BioConnect", 100);

#if defined(CONFIG_IDF_TARGET_ESP32S3) && CONFIG_IDF_TARGET_ESP32S3
constexpr uint8_t EMG_PIN = 4;       // ESP32-S3 ADC1-capable GPIO
constexpr uint8_t LO_PLUS_PIN = 5;
constexpr uint8_t LO_MINUS_PIN = 6;
#else
constexpr uint8_t EMG_PIN = 35;      // Original ESP32 ADC1 input-only GPIO
constexpr uint8_t LO_PLUS_PIN = 32;
constexpr uint8_t LO_MINUS_PIN = 33;
#endif

// Verify these pin choices against the exact board and wiring before flashing.
constexpr uint32_t SAMPLE_PERIOD_US = 2000;  // Target: 500 samples/second
constexpr uint8_t RMS_WINDOW = 50;           // 100 ms at 500 samples/second
constexpr uint8_t CALIBRATION_DECIMATION = 10;
constexpr uint8_t CALIBRATION_POINTS = 100;  // 2 seconds at 50 points/second
constexpr uint8_t IDLE_WINDOW = 31;          // 6.2 seconds at 5 updates/second
constexpr uint8_t CLIP_WINDOW = 50;

constexpr float DC_BASELINE_TAU_SECONDS = 2.0f;
constexpr float ENVELOPE_TAU_SECONDS = 0.04f;
constexpr float MIN_NOISE_SIGMA = 3.0f;       // ADC counts; prevents division by zero
constexpr float MAX_NOISE_SIGMA = 80.0f;      // Above this, report poor signal quality
constexpr float ACTIVATE_Z = 5.0f;
constexpr float RELEASE_Z = 3.0f;
constexpr float STRONG_Z = 12.0f;
constexpr float STRONG_RELEASE_Z = 8.0f;
constexpr uint32_t STATE_CONFIRM_MS = 30;
constexpr uint32_t NOISE_UPDATE_MS = 200;
constexpr uint32_t STATUS_PRINT_MS = 250;

float dcBaseline = 2048.0f;
float envelopeEma = 0.0f;
float rmsSquares[RMS_WINDOW] = {};
float rmsSquareSum = 0.0f;
uint8_t rmsIndex = 0;
uint8_t rmsCount = 0;

float calibrationValues[CALIBRATION_POINTS] = {};
float sortScratch[CALIBRATION_POINTS] = {};
uint8_t calibrationCount = 0;
uint8_t calibrationDecimator = 0;
bool calibrated = false;

float idleValues[IDLE_WINDOW] = {};
float idleScratch[IDLE_WINDOW] = {};
uint8_t idleCount = 0;
uint8_t idleNext = 0;
float noiseMedian = 0.0f;
float noiseMad = 0.0f;

uint8_t clipFlags[CLIP_WINDOW] = {};
uint8_t clipIndex = 0;
uint8_t clipCount = 0;

uint32_t lastSampleUs = 0;
uint32_t lastNoiseUpdateMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastSampleIntervalUs = SAMPLE_PERIOD_US;
uint32_t samplesSinceStart = 0;

bool wasConnected = false;
bool wasLeadOff = false;

enum class InputState : uint8_t { Idle, Light, Strong };
InputState currentState = InputState::Idle;
InputState candidateState = InputState::Idle;
uint32_t candidateSinceMs = 0;

float medianOf(float *values, uint8_t count) {
  if (count == 0) return 0.0f;

  for (uint8_t i = 0; i < count; ++i) {
    float value = values[i];
    int8_t j = static_cast<int8_t>(i) - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = value;
  }

  if ((count & 1U) != 0U) return values[count / 2];
  return (values[count / 2 - 1] + values[count / 2]) * 0.5f;
}

void resetSignalWindow() {
  for (uint8_t i = 0; i < RMS_WINDOW; ++i) rmsSquares[i] = 0.0f;
  for (uint8_t i = 0; i < CLIP_WINDOW; ++i) clipFlags[i] = 0;
  rmsSquareSum = 0.0f;
  rmsIndex = 0;
  rmsCount = 0;
  clipIndex = 0;
  clipCount = 0;
  envelopeEma = 0.0f;
}

void releaseKeys() {
  if (bleKeyboard.isConnected()) bleKeyboard.releaseAll();
  currentState = InputState::Idle;
  candidateState = InputState::Idle;
}

void beginCalibration() {
  releaseKeys();
  resetSignalWindow();
  calibrationCount = 0;
  calibrationDecimator = 0;
  idleCount = 0;
  idleNext = 0;
  noiseMedian = 0.0f;
  noiseMad = 0.0f;
  calibrated = false;
  Serial.println("Calibration: relax the target muscle for about 2 seconds.");
}

float noiseSigma() {
  const float robustSigma = 1.4826f * noiseMad;
  return robustSigma > MIN_NOISE_SIGMA ? robustSigma : MIN_NOISE_SIGMA;
}

float zScore(float envelope) {
  return (envelope - noiseMedian) / noiseSigma();
}

void updateNoiseModel() {
  for (uint8_t i = 0; i < idleCount; ++i) idleScratch[i] = idleValues[i];
  noiseMedian = medianOf(idleScratch, idleCount);

  for (uint8_t i = 0; i < idleCount; ++i) {
    idleScratch[i] = fabsf(idleValues[i] - noiseMedian);
  }
  noiseMad = medianOf(idleScratch, idleCount);
}

void finishCalibration() {
  for (uint8_t i = 0; i < calibrationCount; ++i) {
    sortScratch[i] = calibrationValues[i];
  }
  noiseMedian = medianOf(sortScratch, calibrationCount);

  for (uint8_t i = 0; i < calibrationCount; ++i) {
    sortScratch[i] = fabsf(calibrationValues[i] - noiseMedian);
  }
  noiseMad = medianOf(sortScratch, calibrationCount);

  const uint8_t seedCount = calibrationCount < IDLE_WINDOW
                                ? calibrationCount
                                : IDLE_WINDOW;
  for (uint8_t i = 0; i < seedCount; ++i) idleValues[i] = calibrationValues[i];
  idleCount = seedCount;
  idleNext = seedCount % IDLE_WINDOW;
  calibrated = true;

  Serial.print("Calibration complete; noise median=");
  Serial.print(noiseMedian, 1);
  Serial.print(" MAD=");
  Serial.println(noiseMad, 1);
}

void addIdleObservation(float envelope) {
  idleValues[idleNext] = envelope;
  idleNext = (idleNext + 1) % IDLE_WINDOW;
  if (idleCount < IDLE_WINDOW) ++idleCount;
  updateNoiseModel();
}

bool leadOff() {
  return digitalRead(LO_PLUS_PIN) == HIGH || digitalRead(LO_MINUS_PIN) == HIGH;
}

bool signalClipped() {
  return clipCount >= 3;
}

InputState desiredStateFor(float z) {
  switch (currentState) {
    case InputState::Idle:
      if (z >= STRONG_Z) return InputState::Strong;
      if (z >= ACTIVATE_Z) return InputState::Light;
      return InputState::Idle;

    case InputState::Light:
      if (z >= STRONG_Z) return InputState::Strong;
      if (z <= RELEASE_Z) return InputState::Idle;
      return InputState::Light;

    case InputState::Strong:
      if (z <= RELEASE_Z) return InputState::Idle;
      if (z <= STRONG_RELEASE_Z) return InputState::Light;
      return InputState::Strong;
  }
  return InputState::Idle;
}

const char *stateName(InputState state) {
  switch (state) {
    case InputState::Light: return "LIGHT";
    case InputState::Strong: return "STRONG";
    default: return "IDLE";
  }
}

void applyState(InputState nextState) {
  if (nextState == currentState) return;

  releaseKeys();
  currentState = nextState;
  candidateState = nextState;

  if (currentState == InputState::Light) {
    bleKeyboard.press('a');
  } else if (currentState == InputState::Strong) {
    bleKeyboard.press('d');
  }

  Serial.print("Input state: ");
  Serial.println(stateName(currentState));
}

void updateState(InputState desired, uint32_t nowMs) {
  if (desired == currentState) {
    candidateState = currentState;
    return;
  }

  if (desired != candidateState) {
    candidateState = desired;
    candidateSinceMs = nowMs;
    return;
  }

  if (nowMs - candidateSinceMs >= STATE_CONFIRM_MS) {
    applyState(candidateState);
  }
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'c' || command == 'C') beginCalibration();
  }
}

void printStatus(uint32_t nowMs, float envelope, float z, bool isLeadOff,
                 bool clipped) {
  if (nowMs - lastStatusMs < STATUS_PRINT_MS) return;
  lastStatusMs = nowMs;

  const char *quality = "READY";
  if (isLeadOff) quality = "LEADS_OFF";
  else if (clipped) quality = "CLIPPED";
  else if (!calibrated) quality = "CALIBRATING";
  else if (noiseSigma() > MAX_NOISE_SIGMA) quality = "NOISY";

  Serial.print("Q=");
  Serial.print(quality);
  Serial.print(" state=");
  Serial.print(stateName(currentState));
  Serial.print(" env=");
  Serial.print(envelope, 1);
  Serial.print(" z=");
  Serial.print(z, 1);
  Serial.print(" sigma=");
  Serial.print(noiseSigma(), 1);
  Serial.print(" sample_us=");
  Serial.println(lastSampleIntervalUs);
}

void setup() {
  Serial.begin(115200);
  pinMode(LO_PLUS_PIN, INPUT);
  pinMode(LO_MINUS_PIN, INPUT);
  pinMode(EMG_PIN, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(EMG_PIN, ADC_11db);

  bleKeyboard.begin();
  lastSampleUs = micros();
  beginCalibration();
}

void loop() {
  pollSerialCommands();

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - lastSampleUs;
  if (elapsedUs < SAMPLE_PERIOD_US) {
    delayMicroseconds(100);
    return;
  }
  lastSampleUs = nowUs;
  lastSampleIntervalUs = elapsedUs;
  ++samplesSinceStart;

  const uint32_t nowMs = millis();
  const bool isLeadOff = leadOff();
  const bool connected = bleKeyboard.isConnected();

  if (connected && !wasConnected) bleKeyboard.releaseAll();
  wasConnected = connected;

  if (isLeadOff) {
    if (!wasLeadOff) {
      releaseKeys();
      resetSignalWindow();
      calibrationCount = 0;
      calibrationDecimator = 0;
      calibrated = false;
      Serial.println("Lead-off detected; output released. Recalibrating after electrodes reconnect.");
    }
    wasLeadOff = true;
    printStatus(nowMs, envelopeEma, 0.0f, true, false);
    return;
  }

  if (wasLeadOff) {
    resetSignalWindow();
    calibrationCount = 0;
    calibrationDecimator = 0;
    calibrated = false;
    wasLeadOff = false;
    Serial.println("Electrodes reconnected; keep relaxed for calibration.");
  }

  const int raw = analogRead(EMG_PIN);
  const bool atRail = raw <= 8 || raw >= 4087;
  clipCount -= clipFlags[clipIndex];
  clipFlags[clipIndex] = atRail ? 1 : 0;
  clipCount += clipFlags[clipIndex];
  clipIndex = (clipIndex + 1) % CLIP_WINDOW;

  if (atRail) {
    releaseKeys();
    printStatus(nowMs, envelopeEma, 0.0f, false, signalClipped());
    return;
  }

  const float dtSeconds = static_cast<float>(elapsedUs) / 1000000.0f;
  const float dcAlpha = dtSeconds / (DC_BASELINE_TAU_SECONDS + dtSeconds);
  dcBaseline += dcAlpha * (static_cast<float>(raw) - dcBaseline);
  const float centered = static_cast<float>(raw) - dcBaseline;
  const float square = centered * centered;

  rmsSquareSum -= rmsSquares[rmsIndex];
  rmsSquares[rmsIndex] = square;
  rmsSquareSum += square;
  rmsIndex = (rmsIndex + 1) % RMS_WINDOW;
  if (rmsCount < RMS_WINDOW) ++rmsCount;

  if (rmsCount < RMS_WINDOW) {
    printStatus(nowMs, envelopeEma, 0.0f, false, signalClipped());
    return;
  }

  const float rmsEnvelope = sqrtf(rmsSquareSum / RMS_WINDOW);
  const float envelopeAlpha = dtSeconds / (ENVELOPE_TAU_SECONDS + dtSeconds);
  envelopeEma += envelopeAlpha * (rmsEnvelope - envelopeEma);

  if (!calibrated) {
    ++calibrationDecimator;
    if (calibrationDecimator >= CALIBRATION_DECIMATION) {
      calibrationDecimator = 0;
      calibrationValues[calibrationCount++] = envelopeEma;
      if (calibrationCount >= CALIBRATION_POINTS) finishCalibration();
    }
    printStatus(nowMs, envelopeEma, 0.0f, false, signalClipped());
    return;
  }

  const float z = zScore(envelopeEma);
  if (!signalClipped() && currentState == InputState::Idle &&
      z < ACTIVATE_Z && nowMs - lastNoiseUpdateMs >= NOISE_UPDATE_MS) {
    addIdleObservation(envelopeEma);
    lastNoiseUpdateMs = nowMs;
  }

  if (!connected) {
    currentState = InputState::Idle;
    candidateState = InputState::Idle;
  } else if (signalClipped() || noiseSigma() > MAX_NOISE_SIGMA) {
    releaseKeys();
  } else {
    updateState(desiredStateFor(z), nowMs);
  }

  printStatus(nowMs, envelopeEma, z, false, signalClipped());
}
