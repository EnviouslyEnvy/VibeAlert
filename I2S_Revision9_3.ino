#include "driver/i2s.h"
// #include "esp_sleep.h"
// #include "driver/rtc_io.h"
// #include "driver/gpio.h"
// #include "esp_wifi.h"
// #include "esp_pm.h"
#include "esp_dsp.h"
// #include "esp_log.h"

// Hardware Configuration
#define I2S_WS_PIN    6    // Word Select (WS) - D3/GPIO6
#define I2S_SD_PIN    7    // Serial Data (SD) - D4/GPIO7  
#define I2S_SCK_PIN   8    // Serial Clock (SCLK) - D5/GPIO8
// #define BUTTON_POWER_PIN 47  // Constant voltage output pin - D12/GPIO47 (RTC GPIO)
// #define SLEEP_DETECT_PIN 38  // Sleep detection pin - D11/GPIO38 (RTC GPIO)
#define OUTPUT_PIN    2    // PWM output pin - D2
#define LED_PIN       10   // LED indicator - D7/GPIO10

// I2S Configuration
#define I2S_PORT      I2S_NUM_0
#define I2S_SAMPLE_RATE 16000
#define I2S_SAMPLE_BITS 32
#define I2S_READ_LEN  1024  // Must be power of 2 for FFT
#define I2S_DMA_BUF_COUNT 4
#define I2S_DMA_BUF_LEN 64

// FFT Configuration
#define FFT_SIZE 512
#define FFT_SAMPLE_COUNT (FFT_SIZE)
#define FREQ_BIN_WIDTH (I2S_SAMPLE_RATE / FFT_SIZE)  // ~31.25 Hz per bin

// Frequency Detection Parameters
#define FREQ_SEARCH_MIN 50
#define FREQ_SEARCH_MAX 7500
#define PERSISTENCE_THRESHOLD 15
#define MAGNITUDE_THRESHOLD 0.25
#define NOISE_FLOOR_RATIO 8.0
#define PURITY_THRESHOLD 0.7
#define MIN_STABILITY_TIME 3000

// Intermittent Alarm Detection Parameters (strict)
#define MAX_ALARM_FREQUENCIES 3
#define INTERMITTENT_MAGNITUDE_THRESHOLD 0.6
#define INTERMITTENT_PURITY_THRESHOLD 0.92
#define ALARM_MEMORY_TIMEOUT 8000
#define MIN_ALARM_CYCLES 5
#define MAX_SILENCE_DURATION 5000
#define MIN_ALARM_ON_TIME 300
#define MIN_ALARM_OFF_TIME 200
#define PATTERN_CONFIDENCE_THRESHOLD 0.88
#define MAX_ALARM_MODE_DURATION 10000
#define CONFIDENCE_DECAY_RATE 0.08
#define RAPID_CONFIDENCE_DECAY 0.20

// Voice Rejection Parameters
#define ALARM_FREQ_MIN 400
#define ALARM_FREQ_MAX 4000
#define MAX_HARMONIC_CONTENT 0.3
#define MIN_SIGNAL_STABILITY 0.85
#define FREQUENCY_DEVIATION_PENALTY 0.4
#define SPECTRAL_COMPLEXITY_THRESHOLD 0.25

// Amplitude Processing Parameters
#define OUTPUT_MIN_THRESHOLD 0.002
#define OUTPUT_MAX_AMPLITUDE 0.15
#define ESP32_MAX_OUTPUT 3.3
#define SENSITIVITY_MULTIPLIER 6.0

// Pulsing Pattern Parameters
#define PULSE_ON_TIME 1000
#define PULSE_OFF_TIME 1000
#define PULSE_AMPLITUDE 255

// Light Sleep Audio Detection Parameters
#define WAKE_THRESHOLD 0.005
#define ACTIVE_THRESHOLD 0.015
#define SLEEP_THRESHOLD 0.001
#define SLEEP_DELAY_MS 5000
#define LIGHT_SLEEP_SAMPLE_MS 200

// DVFS Configuration
#define HIGH_POWER_CPU_FREQ 240
#define LOW_POWER_CPU_FREQ 80

// System State
enum SystemState {
  STATE_ACTIVE,
  STATE_LIGHT_SLEEP,
  STATE_DEEP_SLEEP
};

enum OutputMode {
  OUTPUT_AMPLITUDE,
  OUTPUT_PULSING
};

SystemState currentState = STATE_ACTIVE;
OutputMode outputMode = OUTPUT_AMPLITUDE;

// FFT Buffers
float* fft_input_buffer;
float* fft_output_buffer;
float* window_buffer;
float* magnitude_buffer;

// Frequency Tracking
struct FrequencyTracker {
  int targetFreqBin;
  int persistenceCount;
  float lastMagnitude;
  bool isDetected;
  unsigned long lastDetectionTime;
  unsigned long firstDetectionTime;
  int frequencyVariance;
  float lastPurity;
};

FrequencyTracker tracker = {0, 0, 0.0, false, 0, 0, 0, 0.0};

// Intermittent Alarm Detection Structures
struct AlarmFrequencyMemory {
  int freqBin;
  float avgMagnitude;
  float avgPurity;
  unsigned long lastSeenTime;
  unsigned long firstSeenTime;
  int totalDetections;
  int cycleCount;
  unsigned long lastOnTime;
  unsigned long lastOffTime;
  unsigned long totalOnDuration;
  unsigned long totalOffDuration;
  bool isActive;
  bool wasActive;
  float confidence;
  float harmonicContent;
  float frequencyStability;
  float spectralComplexity;
  int frequencyDeviations;
};

struct IntermittentAlarmTracker {
  AlarmFrequencyMemory alarmFreqs[MAX_ALARM_FREQUENCIES];
  int numAlarmFreqs;
  int bestAlarmIndex;
  bool alarmModeActive;
  unsigned long alarmModeStartTime;
  float bestConfidence;
};

IntermittentAlarmTracker alarmTracker = {{{0}}, 0, -1, false, 0, 0.0};

// Frequency Peak (analysis)
struct FreqPeak {
  int bin;
  float magnitude;
  float frequency;
};

// Pulsing Pattern Variables
unsigned long pulseStartTime = 0;
bool pulseState = false;
bool pulsingActive = false;

// Amplitude Variables
double currentAmplitude = 0.0;
double enhancedAmplitude = 0.0;

// Light Sleep Variables
unsigned long lastAudioTime = 0;
bool serialOutputEnabled = true;

// DVFS Variables
int currentCpuFreq = HIGH_POWER_CPU_FREQ;

void setCpuFrequencyAndTrack(int freqMhz) {
  setCpuFrequencyMhz(freqMhz);
  currentCpuFreq = freqMhz;
  if (Serial && serialOutputEnabled) {
    Serial.printf("CPU %dMHz\n", freqMhz);
  }
}

// void setupGPIO() {
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);

//   pinMode(OUTPUT_PIN, OUTPUT);
//   analogWrite(OUTPUT_PIN, 0);
// }

bool initializeDSP() {
  esp_err_t ret = dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  if (ret != ESP_OK) {
    if (Serial) Serial.printf("dsps_fft2r_init_fc32 failed: %d\n", ret);
    return false;
  }

  fft_input_buffer = (float*)malloc(FFT_SIZE * 2 * sizeof(float));
  fft_output_buffer = (float*)malloc(FFT_SIZE * 2 * sizeof(float));
  window_buffer = (float*)malloc(FFT_SIZE * sizeof(float));
  magnitude_buffer = (float*)malloc(FFT_SIZE / 2 * sizeof(float));

  if (!fft_input_buffer || !fft_output_buffer || !window_buffer || !magnitude_buffer) {
    if (Serial) Serial.println("Failed to allocate FFT buffers");
    return false;
  }

  dsps_wind_hann_f32(window_buffer, FFT_SIZE);
  return true;
}

bool initializeI2S() {
  i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = I2S_DMA_BUF_COUNT,
    .dma_buf_len = I2S_DMA_BUF_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = -1,
    .data_in_num = I2S_SD_PIN
  };

  esp_err_t result = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (result != ESP_OK) {
    if (Serial) Serial.printf("i2s_driver_install failed: %d\n", result);
    return false;
  }

  result = i2s_set_pin(I2S_PORT, &pin_config);
  if (result != ESP_OK) {
    if (Serial) Serial.printf("i2s_set_pin failed: %d\n", result);
    return false;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  return true;
}

double readI2SAudioAmplitude() {
  uint8_t i2s_data[I2S_READ_LEN];
  size_t bytes_read = 0;

  esp_err_t result = i2s_read(I2S_PORT, i2s_data, I2S_READ_LEN, &bytes_read, pdMS_TO_TICKS(50));
  if (result != ESP_OK || bytes_read == 0) {
    return 0.0;
  }

  int32_t* samples = (int32_t*)i2s_data;
  int sample_count = bytes_read / 4;

  int32_t signalMax = INT32_MIN;
  int32_t signalMin = INT32_MAX;

  for (int i = 0; i < sample_count; i++) {
    int32_t sample = samples[i] >> 8;
    if (sample > signalMax) signalMax = sample;
    if (sample < signalMin) signalMin = sample;
  }

  int32_t peakToPeak = signalMax - signalMin;
  double amplitude = (double)peakToPeak / 16777216.0;
  return amplitude;
}

// Voice Rejection Helpers
float calculateHarmonicContent(int peakBin, float peakMagnitude) {
  if (peakBin < 0 || peakMagnitude <= 0.0f) return 1.0f;

  float fundamentalFreq = peakBin * FREQ_BIN_WIDTH;
  float harmonicSum = 0.0f;
  int harmonicCount = 0;

  for (int harmonic = 2; harmonic <= 4; harmonic++) {
    int harmonicBin = (int)(fundamentalFreq * harmonic / FREQ_BIN_WIDTH);
    if (harmonicBin < FFT_SIZE / 2) {
      harmonicSum += magnitude_buffer[harmonicBin];
      harmonicCount++;
    }
  }

  if (harmonicCount == 0) return 0.0f;
  float averageHarmonic = harmonicSum / harmonicCount;
  return averageHarmonic / peakMagnitude;
}

float calculateFrequencyStability(AlarmFrequencyMemory* alarm, int currentBin) {
  if (alarm->totalDetections < 2) return 1.0f;

  int deviation = abs(currentBin - alarm->freqBin);
  if (deviation > 2) {
    alarm->frequencyDeviations++;
    return 0.5f;
  }

  float stability = 1.0f - (float)alarm->frequencyDeviations / (float)alarm->totalDetections;
  return constrain(stability, 0.0f, 1.0f);
}

float calculateSpectralComplexity(int peakBin) {
  if (peakBin < 0) return 1.0f;

  float totalEnergy = 0.0f;
  float peakRegionEnergy = 0.0f;
  int rangeStart = max(1, peakBin - 10);
  int rangeEnd = min(FFT_SIZE / 2 - 1, peakBin + 10);

  for (int i = rangeStart; i <= rangeEnd; i++) {
    totalEnergy += magnitude_buffer[i];
    if (abs(i - peakBin) <= 2) {
      peakRegionEnergy += magnitude_buffer[i];
    }
  }

  if (totalEnergy == 0.0f) return 1.0f;
  float peakConcentration = peakRegionEnergy / totalEnergy;
  return 1.0f - peakConcentration;
}

bool isValidAlarmFrequency(float frequency) {
  return (frequency >= ALARM_FREQ_MIN && frequency <= ALARM_FREQ_MAX);
}

void checkSerialCommands() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'p' || command == 'P') {
      serialOutputEnabled = false;
      if (Serial) Serial.println("serial: paused");
    } else if (command == 's' || command == 'S') {
      serialOutputEnabled = true;
      if (Serial) Serial.println("serial: started");
    } else if (command == 'w' || command == 'W') {
      if (currentState == STATE_LIGHT_SLEEP) {
        setCpuFrequencyAndTrack(HIGH_POWER_CPU_FREQ);
        currentState = STATE_ACTIVE;
        lastAudioTime = millis();
        digitalWrite(LED_PIN, LOW);
        if (Serial) Serial.println("wake: light-sleep -> active");
      }
    } else if (command == 'd' || command == 'D') {
      if (Serial) {
        Serial.printf("LED pin (GPIO%d): %s\n", LED_PIN, digitalRead(LED_PIN) ? "HIGH" : "LOW");
      }
    } else if (command == 'r' || command == 'R') {
      resetAlarmTracker();
      if (Serial) Serial.println("alarm tracker: reset");
    }
  }
}

void loop() {
  checkSerialCommands();

  switch (currentState) {
    case STATE_ACTIVE:
      processActiveMode();
      break;
    case STATE_LIGHT_SLEEP:
      processLightSleep();
      break;
    case STATE_DEEP_SLEEP:
      if (Serial) Serial.println("unexpected: deep sleep state");
      break;
  }
}

void processActiveMode() {
  processAudioSample();

  if (enhancedAmplitude > ACTIVE_THRESHOLD) {
    lastAudioTime = millis();
  }

  if (millis() - lastAudioTime > SLEEP_DELAY_MS) {
    if (serialOutputEnabled && Serial) {
      Serial.println("light sleep");
      Serial.flush();
    }

    setCpuFrequencyAndTrack(LOW_POWER_CPU_FREQ);
    currentState = STATE_LIGHT_SLEEP;
    analogWrite(OUTPUT_PIN, 0);
    digitalWrite(LED_PIN, HIGH);
  } else {
    delay(50);
  }
}

void processLightSleep() {
  delay(LIGHT_SLEEP_SAMPLE_MS);

  double rawAmplitude = readI2SAudioAmplitude();
  double enhancedAmplitude = rawAmplitude * SENSITIVITY_MULTIPLIER;

  if (enhancedAmplitude > WAKE_THRESHOLD) {
    if (serialOutputEnabled && Serial) {
      Serial.printf("wake: enhanced=%.4f\n", enhancedAmplitude);
    }

    setCpuFrequencyAndTrack(HIGH_POWER_CPU_FREQ);
    currentState = STATE_ACTIVE;
    lastAudioTime = millis();
    digitalWrite(LED_PIN, LOW);
  } else {
    if (serialOutputEnabled && Serial) {
      Serial.print("LIGHT_SLEEP,");
      Serial.print(rawAmplitude, 4);
      Serial.print(",");
      Serial.print(enhancedAmplitude, 4);
      Serial.print(",");
      Serial.print("0.00");
      Serial.print(",");
      Serial.println("0");
    }
  }
}

void processAudioSample() {
  uint8_t i2s_data[I2S_READ_LEN];
  size_t bytes_read = 0;

  esp_err_t result = i2s_read(I2S_PORT, i2s_data, I2S_READ_LEN, &bytes_read, pdMS_TO_TICKS(100));
  if (result != ESP_OK || bytes_read == 0) {
    if (Serial) Serial.println("i2s read failed");
    return;
  }

  int32_t* samples = (int32_t*)i2s_data;
  int sample_count = min((int)(bytes_read / 4), FFT_SIZE);

  int32_t signalMax = INT32_MIN;
  int32_t signalMin = INT32_MAX;
  for (int i = 0; i < sample_count; i++) {
    int32_t sample = samples[i] >> 8;
    if (sample > signalMax) signalMax = sample;
    if (sample < signalMin) signalMin = sample;
  }

  int32_t peakToPeak = signalMax - signalMin;
  currentAmplitude = (double)peakToPeak / 16777216.0;
  enhancedAmplitude = currentAmplitude * SENSITIVITY_MULTIPLIER;

  for (int i = 0; i < sample_count; i++) {
    float sample = (float)(samples[i] >> 8) / 8388608.0f;
    fft_input_buffer[i * 2] = sample * window_buffer[i];
    fft_input_buffer[i * 2 + 1] = 0.0f;
  }

  for (int i = sample_count; i < FFT_SIZE; i++) {
    fft_input_buffer[i * 2] = 0.0f;
    fft_input_buffer[i * 2 + 1] = 0.0f;
  }

  dsps_fft2r_fc32(fft_input_buffer, FFT_SIZE);

  for (int i = 0; i < FFT_SIZE / 2; i++) {
    float real = fft_input_buffer[i * 2];
    float imag = fft_input_buffer[i * 2 + 1];
    magnitude_buffer[i] = sqrt(real * real + imag * imag);
  }

  analyzeFrequencies();
  updateOutput();
} 

void setup() {
  Serial.begin(115200);

  // Keep radios off by not initializing
  if (Serial) {
    Serial.println("I2S_Revision9_3");
    Serial.printf("sample_rate=%d fft=%d bin=%.2fHz\n", I2S_SAMPLE_RATE, FFT_SIZE, FREQ_BIN_WIDTH);
  }

  setCpuFrequencyMhz(HIGH_POWER_CPU_FREQ);
  currentCpuFreq = HIGH_POWER_CPU_FREQ;

  setupGPIO();

  if (!initializeDSP()) {
    if (Serial) Serial.println("DSP init failed");
    while(1) delay(1000);
  }

  if (!initializeI2S()) {
    if (Serial) Serial.println("I2S init failed");
    while(1) delay(1000);
  }

  tracker.targetFreqBin = 0;
  lastAudioTime = millis();

  if (Serial) {
    Serial.printf("analyze %d-%dHz, bins %d-%d of %d\n",
                  FREQ_SEARCH_MIN, FREQ_SEARCH_MAX,
                  (int)(FREQ_SEARCH_MIN / FREQ_BIN_WIDTH),
                  (int)(FREQ_SEARCH_MAX / FREQ_BIN_WIDTH),
                  FFT_SIZE / 2);
    Serial.println("commands: p(pause), s(start), w(wake), d(diag), r(reset)");
  }
}

void updateIntermittentAlarmDetection(FreqPeak* peaks, int peakCount) {
  unsigned long currentTime = millis();

  for (int i = 0; i < alarmTracker.numAlarmFreqs; i++) {
    AlarmFrequencyMemory* alarm = &alarmTracker.alarmFreqs[i];
    alarm->wasActive = alarm->isActive;
    alarm->isActive = false;

    for (int j = 0; j < peakCount; j++) {
      if (abs(peaks[j].bin - alarm->freqBin) <= 1) {
        float purity = calculatePurity(peaks[j].bin, peaks[j].magnitude);

        float harmonicContent = calculateHarmonicContent(peaks[j].bin, peaks[j].magnitude);
        float frequencyStability = calculateFrequencyStability(alarm, peaks[j].bin);
        float spectralComplexity = calculateSpectralComplexity(peaks[j].bin);
        bool validFreqRange = isValidAlarmFrequency(peaks[j].frequency);

        if (peaks[j].magnitude > INTERMITTENT_MAGNITUDE_THRESHOLD && 
            purity > INTERMITTENT_PURITY_THRESHOLD &&
            harmonicContent < MAX_HARMONIC_CONTENT &&
            frequencyStability > MIN_SIGNAL_STABILITY &&
            spectralComplexity < SPECTRAL_COMPLEXITY_THRESHOLD &&
            validFreqRange) {

          alarm->isActive = true;
          alarm->lastSeenTime = currentTime;
          alarm->totalDetections++;

          alarm->avgMagnitude = (alarm->avgMagnitude + peaks[j].magnitude) / 2.0f;
          alarm->avgPurity = (alarm->avgPurity + purity) / 2.0f;
          alarm->harmonicContent = (alarm->harmonicContent + harmonicContent) / 2.0f;
          alarm->frequencyStability = frequencyStability;
          alarm->spectralComplexity = (alarm->spectralComplexity + spectralComplexity) / 2.0f;

          if (!alarm->wasActive && alarm->isActive) {
            alarm->lastOnTime = currentTime;
            if (alarm->lastOffTime > 0) {
              unsigned long offDuration = currentTime - alarm->lastOffTime;
              if (offDuration >= MIN_ALARM_OFF_TIME && offDuration <= MAX_SILENCE_DURATION) {
                alarm->totalOffDuration += offDuration;
                alarm->cycleCount++;
              }
            }
          }
          break;
        } else {
          if (Serial && serialOutputEnabled) {
            static unsigned long lastRejectMsg = 0;
            if (currentTime - lastRejectMsg > 2000) {
              lastRejectMsg = currentTime;
              Serial.printf("reject: %.0fHz mag=%.2f pur=%.2f harm=%.2f stab=%.2f comp=%.2f %s\n",
                           peaks[j].frequency, peaks[j].magnitude, purity, harmonicContent, 
                           frequencyStability, spectralComplexity, validFreqRange ? "OK" : "RANGE");
            }
          }
          alarm->confidence = max(0.0f, alarm->confidence - 0.1f);
        }
      }
    }

    if (alarm->wasActive && !alarm->isActive) {
      alarm->lastOffTime = currentTime;
      if (alarm->lastOnTime > 0) {
        unsigned long onDuration = currentTime - alarm->lastOnTime;
        if (onDuration >= MIN_ALARM_ON_TIME) {
          alarm->totalOnDuration += onDuration;
        }
      }
    }

    if (currentTime - alarm->lastSeenTime > ALARM_MEMORY_TIMEOUT) {
      removeAlarmFrequency(i);
      i--;
    } else {
      updateAlarmConfidenceWithVoiceRejection(alarm);
    }
  }

  for (int i = 0; i < peakCount; i++) {
    float purity = calculatePurity(peaks[i].bin, peaks[i].magnitude);
    float harmonicContent = calculateHarmonicContent(peaks[i].bin, peaks[i].magnitude);
    float spectralComplexity = calculateSpectralComplexity(peaks[i].bin);
    bool validFreqRange = isValidAlarmFrequency(peaks[i].frequency);

    if (peaks[i].magnitude > INTERMITTENT_MAGNITUDE_THRESHOLD && 
        purity > INTERMITTENT_PURITY_THRESHOLD &&
        harmonicContent < MAX_HARMONIC_CONTENT &&
        spectralComplexity < SPECTRAL_COMPLEXITY_THRESHOLD &&
        validFreqRange) {

      bool isNew = true;
      for (int j = 0; j < alarmTracker.numAlarmFreqs; j++) {
        if (abs(peaks[i].bin - alarmTracker.alarmFreqs[j].freqBin) <= 1) {
          isNew = false;
          break;
        }
      }

      if (isNew && alarmTracker.numAlarmFreqs < MAX_ALARM_FREQUENCIES) {
        addNewAlarmFrequencyWithVoiceRejection(peaks[i].bin, peaks[i].magnitude, purity, 
                                               harmonicContent, spectralComplexity);
      }
    }
  }

  updateBestAlarmCandidate();

  if (!alarmTracker.alarmModeActive && alarmTracker.bestConfidence > PATTERN_CONFIDENCE_THRESHOLD) {
    activateIntermittentAlarmMode();
  } else if (alarmTracker.alarmModeActive) {
    bool shouldExit = false;

    if (alarmTracker.bestConfidence < PATTERN_CONFIDENCE_THRESHOLD * 0.3) {
      shouldExit = true;
    }

    if (currentTime - alarmTracker.alarmModeStartTime > MAX_ALARM_MODE_DURATION) {
      shouldExit = true;
    }

    if (alarmTracker.bestAlarmIndex >= 0) {
      AlarmFrequencyMemory* bestAlarm = &alarmTracker.alarmFreqs[alarmTracker.bestAlarmIndex];
      if (currentTime - bestAlarm->lastSeenTime > MAX_SILENCE_DURATION) {
        shouldExit = true;
      }
    }

    if (shouldExit) {
      if (Serial && serialOutputEnabled) {
        if (alarmTracker.bestConfidence < PATTERN_CONFIDENCE_THRESHOLD * 0.3) {
          Serial.println("alarm exit: low confidence");
        } else if (currentTime - alarmTracker.alarmModeStartTime > MAX_ALARM_MODE_DURATION) {
          Serial.println("alarm exit: max duration");
        } else {
          Serial.println("alarm exit: inactivity");
        }
      }
      deactivateIntermittentAlarmMode();
    }
  }
}

void addNewAlarmFrequencyWithVoiceRejection(int freqBin, float magnitude, float purity, 
                                            float harmonicContent, float spectralComplexity) {
  if (alarmTracker.numAlarmFreqs >= MAX_ALARM_FREQUENCIES) return;

  AlarmFrequencyMemory* alarm = &alarmTracker.alarmFreqs[alarmTracker.numAlarmFreqs];
  alarm->freqBin = freqBin;
  alarm->avgMagnitude = magnitude;
  alarm->avgPurity = purity;
  alarm->harmonicContent = harmonicContent;
  alarm->spectralComplexity = spectralComplexity;
  alarm->frequencyStability = 1.0f;
  alarm->frequencyDeviations = 0;
  alarm->firstSeenTime = millis();
  alarm->lastSeenTime = millis();
  alarm->totalDetections = 1;
  alarm->cycleCount = 0;
  alarm->lastOnTime = millis();
  alarm->lastOffTime = 0;
  alarm->totalOnDuration = 0;
  alarm->totalOffDuration = 0;
  alarm->isActive = true;
  alarm->wasActive = false;
  alarm->confidence = 0.05f;

  alarmTracker.numAlarmFreqs++;

  if (Serial && serialOutputEnabled) {
    Serial.printf("new alarm candidate: %.1fHz mag=%.2f pur=%.2f harm=%.2f comp=%.2f\n", 
                  freqBin * FREQ_BIN_WIDTH, magnitude, purity, harmonicContent, spectralComplexity);
  }
}

void updateAlarmConfidenceWithVoiceRejection(AlarmFrequencyMemory* alarm) {
  unsigned long currentTime = millis();
  unsigned long timeSinceLastSeen = currentTime - alarm->lastSeenTime;

  float confidence = 0.0f;

  float detectionRatio = (float)alarm->totalDetections / max(1UL, (currentTime - alarm->firstSeenTime) / 500);
  confidence += constrain(detectionRatio, 0.0f, 0.2f);

  if (alarm->cycleCount >= MIN_ALARM_CYCLES) {
    confidence += 0.3f;

    if (alarm->totalOnDuration > 0 && alarm->totalOffDuration > 0) {
      float avgOnTime = (float)alarm->totalOnDuration / alarm->cycleCount;
      float avgOffTime = (float)alarm->totalOffDuration / alarm->cycleCount;

      if (avgOnTime > MIN_ALARM_ON_TIME && avgOffTime > MIN_ALARM_OFF_TIME && 
          avgOnTime < 2000 && avgOffTime < 3000) {
        confidence += 0.15f;
      }
    }
  }

  confidence += constrain(alarm->avgMagnitude - INTERMITTENT_MAGNITUDE_THRESHOLD, 0.0f, 0.05f);
  confidence += constrain(alarm->avgPurity - INTERMITTENT_PURITY_THRESHOLD, 0.0f, 0.05f);
  confidence += constrain(MAX_HARMONIC_CONTENT - alarm->harmonicContent, 0.0f, 0.1f);
  confidence += constrain(alarm->frequencyStability - MIN_SIGNAL_STABILITY, 0.0f, 0.1f);
  confidence += constrain(SPECTRAL_COMPLEXITY_THRESHOLD - alarm->spectralComplexity, 0.0f, 0.1f);

  float decayRate = CONFIDENCE_DECAY_RATE;
  if (timeSinceLastSeen > MAX_SILENCE_DURATION) {
    decayRate = RAPID_CONFIDENCE_DECAY;
  }
  float timePenalty = (timeSinceLastSeen / 300.0f) * decayRate;
  confidence -= timePenalty;

  if (timeSinceLastSeen > MAX_SILENCE_DURATION * 1.2) {
    confidence -= 0.4f;
  }

  if (alarm->harmonicContent > MAX_HARMONIC_CONTENT) {
    confidence -= 0.2f;
  }
  if (alarm->frequencyStability < MIN_SIGNAL_STABILITY) {
    confidence -= 0.15f;
  }
  if (alarm->spectralComplexity > SPECTRAL_COMPLEXITY_THRESHOLD) {
    confidence -= 0.1f;
  }

  alarm->confidence = constrain(confidence, 0.0f, 1.0f);
} 

void removeAlarmFrequency(int index) {
  if (index < 0 || index >= alarmTracker.numAlarmFreqs) return;

  for (int i = index; i < alarmTracker.numAlarmFreqs - 1; i++) {
    alarmTracker.alarmFreqs[i] = alarmTracker.alarmFreqs[i + 1];
  }
  alarmTracker.numAlarmFreqs--;

  if (alarmTracker.bestAlarmIndex == index) {
    alarmTracker.bestAlarmIndex = -1;
  } else if (alarmTracker.bestAlarmIndex > index) {
    alarmTracker.bestAlarmIndex--;
  }
}

void updateBestAlarmCandidate() {
  alarmTracker.bestAlarmIndex = -1;
  alarmTracker.bestConfidence = 0.0f;

  for (int i = 0; i < alarmTracker.numAlarmFreqs; i++) {
    if (alarmTracker.alarmFreqs[i].confidence > alarmTracker.bestConfidence) {
      alarmTracker.bestConfidence = alarmTracker.alarmFreqs[i].confidence;
      alarmTracker.bestAlarmIndex = i;
    }
  }
}

void activateIntermittentAlarmMode() {
  alarmTracker.alarmModeActive = true;
  alarmTracker.alarmModeStartTime = millis();

  outputMode = OUTPUT_PULSING;
  pulsingActive = true;
  pulseStartTime = millis();
  pulseState = true;

  if (Serial && serialOutputEnabled && alarmTracker.bestAlarmIndex >= 0) {
    AlarmFrequencyMemory* bestAlarm = &alarmTracker.alarmFreqs[alarmTracker.bestAlarmIndex];
    Serial.printf("alarm: %.1f Hz conf=%.2f cycles=%d\n",
                  bestAlarm->freqBin * FREQ_BIN_WIDTH, bestAlarm->confidence, bestAlarm->cycleCount);
  }
}

void deactivateIntermittentAlarmMode() {
  unsigned long alarmDuration = millis() - alarmTracker.alarmModeStartTime;
  alarmTracker.alarmModeActive = false;

  if (outputMode == OUTPUT_PULSING && !tracker.isDetected) {
    outputMode = OUTPUT_AMPLITUDE;
    pulsingActive = false;
    pulseState = false;
  }

  if (Serial && serialOutputEnabled) {
    Serial.printf("alarm end: %lums, best_conf=%.2f\n", 
                  alarmDuration, 
                  alarmTracker.bestConfidence);
  }
}

void analyzeFrequencies() {
  int freq_start_bin = FREQ_SEARCH_MIN / FREQ_BIN_WIDTH;
  int freq_end_bin = FREQ_SEARCH_MAX / FREQ_BIN_WIDTH;

  freq_start_bin = constrain(freq_start_bin, 1, FFT_SIZE / 2 - 1);
  freq_end_bin = constrain(freq_end_bin, freq_start_bin + 1, FFT_SIZE / 2 - 1);

  FreqPeak peaks[10];
  int peakCount = 0;

  for (int i = freq_start_bin + 1; i < freq_end_bin - 1; i++) {
    if (magnitude_buffer[i] > magnitude_buffer[i-1] && 
        magnitude_buffer[i] > magnitude_buffer[i+1] &&
        magnitude_buffer[i] > 0.02) {

      if (peakCount < 10) {
        peaks[peakCount].bin = i;
        peaks[peakCount].magnitude = magnitude_buffer[i];
        peaks[peakCount].frequency = i * FREQ_BIN_WIDTH;
        peakCount++;
      }
    }
  }

  for (int i = 0; i < peakCount - 1; i++) {
    for (int j = i + 1; j < peakCount; j++) {
      if (peaks[j].magnitude > peaks[i].magnitude) {
        FreqPeak temp = peaks[i];
        peaks[i] = peaks[j];
        peaks[j] = temp;
      }
    }
  }

  float peakMagnitude = 0.0f;
  int peakBin = -1;
  if (peakCount > 0) {
    peakMagnitude = peaks[0].magnitude;
    peakBin = peaks[0].bin;
  }

  float noiseFloor = calculateNoiseFloor(peakBin);
  float purity = calculatePurity(peakBin, peakMagnitude);

  bool significantPeak = (peakMagnitude > MAGNITUDE_THRESHOLD) && 
                        (peakMagnitude > noiseFloor * NOISE_FLOOR_RATIO) &&
                        (purity > PURITY_THRESHOLD);

  if (significantPeak) {
    if (abs(peakBin - tracker.targetFreqBin) <= 1) {
      tracker.persistenceCount++;
      tracker.lastMagnitude = peakMagnitude;
      tracker.lastDetectionTime = millis();
      tracker.lastPurity = purity;

      if (tracker.persistenceCount == 1) {
        tracker.firstDetectionTime = millis();
      }

      unsigned long stabilityTime = millis() - tracker.firstDetectionTime;
      bool hasStability = stabilityTime >= MIN_STABILITY_TIME;
      bool hasPersistence = tracker.persistenceCount >= PERSISTENCE_THRESHOLD;

      if (hasPersistence && hasStability && !tracker.isDetected) {
        tracker.isDetected = true;
        onFrequencyDetected(peakBin, peakMagnitude);
      }
    } else {
      int binDifference = abs(peakBin - tracker.targetFreqBin);

      if (tracker.persistenceCount > 0) {
        tracker.frequencyVariance += binDifference;
        if (tracker.frequencyVariance > 5) {
          resetTracker();
          return;
        }
      }

      tracker.targetFreqBin = peakBin;
      tracker.persistenceCount = 1;
      tracker.lastMagnitude = peakMagnitude;
      tracker.lastPurity = purity;
      tracker.isDetected = false;
      tracker.firstDetectionTime = millis();
      tracker.frequencyVariance = 0;
    }
  } else {
    if (millis() - tracker.lastDetectionTime > 2000) {
      if (tracker.isDetected) {
        onFrequencyLost();
      }
      resetTracker();
    }
  }

  updateIntermittentAlarmDetection(peaks, peakCount);

  // Optional: reduce continuous spam; keep concise outputStatus
  outputStatus(peakBin, peakMagnitude, noiseFloor, significantPeak, purity);
}

float calculateNoiseFloor(int peakBin) {
  float sum = 0.0f;
  int count = 0;

  for (int i = max(1, peakBin - 10); i <= min(FFT_SIZE / 2 - 1, peakBin + 10); i++) {
    if (abs(i - peakBin) > 2) {
      sum += magnitude_buffer[i];
      count++;
    }
  }

  return count > 0 ? sum / count : 0.0f;
}

float calculatePurity(int peakBin, float peakMagnitude) {
  if (peakBin < 0 || peakMagnitude <= 0.0f) return 0.0f;

  float surroundingSum = 0.0f;
  int surroundingCount = 0;

  for (int i = max(1, peakBin - 5); i <= min(FFT_SIZE / 2 - 1, peakBin + 5); i++) {
    if (abs(i - peakBin) > 1) {
      surroundingSum += magnitude_buffer[i];
      surroundingCount++;
    }
  }

  if (surroundingCount == 0) return 0.0f;

  float averageSurrounding = surroundingSum / surroundingCount;
  float purity = (averageSurrounding > 0.0f) ? peakMagnitude / averageSurrounding : 0.0f;
  return constrain(purity / 15.0f, 0.0f, 1.0f);
}

void resetTracker() {
  tracker.persistenceCount = 0;
  tracker.isDetected = false;
  tracker.lastMagnitude = 0.0f;
  tracker.lastDetectionTime = 0;
  tracker.firstDetectionTime = 0;
  tracker.frequencyVariance = 0;
  tracker.lastPurity = 0.0f;

  if (!alarmTracker.alarmModeActive) {
    outputMode = OUTPUT_AMPLITUDE;
    pulsingActive = false;
    pulseState = false;
  }
}

void resetAlarmTracker() {
  alarmTracker.numAlarmFreqs = 0;
  alarmTracker.bestAlarmIndex = -1;
  alarmTracker.alarmModeActive = false;
  alarmTracker.alarmModeStartTime = 0;
  alarmTracker.bestConfidence = 0.0f;

  for (int i = 0; i < MAX_ALARM_FREQUENCIES; i++) {
    AlarmFrequencyMemory* alarm = &alarmTracker.alarmFreqs[i];
    alarm->freqBin = 0;
    alarm->avgMagnitude = 0.0f;
    alarm->avgPurity = 0.0f;
    alarm->harmonicContent = 0.0f;
    alarm->frequencyStability = 0.0f;
    alarm->spectralComplexity = 0.0f;
    alarm->frequencyDeviations = 0;
    alarm->lastSeenTime = 0;
    alarm->firstSeenTime = 0;
    alarm->totalDetections = 0;
    alarm->cycleCount = 0;
    alarm->lastOnTime = 0;
    alarm->lastOffTime = 0;
    alarm->totalOnDuration = 0;
    alarm->totalOffDuration = 0;
    alarm->isActive = false;
    alarm->wasActive = false;
    alarm->confidence = 0.0f;
  }

  if (outputMode == OUTPUT_PULSING && !tracker.isDetected) {
    outputMode = OUTPUT_AMPLITUDE;
    pulsingActive = false;
    pulseState = false;
  }

  if (Serial && serialOutputEnabled) {
    Serial.println("alarm tracker: cleared");
  }
}

void onFrequencyDetected(int freqBin, float magnitude) {
  float frequency = freqBin * FREQ_BIN_WIDTH;
  if (Serial) {
    Serial.printf("tone: %.1f Hz mag=%.4f persist=%d\n", frequency, magnitude, tracker.persistenceCount);
  }

  outputMode = OUTPUT_PULSING;
  pulsingActive = true;
  pulseStartTime = millis();
  pulseState = true;
}

void onFrequencyLost() {
  if (Serial) {
    Serial.println("tone lost");
  }

  outputMode = OUTPUT_AMPLITUDE;
  pulsingActive = false;
  pulseState = false;
}

void outputStatus(int peakBin, float peakMagnitude, float noiseFloor, bool significant, float purity) {
  static unsigned long lastOutput = 0;
  if (millis() - lastOutput < 500) return;
  lastOutput = millis();

  if (!Serial || !serialOutputEnabled) return;

  float peakFreq = peakBin * FREQ_BIN_WIDTH;

  unsigned long timeSinceLastAudio = millis() - lastAudioTime;
  unsigned long timeToSleep = 0;
  if (currentState == STATE_ACTIVE && timeSinceLastAudio < SLEEP_DELAY_MS) {
    timeToSleep = SLEEP_DELAY_MS - timeSinceLastAudio;
  }

  Serial.printf("%s(%dMHz) %s Amp: %.4f->%.4f",
                currentState == STATE_ACTIVE ? "ACTIVE" : 
                currentState == STATE_LIGHT_SLEEP ? "LIGHT_SLEEP" : "DEEP_SLEEP",
                currentCpuFreq,
                outputMode == OUTPUT_PULSING ? "PULSING" : "AMPLITUDE",
                currentAmplitude, enhancedAmplitude);

  if (currentState == STATE_ACTIVE && timeToSleep > 0) {
    Serial.printf(" sleep_in=%lums", timeToSleep);
  }

  if (currentState == STATE_ACTIVE) {
    Serial.printf(" Freq: %.1fHz(%.4f) Noise=%.4f Pur=%.2f Persist=%d/%d %s",
                  peakFreq, peakMagnitude, noiseFloor, purity,
                  tracker.persistenceCount, PERSISTENCE_THRESHOLD,
                  tracker.isDetected ? "DETECTED" : significant ? "TRACKING" : "SEARCHING");

    if (tracker.persistenceCount > 0 && !tracker.isDetected) {
      unsigned long stabilityTime = millis() - tracker.firstDetectionTime;
      Serial.printf(" Stable: %lums/%dms", stabilityTime, MIN_STABILITY_TIME);
    }

    if (outputMode == OUTPUT_PULSING && pulsingActive) {
      Serial.printf(" Pulse:%s", pulseState ? "ON" : "OFF");
    }

    if (alarmTracker.numAlarmFreqs > 0) {
      Serial.printf(" Alarms:%d", alarmTracker.numAlarmFreqs);
      if (alarmTracker.bestAlarmIndex >= 0) {
        AlarmFrequencyMemory* bestAlarm = &alarmTracker.alarmFreqs[alarmTracker.bestAlarmIndex];
        Serial.printf(" Best: %.1fHz(conf=%.2f,cyc=%d,harm=%.2f,stab=%.2f)", 
                      bestAlarm->freqBin * FREQ_BIN_WIDTH, 
                      bestAlarm->confidence, 
                      bestAlarm->cycleCount,
                      bestAlarm->harmonicContent,
                      bestAlarm->frequencyStability);
        if (alarmTracker.alarmModeActive) {
          Serial.printf(" MODE:ALARM");
        }
      }
    }
  }

  Serial.println();
}

void updateOutput() {
  if (outputMode == OUTPUT_PULSING && pulsingActive) {
    updatePulsingOutput();
  } else {
    updateAmplitudeOutput();
  }
}

void updatePulsingOutput() {
  unsigned long currentTime = millis();
  unsigned long timeInCycle = currentTime - pulseStartTime;

  if (pulseState) {
    if (timeInCycle >= PULSE_ON_TIME) {
      pulseState = false;
      pulseStartTime = currentTime;
      analogWrite(OUTPUT_PIN, 0);
    } else {
      analogWrite(OUTPUT_PIN, PULSE_AMPLITUDE);
    }
  } else {
    if (timeInCycle >= PULSE_OFF_TIME) {
      pulseState = true;
      pulseStartTime = currentTime;
      analogWrite(OUTPUT_PIN, PULSE_AMPLITUDE);
    } else {
      analogWrite(OUTPUT_PIN, 0);
    }
  }
}

void updateAmplitudeOutput() {
  double outputVoltage = 0.0;
  int pwmValue = 0;

  if (enhancedAmplitude >= OUTPUT_MIN_THRESHOLD) {
    double scaledAmplitude = min(enhancedAmplitude, OUTPUT_MAX_AMPLITUDE);
    pwmValue = map(constrain(scaledAmplitude * 1000, OUTPUT_MIN_THRESHOLD * 1000, OUTPUT_MAX_AMPLITUDE * 1000), 
                  OUTPUT_MIN_THRESHOLD * 1000, OUTPUT_MAX_AMPLITUDE * 1000, 0, 255);
    pwmValue = constrain(pwmValue, 0, 255);
    analogWrite(OUTPUT_PIN, pwmValue);
    outputVoltage = (pwmValue / 255.0) * ESP32_MAX_OUTPUT;
  } else {
    analogWrite(OUTPUT_PIN, 0);
    outputVoltage = 0.0;
  }
} 


