const int micPin = A0;  // Analog pin for microphone input
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

// Sensitivity tuning
const double WAKE_THRESHOLD = 0.1;      // How sensitive to wake up
const double SLEEP_THRESHOLD = 0.05;    // How quiet before sleeping
const int QUIET_TIME = 2000;            // How long quiet before sleep (ms)

// Timing tuning  
const int LOW_POWER_INTERVAL = 2000;    // Check every 2 seconds
const int HIGH_PRECISION_INTERVAL = 50; // Check every 50ms when active

// Low power mode settings
const int LOW_POWER_SAMPLE_WINDOW = 200;  // Shorter sampling window for power saving

// State machine variables
enum AudioState {
  LOW_POWER_MODE,
  HIGH_PRECISION_MODE
};

AudioState currentState = LOW_POWER_MODE;
int quietCount = 0;
const int QUIET_COUNT_LIMIT = QUIET_TIME / HIGH_PRECISION_INTERVAL; // How many quiet samples before switching

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  Serial.println("Starting adaptive audio monitoring...");
  Serial.println("Mode: LOW_POWER");
}

void loop() {
  switch(currentState) {
    case LOW_POWER_MODE:
      lowPowerSample();
      break;
      
    case HIGH_PRECISION_MODE:
      highPrecisionSample();
      break;
  }
}

void lowPowerSample() {
  unsigned long startMillis = millis();
  unsigned int peakToPeak = 0;
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // Collect data for shorter window to save power
  while (millis() - startMillis < LOW_POWER_SAMPLE_WINDOW) {
    sample = analogRead(micPin);
    if (sample < 1024) {  // Toss out spurious readings
      if (sample > signalMax) {
        signalMax = sample;  // Save just the max levels
      }
      else if (sample < signalMin) {
        signalMin = sample;  // Save just the min levels
      }
    }
  }
  
  peakToPeak = signalMax - signalMin;
  double volts = (peakToPeak * 5.0) / 1024;  // Convert to volts
  
  Serial.print("LP: ");
  Serial.println(volts);
  
  // Check if we should wake up to high precision mode
  if (volts > WAKE_THRESHOLD) {
    switchToHighPrecisionMode();
    return; // Switch immediately, don't delay
  }
  
  // Long delay for power savings
  delay(LOW_POWER_INTERVAL);
}

void highPrecisionSample() {
  unsigned long startMillis = millis();
  unsigned int peakToPeak = 0;
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // Use full precision sampling window
  while (millis() - startMillis < sampleWindow) {
    sample = analogRead(micPin);
    if (sample < 1024) {  // Toss out spurious readings
      if (sample > signalMax) {
        signalMax = sample;  // Save just the max levels
      }
      else if (sample < signalMin) {
        signalMin = sample;  // Save just the min levels
      }
    }
  }
  
  peakToPeak = signalMax - signalMin;
  double volts = (peakToPeak * 5.0) / 1024;  // Convert to volts
  
  Serial.print("HP: ");
  Serial.println(volts);
  
  // Add your detailed audio processing here
  processDetailedAudio(volts);
  
  // Check if we should return to low power mode
  if (volts < SLEEP_THRESHOLD) {
    quietCount++;
    if (quietCount >= QUIET_COUNT_LIMIT) {
      switchToLowPowerMode();
      return; // Switch immediately
    }
  } else {
    quietCount = 0;  // Reset quiet counter when activity detected
  }
  
  // Short delay for high precision monitoring
  delay(HIGH_PRECISION_INTERVAL);
}

void switchToHighPrecisionMode() {
  currentState = HIGH_PRECISION_MODE;
  quietCount = 0;
  Serial.println(">>> Switched to HIGH PRECISION mode");
}

void switchToLowPowerMode() {
  currentState = LOW_POWER_MODE;
  Serial.println(">>> Switched to LOW POWER mode");
}

void processDetailedAudio(double volts) {
  // Add your detailed audio analysis here
  // This is where you can add:
  // - Frequency analysis
  // - Pattern recognition
  // - Data logging
  // - Wireless transmission
  
  // Example: Log significant events
  if (volts > 0.2) {
    Serial.print("SIGNIFICANT ACTIVITY: ");
    Serial.println(volts);
  }
}
