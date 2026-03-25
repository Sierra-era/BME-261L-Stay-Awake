const int PulsePin = A0;
const int LED = LED_BUILTIN;

float filtered = 512.0;
float alpha = 0.15;    

// NEW: Dynamic threshold variables
float movingAverage = 512.0;
float avgAlpha = 0.02; // Very slow moving average to track the wave's center

int Signal;
int BPM = 0;

unsigned long lastBeat = 0;
bool beatState = false;

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(9600); 
}

void loop() {
  Signal = analogRead(PulsePin);

  // 1. Center and Amplify
  float centeredSignal = Signal - 512.0;
  float amplifiedSignal = centeredSignal * 15.0;
  float finalSignal = amplifiedSignal + 512.0;

  // 2. Hardware filter did the heavy lifting, software smooths the rest
  filtered = (alpha * finalSignal) + ((1.0 - alpha) * filtered);

  // 3. NEW: Calculate the slow-moving average of the wave
  movingAverage = (avgAlpha * filtered) + ((1.0 - avgAlpha) * movingAverage);

  // 4. Beat detection: Trigger ONLY if the wave spikes significantly above the moving average
  // The "+ 30.0" acts as a noise buffer so empty room noise doesn't trigger a beat
  if (filtered > (movingAverage + 30.0) && !beatState) {
    beatState = true;
    digitalWrite(LED, HIGH);

    unsigned long now = millis();
    int interval = now - lastBeat;

    // Filter out impossible human heart rates
    if (interval > 300 && interval < 2000) {
      BPM = 60000 / interval;
    }
    lastBeat = now;
  }

  // 5. Reset the beat state when the wave dips back below its own average
  if (filtered < movingAverage) {
    beatState = false;
    digitalWrite(LED, LOW);
  }

  // Plotting 3 Lines to the Serial Plotter
  Serial.print(movingAverage); // Line 1: The Smart Threshold (Center line)
  Serial.print(",");
  Serial.print(filtered);      // Line 2: The Pulse Wave
  Serial.print(",");
  Serial.println(BPM);         // Line 3: The BPM

  delay(10); 
}