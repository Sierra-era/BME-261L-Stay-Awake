// ran it through chat, and this includes filtering, moving average for BPM, and auto-sets the threshold, so it should be less noisy 
////////////////////////////////////////
const int PulsePin = A0;
const int LED = LED_BUILTIN;

float filtered = 0;
float alpha = 0.1;    // smoothing factor (0.05–0.2 works well)

int Signal;
int BPM = 0;
int Threshold = 0;

unsigned long lastBeat = 0;
bool beatState = false;

// auto scaling
int minSignal = 1023;
int maxSignal = 0;

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
}

void loop() {

  Signal = analogRead(PulsePin);

  // Low-pass filter
  filtered = alpha * Signal + (1 - alpha) * filtered;

  // Track min/max for normalization
  if (filtered < minSignal) minSignal = filtered;
  if (filtered > maxSignal) maxSignal = filtered;

  // Normalize signal to large range
  int normalized = map(filtered, minSignal, maxSignal, 400, 700);

  // Dynamic threshold
  Threshold = (maxSignal + minSignal) / 2;

  // Beat detection
  if (filtered > Threshold && !beatState) {

    beatState = true;
    digitalWrite(LED, HIGH);

    unsigned long now = millis();
    int interval = now - lastBeat;

    if (interval > 300) {
      BPM = 60000 / interval;
      lastBeat = now;
    }

  }

  if (filtered < Threshold) {
    beatState = false;
    digitalWrite(LED, LOW);
  }

  Serial.print(normalized);
  Serial.print(",");
  Serial.println(BPM);

  delay(5);
}
