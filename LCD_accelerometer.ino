// LCD setup
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS     10
#define TFT_RST    8
#define TFT_DC     9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Modulino accelerometer and gyroscope set up
#include "Modulino.h"

ModulinoMovement movement;

// Motion detection thresholds
const float MOTION_THRESHOLD = 0.20;  // g units for acceleration
const float ROTATION_THRESHOLD = 20.0;  // degrees per second

// Calibration variables
float baseX = 0, baseY = 0, baseZ = 1.0;  // Expect 1g on Z when flat
float baseRoll = 0, basePitch = 0, baseYaw = 0;
bool isCalibrated = false;

// Motion state tracking
bool inMotion = false;
unsigned long motionStartTime = 0;
unsigned long stillStartTime = 0;
const unsigned long STILL_TIMEOUT = 1500;  // 1.5 seconds to detect stillness

void setup() {
  // LCD setup
  Serial.begin(9600);
  Serial.println("ST7789 Test");

  tft.init(240, 240);   // Correct for 1.54"
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 100);
  tft.print("LCD initialized");
  // Modulino setup
  Modulino.begin();
  movement.begin();
  
  tft.fillScreen(ST77XX_BLACK);
  tft.print("Motion Detection System");
  tft.setCursor(30, 100);
  tft.print("======================");
  tft.setCursor(40, 100);
  tft.print("Keep device still for calibration...");
  
  delay(2000);
  calibrateSensor();
}

void calibrateSensor() {
  const int samples = 50;
  float sumX = 0, sumY = 0, sumZ = 0;
  float sumRoll = 0, sumPitch = 0, sumYaw = 0;
  
  for (int i = 0; i < samples; i++) {
    movement.update();
    sumX += movement.getX();
    sumY += movement.getY();
    sumZ += movement.getZ();
    sumRoll += movement.getRoll();
    sumPitch += movement.getPitch();
    sumYaw += movement.getYaw();
    delay(20);
  }
  
  baseX = sumX / samples;
  baseY = sumY / samples;
  baseZ = sumZ / samples;
  baseRoll = sumRoll / samples;
  basePitch = sumPitch / samples;
  baseYaw = sumYaw / samples;
  
  isCalibrated = true;
  tft.fillRect(40, 100, 10, 10, ST77XX_BLACK)
  tft.setCursor(40, 100);
  tft.print(" Calibration complete!");

}

void detectMotion() {
  movement.update();
  
  // Calculate deltas from baseline
  float deltaX = abs(movement.getX() - baseX);
  float deltaY = abs(movement.getY() - baseY);
  float deltaZ = abs(movement.getZ() - baseZ);
  float deltaRoll = abs(movement.getRoll() - baseRoll);
  float deltaPitch = abs(movement.getPitch() - basePitch);
  float deltaYaw = abs(movement.getYaw() - baseYaw);
  
  // Check if any threshold is exceeded
  bool motionDetected = (deltaX > MOTION_THRESHOLD || 
                         deltaY > MOTION_THRESHOLD || 
                         deltaZ > MOTION_THRESHOLD ||
                         deltaRoll > ROTATION_THRESHOLD ||
                         deltaPitch > ROTATION_THRESHOLD ||
                         deltaYaw > ROTATION_THRESHOLD);
  
  // State machine for motion detection
  if (motionDetected && !inMotion) {
    // Motion just started
    inMotion = true;
    motionStartTime = millis();
    Serial.println(" MOTION DETECTED!");
    tft.fillScreen(0x0000); // black out/clear screen
    tft.setCursor(20, 100);
    tft.print("Motion: Active");
    
  }
  else if (!motionDetected && inMotion) {
    // Motion might have stopped
    if (stillStartTime == 0) {
      stillStartTime = millis();
    }
    if (millis() - stillStartTime > STILL_TIMEOUT) {
      // Device has been still for timeout period
      inMotion = false;
      unsigned long duration = (millis() - motionStartTime) / 1000;
      Serial.print(" Motion stopped. Duration: ");
      Serial.print(duration);
      Serial.println(" seconds");
      stillStartTime = 0;
      tft.fillScreen(0x0000); // black out/clear screen
      tft.setCursor(20, 100);
      tft.print("Motion: Inactive");
    }
  }
  else if (motionDetected && inMotion) {
    // Still in motion, reset still timer
    stillStartTime = 0;
  }
}

void loop() {
  if (isCalibrated) {
    detectMotion();
    delay(50);
  }
}
