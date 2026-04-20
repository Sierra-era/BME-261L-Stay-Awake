/* 
BME 261L
Code for Stay Awake Smart Watch
Written by: Sierra Nguyen, Ali Elhagehassan, Bilal Saadi, Hannah Dvorachek,

This code takes input from Arduino Modulino movement accelerometer, DFRobot Heart rate sensor to determine if the user is falling asleep
displays BPM, current time and how long the user is still for on the GODIYMODULES 1.54" TFT LCD Display and has a setings menu which is interacted with through a Sparkfun Rotary encoder
It also activates the Tinkersphere Haptic Feedback Module and DFRobot Vibration Motor module when the user is determined to be falling asleep to wake the user up

reference code LCD menu
https://educ8s.tv/arduino-rotary-encoder-menu/

reference code rotary encoder interrupts
https://gist.github.com/legomushroom/1b0ba79851a549048f6ab302700be796

reference code for DFRobot HR Monitor
https://github.com/DFRobot/DFRobot_Heartrate

reference code for Arduino Modulino Movement
https://docs.arduino.cc/tutorials/modulino-movement/how-movement/

*/
// vibration //
#define DFvibpin 3
#define tinkervibpin 4


// RTC //
#define RTC_interruptpin 20 // A6
#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 myRTC;
DateTime now = myRTC.now();
DateTime alarm1Time = DateTime(now.year(), now.month(), now.day(), 0, 0, 0);
int alarm_year = now.year();
int alarm_month = now.month();
int alarm_day = now.day();
int alarm_hour = 0;
int alarm_minute = 0;

//// rotary encoder set up ////
// [D2][GND][D3]
// [D4][GND]
#define pinA 7
#define pinB 6
#define pinbutton 5

volatile boolean halfleft = false;      // Used in both interrupt routines
volatile boolean halfright = false;

boolean up = false;
boolean down = false;
boolean middle = false;

unsigned long lastDebounceTime = 0;
//// LCD setup ////
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>

#define TFT_CS     8
#define TFT_RST    10
#define TFT_DC     9

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
// Color definitions
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF

//// menu ////
volatile int menuitem = 1; // changed in interrupts from rotation
int lastMenuItem = menuitem; // to know when to update lcd

volatile int page = 1; //chnaged in interrupts when button pressed 
int oldpage = page; // might not need this
// page 1 = BPM, stillness, time readout
// page 2 = settings menu
// page 3  = change specific menu item value
// page 5 = sleeping alert

String menuItem1 = "Mode";
String menuItem2 = "BPM Cutoff";
String menuItem3 = "Stillness Cutoff";
String menuItem4 = "Vibration Intensity";
String menuItem5 = "Set Alarm";
// String menuItem6 = "Reset"; // from reference code 

String Mode[2] = {"Idle/Inactive", "Alert"}; // 1
int selected_mode = 1; //1
int BPM_Threshold = 10; // 10% base // 2
int Still_Sleep_Threshold = 20; // 20 s base // 3
String Vib_int[3] = {"Low", "Medium", "High"}; // 4
int selected_vib_int = 1; //4
int Alarm_Time = 0; // 5
int time_item = 1;
static bool drawn = false; // static means var keeps value between func calls
//// Modulino accelerometer and gyroscope set up ////
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

unsigned long duration_motion = 0;
unsigned long old_duration_motion = 1;
unsigned long duration_still = 0;
unsigned long old_duration_still = 1;

// HR Monitor
#define heartratePin A1
#include "DFRobot_Heartrate.h"

DFRobot_Heartrate heartrate(DIGITAL_MODE); ///< ANALOG_MODE (shows raw wavelength) or DIGITAL_MODE (better bc we only need to count BPM)

uint8_t rateValue = 0;
uint8_t old_rateValue = 1;
uint8_t newRate =0;

unsigned long lastBPMUpdate = 0;
const int BPM_INTERVAL = 20; // ms

int n = 0; // # of times bpm has been called and used for averageing
float BPM_average =0;
float BPM_oldaverage = 0;
// sleep logic
bool sleep_status = false;

void setup() {
  Serial.begin(9600);
  // RTC
  myRTC.begin();
  Wire.begin(); // I2C
  if (myRTC.lostPower()) {
    Serial.println("RTC lost power, setting time!");
    myRTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  // Uncomment if you need to adjust the time
  // myRTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  myRTC.disable32K();
  pinMode(RTC_interruptpin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_interruptpin), onalarm, FALLING);
  // reset alarms
  myRTC.clearAlarm(1);
  myRTC.clearAlarm(2);
  // stop oscillating signals at SQW Pin, otherwise setAlarm1 will fail
  myRTC.writeSqwPinMode(DS3231_OFF);
  // turn off alarm 2 (in case it isn't off already)
  // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
  myRTC.disableAlarm(2);

  // LCD setup
  

  tft.init(240, 240); 
  tft.setRotation(1);
  tft.fillScreen(ST77XX_WHITE);

  // rotary encoder
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinbutton, INPUT_PULLUP);
  
  attachInterrupt(pinA, isr_2, FALLING);   // Call isr_2 when digital pin 2 goes LOW
  attachInterrupt(pinB, isr_3, FALLING);   // Call isr_3 when digital pin 3 goes LOW
  // its bad to read button in interrupt so its read in loop

  // Modulino setup lets user know to hold still then call callibration func
  Modulino.begin();
  movement.begin();
  
  tft.fillScreen(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
  tft.setCursor(5, 100);
  tft.print("Keep device still");
  tft.setCursor(5, 120);
  tft.print("for calibration...");
  
  delay(2000);
  calibrateSensor();

}

void loop() {

  drawHomepage(); // draws homepage
  readrotary(); // checks for rotary encoder inputs
  detectMotion(); // checks for movement
  
  if (millis() - lastBPMUpdate > BPM_INTERVAL) { // gets BPM value
    heartrate.getValue(heartratePin);
    rateValue = heartrate.getRate(); // Get heart rate value 
    lastBPMUpdate = millis();
    n++; 
    BPM_average = BPM_oldaverage + 1/n*(rateValue-BPM_oldaverage); // itertive average of all BPM values
    if(rateValue) {
    Serial.println(rateValue);
    }
    if (page == 1 && rateValue != old_rateValue) {  
    tft.fillRect(10, 10, 150, 20, ST77XX_WHITE);
    tft.setCursor(10, 10);
    tft.print("BPM: ");
    tft.print(rateValue); 
    old_rateValue = rateValue;
    }
  }
  
  drawMenu(); // checks to draw settings menu
  determinesleep(); // check for sleeping
  checkalarm(); // checks if alarm set or clears if done
}

// if on page one draw homepage once to prevent  flickering and if page changes reset to allow homepage to be drawn again
void drawHomepage(){
  if (page != 1){
    drawn = false;
  }
  if (page ==1){
    if (page == 1 && !drawn){
      // vib turn off redundancy
      analogWrite(DFvibpin,0);
      analogWrite(tinkervibpin,0);

      tft.fillScreen(ST77XX_WHITE);  // clear ONCE
      tft.setTextSize(2);
      tft.setTextColor(ST77XX_BLACK,ST77XX_WHITE);
      tft.setCursor(10, 10);
      tft.print("BPM: ");
      tft.print(rateValue);
      tft.drawFastHLine(0,30,240,ST77XX_BLACK);
      
      
      tft.setCursor(10, 40);
      tft.fillRect(10,40,240,15,ST77XX_WHITE);
      tft.print("Time Still: ");
      tft.print(duration_still);
      tft.drawFastHLine(0,60,240,ST77XX_BLACK);
      
      tft.setCursor(10,80);
      DateTime now = myRTC.now();
      tft.print(now.month(), DEC);
      tft.print('/');
      tft.print(now.day(), DEC);
      tft.print('/');
      tft.print(now.year(), DEC);
      

      tft.setCursor(10,100);
      tft.print(now.hour(), DEC);
      tft.print(":");
      tft.print(now.minute(), DEC);
      drawn = true;
    }
  }
  
      
}

void drawMenu(){
  //Serial.println("drawMenu");
  if (page ==2){
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_BLACK,ST77XX_WHITE);
    int x = (240 - 13 * 12) / 2; // rough centering (6 chars × ~12px each)
    tft.setCursor(x, 10);
    tft.print("SETTINGS MENU");
    tft.drawFastHLine(0,30,240,ST77XX_BLACK); // draws underline for aestetics

    // start of printing menu items depending on what is selected it will be highlighted
    if(menuitem==1)
    {   
      displayMenuItem(menuItem1, 40,true);
      displayMenuItem(menuItem2, 70,false);
      displayMenuItem(menuItem3, 100,false);
      displayMenuItem(menuItem4, 130,false);
      displayMenuItem(menuItem5, 160,false);
    }
    else if(menuitem == 2)
    {
      displayMenuItem(menuItem1, 40,false);
      displayMenuItem(menuItem2, 70,true);
      displayMenuItem(menuItem3, 100,false);
      displayMenuItem(menuItem4, 130,false);
      displayMenuItem(menuItem5, 160,false);
    }
    else if(menuitem == 3)
    {
      displayMenuItem(menuItem1, 40,false);
      displayMenuItem(menuItem2, 70,false);
      displayMenuItem(menuItem3, 100,true);
      displayMenuItem(menuItem4, 130,false);
      displayMenuItem(menuItem5, 160,false);
    }
     else if(menuitem == 4)
    {
      displayMenuItem(menuItem1, 40,false);
      displayMenuItem(menuItem2, 70,false);
      displayMenuItem(menuItem3, 100,false);
      displayMenuItem(menuItem4, 130,true);
      displayMenuItem(menuItem5, 160,false);
    }
    else if(menuitem == 5)
    {
      displayMenuItem(menuItem1, 40,false);
      displayMenuItem(menuItem2, 70,false);
      displayMenuItem(menuItem3, 100,false);
      displayMenuItem(menuItem4, 130,false);
      displayMenuItem(menuItem5, 160,true);
    }
  }
  // start of page 3 opening settings specific to make changes
  else if (page==3 && menuitem == 1) 
  {    
   displayStringMenuPage(menuItem1, Mode[selected_mode] );
  }
  else if (page==3 && menuitem == 2) 
  {
   displayIntMenuPage(menuItem2, BPM_Threshold );
   tft.print("%");
  }
   else if (page==3 && menuitem == 3) 
  {
   displayIntMenuPage(menuItem3, Still_Sleep_Threshold );
  }
  else if (page==3 && menuitem == 4) 
  {
   displayStringMenuPage(menuItem4, Vib_int[selected_vib_int] );
  }
  else if (page==3 && menuitem == 5) // same func but added second value bc hour and minute and hard coded selction
  {
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
    int x = (240 - menuItem5.length() * 12) / 2; // rough centering ( chars × ~12px each)
    tft.setCursor(x, 10);
    tft.print(menuItem5);
    tft.drawFastHLine(0,40,240,ST77XX_BLACK);
    tft.setCursor(10, 50);
    tft.print("Alarm Hour:Minute");
    tft.setTextSize(2);
    tft.setCursor(10, 80);
    if (time_item == 1){
      tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK); // if selected highlight
    }else{
      tft.setTextColor(ST77XX_BLACK,ST77XX_WHITE);
    }
    tft.print(alarm_hour);
    tft.print(":");
    if (time_item == 2){
      tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK); // if selected highlight
    }else{
      tft.setTextColor(ST77XX_BLACK,ST77XX_WHITE);
    }
    tft.print(alarm_minute);

  }
}

// used for highlighting selected item
void displayMenuItem(String item, int position, boolean selected){
    if(selected) // if selected "highlight item"
    {
      tft.setTextColor(ST77XX_WHITE,ST77XX_BLACK); // sets text color to white on black background
    }else // if not selected leave as it 
    {
      tft.setTextColor(ST77XX_BLACK,ST77XX_WHITE); // sets text color to black on white background
    }
    tft.setCursor(0, position); 
    tft.print(">"+item); // prints > menu item
}

// used for displaying changeable value / set value integer settings
void displayIntMenuPage(String menuItem, int value){
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
    int x = (240 - menuItem.length() * 12) / 2; // rough centering ( chars × ~12px each)
    tft.setCursor(x, 10);
    tft.print(menuItem);
    tft.drawFastHLine(0,40,240,ST77XX_BLACK);
    tft.setCursor(10, 50);
    tft.print("Value");
    tft.setTextSize(2);
    tft.setCursor(10, 80);
    tft.print(value);
    tft.setTextSize(2);
}

// used for arbitrary string based settings ie intensity and mode // same func different input format
void displayStringMenuPage(String menuItem, String value){
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
    int x = (240 - menuItem.length() * 12) / 2; // rough centering ( chars × ~12px each)
    tft.setCursor(x, 10);
    tft.print(menuItem);
    tft.drawFastHLine(0,40,240,ST77XX_BLACK);
    tft.setCursor(10, 50);
    tft.print("Value");
    tft.setTextSize(2);
    tft.setCursor(10, 80);
    tft.print(value);
    tft.setTextSize(2);
}

void alarmcheck(){ // draws menu bc to much to call in isr 

  if (page == 5){
  tft.fillScreen(ST77XX_WHITE);  // clear ONCE
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_BLACK,ST77XX_WHITE);
  tft.setCursor(10, 10);
  tft.print("Press button to stop alarm");
  } else{ // turns off vib if not on page 5
    analogWrite(DFvibpin,0);
    analogWrite(tinkervibpin,0);
  }
}

// rotary interrupt isr functions 
// half left and rights prevent double counts
void isr_2(){                                              // Pin2 went LOW
  if(digitalRead(2) == LOW){                               // Pin2 still LOW ?
    if(digitalRead(3) == HIGH && halfright == false){      // if Pin3 is HIGH
      halfright = true;                                    // One half click clockwise
    }  
    if(digitalRead(3) == LOW && halfleft == true){         // if Pin3 is LOW
      halfleft = false;                                    // One whole click counter-
      down = true;                                           // clockwise
    }
  }
}
void isr_3(){                                             // Pin3 went LOW
  if(digitalRead(3) == LOW){                              // Pin3 still LOW ?
    if(digitalRead(2) == HIGH && halfleft == false){      // <--
      halfleft = true;                                    // One half  click counter-
    }                                                     // clockwise
    if(digitalRead(2) == LOW && halfright == true){       // -->
      halfright = false;                                  // One whole click clockwise
      up = true;
    }
  }
}

// for pushing the rotary button and interacting with the menu
void readrotary(){
  unsigned long debounceDelay = 100;

  bool button_reading = digitalRead(pinbutton);

  if ( button_reading == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    if ( page == 2 && menuitem == 5){
      // RTC 
      if (time_item == 1){
        time_item = 2;
      }else{
        time_item = 1;
        page++;
      }
    }else if (page <3){
    page++;
    tft.fillScreen(ST77XX_WHITE);
    } else if(page >= 3 ) {
      page = 1;
      tft.fillScreen(ST77XX_WHITE);
    }
  }

  if (up && page == 2 && menuitem>1 ){ // move up on menu item in settings menu
    up = false;
    menuitem--;
  } else if ( up && page == 3){ // change menu item value
    tft.fillRect(10,80,240,15,ST77XX_WHITE); // clear previous value
    up = false;
    if (menuitem == 1){
      selected_mode--;
      if(selected_mode==-1){
        selected_mode = 1;
      }
    } else if (menuitem == 2){
      BPM_Threshold--;
      if ( BPM_Threshold == 0){
        BPM_Threshold =30;
      }
      
    } else if (menuitem == 3){
      Still_Sleep_Threshold--;
      if (Still_Sleep_Threshold ==0){
        Still_Sleep_Threshold = 60;
      }
    } else if (menuitem == 4){
      selected_vib_int--;
      if(selected_vib_int == -1){
        selected_vib_int = 2;
      }
    } else if (menuitem == 5){
      if (time_item == 1){
        alarm_hour--;
        if(alarm_hour== -1 ){
          alarm_hour = 24;
        }
      }else if(time_item ==2){
        alarm_minute--;
        if(alarm_minute == -1){
          alarm_minute=59;
        }
      }
    }
    
  }

  if (down && page == 2 && menuitem<5 ) { //move down on menu item 
    down = false;
    menuitem++;
  } else if ( down && page == 3){ // change menu item value
    tft.fillRect(10,80,240,15,ST77XX_WHITE); // clear previous value
    down = false;
    if (menuitem == 1){
      selected_mode++;
      if(selected_mode==2){
        selected_mode =0;
      }
    } else if (menuitem == 2){
      BPM_Threshold++;
      if ( BPM_Threshold == 51){
        BPM_Threshold =0;
      }
      
    } else if (menuitem == 3){
      Still_Sleep_Threshold++;
      if (Still_Sleep_Threshold ==61){
        Still_Sleep_Threshold = 0;
      }
    } else if (menuitem == 4){
      selected_vib_int++;
      if(selected_vib_int == 3){
        selected_vib_int = 0;
      }
    } else if (menuitem == 5){
      if (time_item == 1){
        alarm_hour++;
        if(alarm_hour== 25 ){
          alarm_hour = 0;
        }
      }else if(time_item ==2){
        alarm_minute++;
        if(alarm_minute == 60){
          alarm_minute=0;
        }
      }
    }
    
  }
}

// callibrates accelerometer and sets "zero"
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
  tft.fillScreen(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
  tft.setCursor(0, 100);
  tft.print("Calibration complete");
  delay(2000);
  tft.fillScreen(ST77XX_WHITE);
}

// detects if movement was big enough to consider movement and counts how long moving/not moving
void detectMotion() {
  movement.update();

  float deltaX = abs(movement.getX() - baseX);
  float deltaY = abs(movement.getY() - baseY);
  float deltaZ = abs(movement.getZ() - baseZ);
  float deltaRoll = abs(movement.getRoll() - baseRoll);
  float deltaPitch = abs(movement.getPitch() - basePitch);
  float deltaYaw = abs(movement.getYaw() - baseYaw);

  bool motionDetected = (deltaX > MOTION_THRESHOLD || 
                         deltaY > MOTION_THRESHOLD || 
                         deltaZ > MOTION_THRESHOLD ||
                         deltaRoll > ROTATION_THRESHOLD ||
                         deltaPitch > ROTATION_THRESHOLD ||
                         deltaYaw > ROTATION_THRESHOLD);

  // --- MOTION START ---
  if (motionDetected && !inMotion) {
    inMotion = true;
    motionStartTime = millis();
    stillStartTime = 0;   // reset still timer
  }

  // --- STILL START ---
  if (!motionDetected && inMotion) {
    if (stillStartTime == 0) {
      stillStartTime = millis();
    }

    if (millis() - stillStartTime > STILL_TIMEOUT) {
      inMotion = false;
      duration_motion = (millis() - motionStartTime) / 1000;
      stillStartTime = millis();  // start still timing NOW
    }
  }

  // --- STILL COUNTING ---
  if (!motionDetected && !inMotion && stillStartTime != 0) {
    duration_still = (millis() - stillStartTime) / 1000;

    if (page == 1 && duration_still != old_duration_still) {
      tft.fillRect(10, 40, 240, 20, ST77XX_WHITE);
      tft.setCursor(10, 40);
      tft.print("Time Still: ");
      tft.print(duration_still);
      old_duration_still = duration_still;
    }
  }

  // --- MOTION AGAIN ---
  if (motionDetected && inMotion) {
    stillStartTime = 0;
    duration_still = 0;
    if (page == 1 && duration_still != old_duration_still){
      tft.fillRect(10, 40, 240, 20, ST77XX_WHITE);
      tft.setCursor(10, 40);
      tft.print("Time Still: ");
      tft.print(duration_still);
      old_duration_still = duration_still;
    }
  }
}

// determines if sleeping
void determinesleep() {
  if ( page == 5){
    tft.fillScreen(ST77XX_WHITE);  // clear ONCE
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_BLACK,ST77XX_WHITE);
    tft.setCursor(10, 10);
    tft.print("Press button to stop alarm");
  }
  if (duration_still > Still_Sleep_Threshold && rateValue < BPM_average * (BPM_Threshold / 100.0)){
    sleep_status = true;
    page =5;
    if( selected_vib_int = 0){ // if low
      analogWrite(tinkervibpin,255);
    } else if (selected_vib_int == 1){ // if medium
      analogWrite(DFvibpin,255);
    } else if (selected_vib_int == 2){ // if high
      analogWrite(DFvibpin,255);
      analogWrite(tinkervibpin,255);
    }
    tft.fillScreen(ST77XX_WHITE);  // clear ONCE
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_BLACK,ST77XX_WHITE);
    tft.setCursor(10, 10);
    tft.print("Press button to stop alarm");
  } else {
    sleep_status = false;
    // turn off vib redundancy
    analogWrite(DFvibpin,0);
    analogWrite(tinkervibpin,0);
  }
}

// interrupt func
void onalarm() { // isr for alarm interrupt 
  page =5;
  if( selected_vib_int == 0){ // if low
    analogWrite(tinkervibpin,255);
  } else if (selected_vib_int == 1){ // if medium
    analogWrite(DFvibpin,255);
  } else if (selected_vib_int == 2){ // if high
    analogWrite(DFvibpin,255);
    analogWrite(tinkervibpin,255);
  }
}

// check if scheduled alarm is same
void checkalarm() {
  if(!myRTC.setAlarm1(alarm1Time, DS3231_A1_Hour)) {  // this mode triggers the alarm when the minutes match
    Serial.println("Error, alarm wasn't set!");
  }else {
    Serial.println("Alarm 1 will happen at specified time");
  }
  // reset if already fired
  if (myRTC.alarmFired(1)) {
    myRTC.clearAlarm(1);
    Serial.println(" - Alarm cleared");
}
}
