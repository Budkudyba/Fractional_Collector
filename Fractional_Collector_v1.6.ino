//Created By: Paul Kudyba, III
// 8-17-20 version 1.6
//to be used with a Arduino MEGA, RAMPS 1.4 Board, and RepRapDiscount Controller

#define VERSION "1.6"

//LCD
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0,23,17,16);//sck,mosi,cs

//encoder
static int pinA = 31;//encoder greycode clk
static int pinB = 33;//encoder greycode data
static int button = 35; //encoder button 
static int Estop = 41;//KILL_PIN 41 ?
int encoderPos = 0; 
//int encoderPinALast = LOW;
//int in = LOW;
static uint8_t prevNextCode = 0;
static uint16_t store=0;


//Home Switch
const int homeSwitch = 3; //X_MIN_PIN (can use interrupt)
bool HOMED = false;

// EEPROM
#include <EEPROM.h>
struct calibration {
  double uL2;
  double uL5;
  int set; //to see if EEPROM is valid is set to 43
  double sample;
  double purge;
  float interval;
};
calibration EEcal;//variable cal struct calibration to retrieve from eeprom

//RTC
#include <RTClib.h>
RTC_DS3231 rtc;

//Steppers
//#include "A4988.h" //changed to 8825
#include "DRV8825.h"
#define MOTOR_STEPS 200

//x axis (step, dir, en) (54,55,38)
//y axis (step, dir, en) (60,61,56)
//z axis (step, dir, en) (46,48,62)
//A4988 name(MOTOR_STEPS, DIR, STEP, ENABLE, MS1, MS2, MS3);
DRV8825 input(MOTOR_STEPS, 55, 54, 38); //x axis
DRV8825 output(MOTOR_STEPS, 61, 60, 56);//y axis
DRV8825 advance(MOTOR_STEPS, 48, 46, 62);//z axis
unsigned long totalMovedIn = 0;
unsigned long totalMovedOut = 0;
unsigned long totalMovedAd = 0;
int roundoffIn = 0; //used to align 200 steps to 1/6 rotations
int roundoffOut = 0;
double uL = 0; //volume difference from target
double uL2 = 39.3095; //default calibration 2mm dia pump
double uL5 = 2.20239; //default calibration 0.5mm dia pump
double cal = 0.0; // in mL used store amount to pump for calibration

//timers
unsigned long timer1 = 0;//used for encoder debounce
unsigned long timer2 = 0;//used for button encoder debounce
unsigned long debounce = 1;//5; //in ms
unsigned long buttonDebounce = 250; //in ms
unsigned long timer3 = 0;//used for rtc to trigger display update
unsigned long dispRTC = 1000; //in ms

//states, menu items, and variables
unsigned int state = 0;
int dispNeedsUpdate = 1;

double sample = 500;// in ul
double purge = 500;// in ul
float interval = 15.0; //in mins
DateTime startTime; // when to start
DateTime lastSampleTime;
int samplesTaken = 0; //total samples per experiment
int intervalSamples = 0; // samples taken without pause
DateTime nextInterval;
bool isClockSet = true;
bool powerLossWhileRunning = false;
int sampleLimit = 30;


//constants
const char States[16][20] = {"Start", "Reset Total Samples", "Tray Advance", "Start at Sample",
                            "Set Cycle Interval", "Set Sample Volume", "Set Purge Volume",
                            "Set Sample Limit",
                            "Cal. Input Pump", "Cal. Sample Pump",
                            "Set Clock Time", "Stats", "Sampler v"VERSION};

void setup() {
  //serial
  Serial.begin(115200);
  Serial.println("Welcome");
  Serial.println("Sampler v"VERSION);
  //LCD
  u8g2.begin();
  //Encoder and Pins
  pinMode(pinA, INPUT_PULLUP); 
  pinMode(pinB, INPUT_PULLUP);
  pinMode(button, INPUT_PULLUP);//active low
  pinMode(Estop, INPUT_PULLUP); //stop button on panel
  pinMode(homeSwitch, INPUT_PULLUP);
  
  //RTC
  rtc.begin();
  if (rtc.lostPower()){
    isClockSet = false;
    rtc.adjust(DateTime(2018, 7, 21, 0, 0, 55));
  }
  //Steppers
  input.begin(120);//120RPM
  input.disable();
  input.setMicrostep(8);
  output.begin(120);
  output.disable();
  output.setMicrostep(8);
  advance.begin(60);
  advance.disable();
  advance.setMicrostep(8);
  advance.setSpeedProfile(advance.LINEAR_SPEED, 2000, 1000);// enable accel,decel
  
  //Welcome Screen
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(14,30,"Welcome!");
  u8g2.sendBuffer();
  delay(2000);
  u8g2.setFont(u8g2_font_t0_12b_tf);
  
  //get calibrations from EEPROM
  Serial.println("Reading EEPROM");
  EEPROM.get(0, EEcal);//address,variable
  Serial.print("uL2: ");
  Serial.println(EEcal.uL2,5);
  Serial.print("uL5: ");
  Serial.println(EEcal.uL5,5);
  Serial.print("set: ");
  Serial.println(EEcal.set);
  Serial.print("sample: ");
  Serial.println(EEcal.sample,2);
  Serial.print("purge: ");
  Serial.println(EEcal.purge,2);
  Serial.print("interval: ");
  Serial.println(EEcal.interval,2);
  if(EEcal.set == 43) {//EEPROM has been written to before
    Serial.print("Loading from EEPROM...");
    uL2 = EEcal.uL2;
    uL5 = EEcal.uL5;
    sample = EEcal.sample;
    purge = EEcal.purge;
    interval = EEcal.interval;
    Serial.println("Success!");
  }
  if(EEcal.set != 43){//EEPROM has never been written to, and needs to be set
    Serial.println("EEPROM nan detected");
    Serial.println("Writing default values");
    EEcal.uL2 = uL2;
    EEcal.uL5 = uL5;
    EEcal.set = 43;
    EEcal.sample = sample;
    EEcal.purge = purge;
    EEcal.interval = interval;
    EEPROM.put(0,EEcal);
    Serial.println("Default values written");
  }
}

void loop() {
  if (millis() - timer1 > debounce){
    timer1 = millis();
    pollEncoder();
  }
  switch(state){
    case 0://------------------------------Start
      if (dispNeedsUpdate) {
        DateTime now = rtc.now();
        timer3 = millis(); //reset RTC timer
        u8g2.clearBuffer();
        u8g2.setCursor(2, 12);
        u8g2.print(States[state]);
        u8g2.setFont(u8g2_font_t0_11_tf);
        u8g2.setCursor(40, 15);
        String s = "Sample:";
        String ml = "mL";
        String t = s + sample/1000 + ml;
        u8g2.setCursor(45, 12);
        u8g2.print(t);
        String p = " Purge:";
        t = p + purge/1000 + ml;
        u8g2.setCursor(45, 22);
        u8g2.print(t);
        String q = "Every:";
        String mins = "mins";
        t = q + interval + mins;
        u8g2.setCursor(45, 31);
        u8g2.print(t);
        String col = ":";
        t = addLeadingZero(now.hour()) + col + addLeadingZero(now.minute()) + col + addLeadingZero(now.second());
        u8g2.setCursor(55, 64);
        u8g2.print(t);
        if (!isClockSet){
          u8g2.setCursor(2, 64);
          u8g2.print("SetClock!");
        }
        if (samplesTaken > 0){
          u8g2.setCursor(2, 53);
          String q = "Paused at Sample: ";
          t = q + addLeadingZero(samplesTaken);
          u8g2.print(t);      
        }
        u8g2.sendBuffer();
        u8g2.setFont(u8g2_font_t0_12b_tf);//reset font to default
        dispNeedsUpdate = 0;
      }
      if (encoderPos > 0){
        state++;
        encoderPos = 0;
        dispNeedsUpdate = 1;
        break;
      }
      if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
        timer2 = millis();
        state = 50;//waiting
        dispNeedsUpdate = 1;
        startTime = rtc.now();
        nextInterval = startTime.unixtime() + long(15);
        //Serial.println(startTime.unixtime());
        //Serial.println(nextInterval.unixtime());
        break;
      }
      if (millis() - timer3 > dispRTC){ //update clock display
        dispNeedsUpdate = 1;
      }
    break;
    case 1://------------------------------RESET TOTAL SAMPLES
      if (dispNeedsUpdate) {
        u8g2.clearBuffer();
        u8g2.setCursor(2, 12);
        u8g2.print(States[state]);
        String s = "Total Samples: ";
        String t = s + addLeadingZero(samplesTaken);
        u8g2.setCursor(2, 50);
        u8g2.print(t);
        u8g2.sendBuffer();
        dispNeedsUpdate = 0;
      }
      checkEncoderMenuStatus();
      if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
        timer2 = millis();
        samplesTaken = 0;
        intervalSamples = 0;
        HOMED = false;
        dispNeedsUpdate = 1;
      }
    break;
    case 2://------------------------------TRAY ADVANCE
      updateDisplay();
      checkEncoderMenuStatus();
      if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
        timer2 = millis();
        HOMED = false;
        advance.enable();
        advance.move(-578);
        advance.disable();
        dispNeedsUpdate = 1;
      }
    break;
    case 3://------------------------------START AT SAMPLE
      updateDisplayWithUnits(samplesTaken, "");
      checkEncoderMenuStatus();
      samplesTaken = changeValueRehome(samplesTaken, 1, 0, 29, "");//value, increment, min, max, unit
    break;
    case 4://------------------------------SET CYCLE INTERVAL
      updateDisplayWithUnits(interval, "min");
      checkEncoderMenuStatus();
      interval = changeValue(interval, 1, 1, 120.0, "min");//value, increment, min, max, unit
      EEcal.interval = interval;
      EEPROM.put(0,EEcal);
    break;
    case 5://------------------------------SET SAMPLE VOLUME
      updateDisplayWithUnits(sample, "uL");
      checkEncoderMenuStatus();
      sample = changeValue(sample, 10, 20, 1500, "uL");//value, increment, min, max, unit
      EEcal.sample = sample;
      EEPROM.put(0,EEcal);
    break;
    case 6://------------------------------SET PURGE VOLUME
      updateDisplayWithUnits(purge, "uL");
      checkEncoderMenuStatus();
      purge = changeValue(purge, 10, 20, 1500, "uL");//value, increment, min, max, unit
      EEcal.purge = purge;
      EEPROM.put(0,EEcal);
    break;
    case 7://------------------------------SET SAMPLE LIMIT
      updateDisplayWithUnits(sampleLimit, "");
      checkEncoderMenuStatus();
      sampleLimit = changeValue(sampleLimit, 1, 1, 30, "");//value, increment, min, max, unit
    break;
    case 8://------------------------------CAL INPUT PUMP
      updateDisplay();
      checkEncoderMenuStatus();
      if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
        timer2 = millis();
        dispNeedsUpdate = 1;
        cal = 90.0; // in mL
        int ini = 90000/uL2; //closest uL to target
        InMove6(ini, "Calibrate 90mL", uL2);
        do{
          if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
            dispNeedsUpdate = 1;
            timer2 = millis();
            break;
          }
          pollEncoder();
          if (encoderPos!=0){
            cal+=float(encoderPos)*0.1;
            if (cal < 0){
              cal = 0;
            }
            dispNeedsUpdate = 1;
            encoderPos = 0;
          }
          updateDisplayWithUnitsEDIT(cal,"mL");
        }while(true); 
        uL2 = uL2 * cal * 1000 / 90000; //update calibration
        //Store calibration in EEPROM
        EEcal.uL2 = uL2;
        EEPROM.put(0,EEcal);
        //Serial.println(uL2);      
      }
    break;
    case 9://------------------------------CAL SAMPLE PUMP
      updateDisplay();
      checkEncoderMenuStatus();
      if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
        timer2 = millis();
        dispNeedsUpdate = 1;
        cal = 8.0; // in mL
        int ini = 8000/uL5; //closest uL to target
        OutMove6(ini, "Calibrate 8mL", uL5);
        do{
          if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
            dispNeedsUpdate = 1;
            timer2 = millis();
            break;
          }
          pollEncoder();
          if (encoderPos!=0){
            cal+=float(encoderPos)*0.1;
            if (cal < 0){
              cal = 0;
            }
            dispNeedsUpdate = 1;
            encoderPos = 0;
          }
          updateDisplayWithUnitsEDIT(cal,"mL");
        }while(true); 
        uL5 = uL5 * cal * 1000 / 8000; //update calibration
        EEcal.uL5 = uL5;
        EEPROM.put(0,EEcal);
        //Serial.println(uL5);      
      }
    break;
    case 10://------------------------------SET TIME
      if (dispNeedsUpdate) {
        DateTime now = rtc.now();
        timer3 = millis(); //reset RTC timer
        u8g2.clearBuffer();
        u8g2.setCursor(2, 12);
        u8g2.print(States[state]);
        u8g2.setFont(u8g2_font_courB08_tf);
        String col = ":";
        String t = addLeadingZero(now.hour()) + col + addLeadingZero(now.minute());
        u8g2.setCursor(40, 25);
        u8g2.print(t);
        u8g2.sendBuffer();
        u8g2.setFont(u8g2_font_t0_12b_tf);
        dispNeedsUpdate = 0;
      }
      checkEncoderMenuStatus();
      
      if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
        dispNeedsUpdate = 1;
        timer2 = millis();
        DateTime now = rtc.now();
        int hr = now.hour();
        int mi = now.minute();
        do{
          if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
            dispNeedsUpdate = 1;
            timer2 = millis();
            rtc.adjust(DateTime(2018, 7, 21, hr, mi, 0));
            isClockSet = true;
            break;
          }
          pollEncoder();
          if (encoderPos!=0){
            mi += encoderPos;
            if (mi < 0){
              hr -= 1;
              mi = 59;
            }
            if (mi >= 60){
              hr += 1;
              mi = 0;
            }
            if (hr <= -1){
              hr = 23;
            }
            if (hr >= 24){
              hr = 0;
            }
            dispNeedsUpdate = 1;
            encoderPos = 0;
          }
          if (dispNeedsUpdate) {
            u8g2.clearBuffer();
            u8g2.setCursor(2, 12);
            u8g2.print(States[state]);
            u8g2.setFont(u8g2_font_t0_15_tf);
            String col = ":";
            String t = addLeadingZero(hr) + col + addLeadingZero(mi);
            u8g2.setCursor(40, 25);
            u8g2.print(t);
            u8g2.sendBuffer();
            u8g2.setFont(u8g2_font_t0_12b_tf);
            dispNeedsUpdate = 0;
          }
        }while(true);
      }
      if (millis() - timer3 > dispRTC){
        dispNeedsUpdate = 1;
      }
    break;
    case 11://------------------------------STATS
      if (dispNeedsUpdate) {
        u8g2.clearBuffer();
        u8g2.setCursor(2, 12);
        u8g2.print(States[state]);
        u8g2.setFont(u8g2_font_t0_11_tf);
        u8g2.setCursor(2, 25);
        String T = "sample cal: ";
        u8g2.print(T + uL5);
        u8g2.setCursor(2, 37);
        T = "input cal: ";
        u8g2.print(T + uL2);
        u8g2.setCursor(2, 49);
        T = "Volume Diff: ";
        u8g2.print(T + uL);
        u8g2.sendBuffer();
        u8g2.setFont(u8g2_font_t0_12b_tf);
        dispNeedsUpdate = 0;
      }
      checkEncoderMenuStatus();
      
    break;
    case 12://------------------------------RESET DEFAULTS
      updateDisplay();
      if (encoderPos < 0){
        state--;
        encoderPos = 0;
        dispNeedsUpdate = 1;
      }
    break;
    case 50://------------------------------IDLE
      advance.disable();
      if (dispNeedsUpdate) {
        DateTime now = rtc.now();
        timer3 = millis(); //reset RTC timer
        u8g2.clearBuffer();
        u8g2.setCursor(2, 12);
        u8g2.print("Waiting");
        
        u8g2.setFont(u8g2_font_t0_11_tf);
        String col = ":";
        String t = addLeadingZero(now.hour()) + col + addLeadingZero(now.minute());
        u8g2.setCursor(55, 64);
        u8g2.print(t);

        String s = "Total Samples: ";
        t = s + addLeadingZero(samplesTaken);
        u8g2.setCursor(2, 50);
        u8g2.print(t);

        s = "Run Samples: ";
        t = s + addLeadingZero(intervalSamples) + " of " + addLeadingZero(sampleLimit);
        u8g2.setCursor(2, 38);
        u8g2.print(t);
        
        DateTime countdown = nextInterval.unixtime() - now.unixtime();
        //Serial.println(countdown.unixtime());
        String n = "Next Sample: ";
        t = n + addLeadingZero(countdown.unixtime()/3600L) + col + addLeadingZero(countdown.unixtime()/60L) + col +
            addLeadingZero(countdown.unixtime()%60L);
        u8g2.setCursor(2, 25);
        u8g2.print(t);
        u8g2.sendBuffer();
        u8g2.setFont(u8g2_font_t0_12b_tf);
        dispNeedsUpdate = 0;
      }
      if (samplesTaken >= sampleLimit){
        state = 0;//Start
      }
      if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
        timer2 = millis();
        state = 0;//Start
        dispNeedsUpdate = 1;
        intervalSamples = 0;
        break;
      }
      if (nextInterval.unixtime() <= rtc.now().unixtime()){
        state = 51;//SAMPLING
        //Serial.println(rtc.now().unixtime());
      }
      if (millis() - timer3 > dispRTC){ //update clock display
        dispNeedsUpdate = 1;
      }
    break;
    case 51:{//------------------------------SAMPLING
      advance.enable();
      if (intervalSamples == 0){
        startTime = rtc.now();
        Serial.println(startTime.unixtime());  
      }
      dispNeedsUpdate = 1;
      while(!HOMED){
        if(dispNeedsUpdate){
          u8g2.clearBuffer();
          u8g2.setCursor(2, 25);
          u8g2.print("Homing Tray");
          u8g2.sendBuffer();
          dispNeedsUpdate = 0;
        }
        if (digitalRead(homeSwitch) == LOW){
          HOMED = true;
          Serial.println("Homed!");
          //move offset
          u8g2.clearBuffer();
          u8g2.setCursor(2, 25);
          u8g2.print("Moving To Sample");
          u8g2.sendBuffer();
          advance.move(-9971);//17.25 vials
          pollEstop();//check if Emergency Stop has been pushed
          //move to current sample if needed
          if (samplesTaken > 0){
            int k = (2 * samplesTaken) * -578;
            advance.move(k);
          }
        }
        pollEstop();//check if Emergency Stop has been pushed
        advance.move(-1);
      }

      //purge
      int smp = purge/uL5;
      OutMove6(smp, "Purging Line...", uL5);
      uL -= smp*uL5;
      delay(1000);//small pause for any drips?
      //advance to sample
      advance.move(-578);
      //sample
      smp = sample/uL5;
      OutMove6(smp, "Sampling...", uL5);
      uL -= smp*uL5;
      //advance to next purge
      advance.move(-578);
      //refill void volume
      float Vvoid = -uL;
      smp = floor(Vvoid/uL2);
      InMove6(smp, "Filling Void Volume...", uL2);
      uL += smp*uL2;
      dispNeedsUpdate = 1;
      state = 50;//IDLE
      samplesTaken++;
      intervalSamples++;
      nextInterval = startTime.unixtime() + long(interval * 60 *(intervalSamples));
      Serial.println(nextInterval.unixtime());
      Serial.print("Volume difference");
      Serial.println(uL);
    break;
    }
    default://--------------------------------DEFAULT(ERROR)
      u8g2.clearBuffer();
      u8g2.setCursor(0, 12);
      u8g2.print("ERROR!");
      u8g2.sendBuffer();
      dispNeedsUpdate = 0;
      Serial.println("Error");
    break;
  }
}
//=======================================================================================
//==================================FUNCTIONS============================================
//=======================================================================================
void checkEncoderMenuStatus(){
  if (encoderPos > 0){
    state++;
    encoderPos = 0;
    dispNeedsUpdate = 1;
  }
  if (encoderPos < 0){
    state--;
    encoderPos = 0;
    dispNeedsUpdate = 1;
  }
}

void updateDisplay(){
  if (dispNeedsUpdate) {
    u8g2.clearBuffer();
    u8g2.setCursor(2, 12);
    u8g2.print(States[state]);
    u8g2.sendBuffer();
    dispNeedsUpdate = 0;
    //Serial.println(States[state]);
  }
}
//used for showing static display with state and value
void updateDisplayWithUnits(float val, String unit){
  if (dispNeedsUpdate) {
    u8g2.clearBuffer();
    u8g2.setCursor(2, 12);
    u8g2.print(States[state]);
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(40, 25);
    String t = val + unit;
    u8g2.print(t);
    u8g2.sendBuffer();
    u8g2.setFont(u8g2_font_t0_12b_tf);//reset font
    dispNeedsUpdate = 0;
  }
}
//used to update display when editing a value (just a font change)
void updateDisplayWithUnitsEDIT(float val, String unit){
  if (dispNeedsUpdate) {
    u8g2.clearBuffer();
    u8g2.setCursor(2, 12);
    u8g2.print(States[state]);
    u8g2.setFont(u8g2_font_t0_15_tf);
    u8g2.setCursor(35, 30);
    String t = val + unit;
    u8g2.print(t);
    u8g2.sendBuffer();
    u8g2.setFont(u8g2_font_t0_12b_tf);//reset font
    dispNeedsUpdate = 0;
  }
}
// used to edit value of a system variable (value to change, increment, min, max, string suffix)
float changeValue(float val, float amnt, int minimum, int maximum,  String unit){
  if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
        dispNeedsUpdate = 1;
        timer2 = millis();
        do{
          if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
            dispNeedsUpdate = 1;
            timer2 = millis();
            return val;
          }
          pollEncoder();
          if (encoderPos!=0){
            val+=float(encoderPos)*amnt;
            val = constrain(val, minimum, maximum);
            dispNeedsUpdate = 1;
            encoderPos = 0;
          }
          updateDisplayWithUnitsEDIT(val,unit);
        }while(true);
      }
}

// used to edit value of a system variable (value to change, increment, min, max, string suffix)
int changeValueRehome(int val, float amnt, int minimum, int maximum,  String unit){
  if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
        dispNeedsUpdate = 1;
        timer2 = millis();
        do{
          if (!digitalRead(button) && millis() - timer2 > buttonDebounce){
            dispNeedsUpdate = 1;
            HOMED = false;
            timer2 = millis();
            return val;
          }
          pollEncoder();
          if (encoderPos!=0){
            val+=float(encoderPos)*amnt;
            val = constrain(val, minimum, maximum);
            dispNeedsUpdate = 1;
            encoderPos = 0;
          }
          updateDisplayWithUnitsEDIT(val,unit);
        }while(true);
      }
}

void pollEncoder(){
  static int8_t rot_enc_table[] = {0,1,1,0,1,0,0,1,1,0,0,1,0,1,1,0};

  prevNextCode <<= 2;
  if (digitalRead(pinB)) prevNextCode |= 0x02;
  if (digitalRead(pinA)) prevNextCode |= 0x01;
  prevNextCode &= 0x0f;

   // If valid then store as 16 bit data.
   if  (rot_enc_table[prevNextCode] ) {
      store <<= 4;
      store |= prevNextCode;
      //if (store==0xd42b) return 1;
      //if (store==0xe817) return -1;
      if ((store&0xff)==0x2b) encoderPos = -1;
      if ((store&0xff)==0x17) encoderPos = 1;
   }else encoderPos = 0;
}

String addLeadingZero(int t){
  String zero = "0";
  if (t<10){
    return zero + String(t);
  }else{
    return String(t);
  }
}

void InMove6(int moves, String disc, double uL){
  timer3 = millis(); //reset RTC timer
  input.enable();
  String u = "uL";
  int sign = 0; // change direction based on if moves is negative or positive
  if (moves > 0)
    sign = 1;
  if (moves < 0){
    sign = -1;
    moves = moves * -1; //make moves positive
  }
  u8g2.clearBuffer();
  u8g2.setCursor(2, 12);
  u8g2.print(disc);
  u8g2.setCursor(25, 25);
  String v = String(0.0);
  u8g2.print(v+u);
  u8g2.sendBuffer();
  for (int i=1;i<=moves;i++) {
    pollEstop();//check if Emergency Stop has been pushed  
    if (roundoffIn == 2) {
      input.move(268*sign);  // ~1/6 moves
      totalMovedIn += 268;
      roundoffIn = 0;
    }else{
      input.move(266*sign);  // ~1/6 moves
      totalMovedIn += 266;
      roundoffIn += 1;
    }
    if (millis() - timer3 > dispRTC){ //update clock display
      u8g2.clearBuffer();
      u8g2.setCursor(2, 12);
      u8g2.print(disc);
      u8g2.setCursor(25, 25);
      String v = String(i*uL);
      u8g2.print(v+u);
      u8g2.sendBuffer();
      timer3 = millis(); //reset RTC timer
    }
  }
  input.disable();
}

void OutMove6(int moves, String disc, double uL){
  timer3 = millis(); //reset RTC timer
  output.enable();
  String u = "uL";
  int sign = 0; // change direction based on if moves is negative or positive
  if (moves > 0)
    sign = 1;
  if (moves < 0){
    sign = -1;
    moves = moves * -1; //make moves positive
  }
  u8g2.clearBuffer();
  u8g2.setCursor(2, 12);
  u8g2.print(disc);
  u8g2.setCursor(25, 25);
  String v = String(0.0);
  u8g2.print(v+u);
  u8g2.sendBuffer();
  for (int i=1;i<=moves;i++) {
    pollEstop();//check if Emergency Stop has been pushed  
    if (roundoffIn == 2) {
      output.move(268*sign);  // ~1/6 moves
      totalMovedOut += 268;
      roundoffOut = 0;
    }else{
      output.move(266*sign);  // ~1/6 moves
      totalMovedOut += 266;
      roundoffOut += 1;
    }
    if (millis() - timer3 > dispRTC){ //update clock display
      u8g2.clearBuffer();
      u8g2.setCursor(2, 12);
      u8g2.print(disc);
      u8g2.setCursor(25, 25);
      String v = String(i*uL);
      u8g2.print(v+u);
      u8g2.sendBuffer();
      timer3 = millis(); //reset RTC timer
    }
  }
  output.disable();
}

void pollEstop(){
  if(!digitalRead(Estop)){
    u8g2.clearBuffer();
    u8g2.setCursor(2, 12);
    u8g2.print("Emergency Stop!");
    u8g2.sendBuffer();
    output.disable();
    input.disable();
    advance.disable();
    while(true){
      delay(1000);
    }
  }
}
