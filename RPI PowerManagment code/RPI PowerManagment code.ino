//User defined

#define BATMAX 63 // max battery cappacity in Wat Hours
#define VMAX 12.59 //over voltage protection of the battery
#define VMIN 9.8 // under volatage protection of the battery
#define ADCmaxVOLTin 2615 // read voltage*100 // depending on used resistor for the divider
#define ADCmaxVOLTbat 1912 // read voltage*100 // depending on used resistor for the divider
#define MAXbatA 1.5 //sensor can read max 14.8 A
#define MAXinA 5 //sensor can read max 14.8 A
#define BatMinProcent 10 // at waht % of the battery it will shout down
#define VinMinCharge 15 // minimal voltage for direct charging
#define DpowerMin 7.5 //direct power (bypassing the battery) minimum voltage
#define InpowerMin 4.8 // minimum voltage for the boost converter
#define OFFdelay 5000 // shut down delay
#define OverTempHigh 60 // defineing the temperature of the overtemperature
#define OverTempLOW 50 // defining when the overtemperature will let it charge again
#define UnderTemp 5 // defining the minimum temperature of charging


#include <SmoothThermistor.h>
#include "Button2.h"
#include <Wire.h>

#define _TASK_SLEEP_ON_IDLE_RUN
#define _TASK_PRIORITY
#define _TASK_WDT_IDS
#define _TASK_TIMECRITICAL
#include <TaskScheduler.h>

//output pins
#define ChargeQ PIN_PA5
#define BoostQ PIN_PC5
#define DirectQ PIN_PC2
#define BatQ PIN_PC0
#define ENC PIN_PB7
#define PWM PIN_PA4

// input pins
#define VIN PIN_PA6
#define BATV PIN_PA7
#define GUZ PIN_PB6
#define GUZ2 PIN_PA2
#define GUZ3 PIN_PC1
#define Tlad PIN_PA1
#define Tbat PIN_PA3
#define Ain PIN_PB4
#define Abat PIN_PB5
#define C2 PIN_PC4
#define C1 PIN_PC3

// thermistor library (a realy easy library for thermistors)
SmoothThermistor useAREF(VDD);
SmoothThermistor smoothThermistor(Tlad, ADC_SIZE_10_BIT, 10000, 10000, 3950, 25, 10);
SmoothThermistor smoothThermistor2(Tbat, ADC_SIZE_10_BIT, 10000, 10000, 3950, 25, 10);

Button2 button1 = Button2(GUZ);
Button2 button2 = Button2(GUZ2);
Button2 button3 = Button2(GUZ3);

String command, Sinput;
int GU = 0;
int GU2 = 0;
int GU3 = 0;
int TladMAX = 60;
float Vin, Vbat, T1, T2, batprocent;
boolean Dpower = false;
boolean Bpower = false;
boolean boost = false;
boolean charge = false;
boolean directCharge = false;
boolean PowerIsOn = false;
boolean PowerUpRequest = false;
boolean PowerDownRequest = false;
boolean PowerDownPermition = false;
boolean PowerAvailable = false;
boolean BATignore = false;
boolean chargeAllowed = false;
boolean DpowerAviable = false;
boolean INpowerAviable = false;
boolean overVolt = false;
boolean underVolt = false;
boolean overCurrentBat = false;
boolean overCurrentIn = false;
boolean overT = false;
bool Pon = false;
bool Poff = false;
float Ai, Bi;
float sek = 0;
long czas = 0;

// variables for power mesurment
float totalAMPcharge = 0;
float averageAmpcharge = 0;
float ampSecondcharge = 0;
float ampHourscharge = 0;
float wattHourscharge = 0;

float batwh = 0;
float Wat_RPI = 0;
float batstart = 0;

float BATwh = 0;
float Win = 0;
float Wbat = 0;

long startczas = 0;
long sample = 0;
int startflag = 1;
float AinS, AbatS, VinS, VbatS;

#include <Smoothed.h>
Smoothed <float> mySensor;
Smoothed <float> mySensor2;
Smoothed <float> mySensor3;
Smoothed <float> mySensor4;

void Serialout();
void READtemp();
void charg();
void Power();
void amp();
void PowerON();
void PowerOFF();
void buttonsH();
void loopTS();
void handler();

// task scheduler priority and timings

Scheduler r;
Scheduler pr;
Scheduler spr;

Task t1(5000, TASK_FOREVER, &Serialout, &r);
Task t2(1000, TASK_FOREVER, &READtemp, &r);
Task t3(500, TASK_FOREVER, &charg, &pr);
Task t4(10, TASK_FOREVER, &Power, &spr);
Task t5(100, TASK_FOREVER, &amp, &spr);
Task t6(500, TASK_FOREVER, &PowerON, &r);
Task t7(500, TASK_FOREVER, &PowerOFF, &r);
Task t8(25, TASK_FOREVER, &buttonsH, &spr);
Task t9(50, TASK_FOREVER, &loopTS, &pr);

void setup() {

  r.setHighPriorityScheduler(&pr);
  pr.setHighPriorityScheduler(&spr);
  r.enableAll(true);

  smoothThermistor.useAREF(true);
  smoothThermistor2.useAREF(true);

  pinMode(C1, OUTPUT);
  pinMode(C2, INPUT_PULLUP);
  pinMode(BatQ, OUTPUT);
  digitalWrite(BatQ, LOW);
  pinMode(ENC, OUTPUT);
  digitalWrite(ENC, HIGH);
  pinMode(ChargeQ, OUTPUT);
  pinMode(BoostQ, OUTPUT);
  pinMode(DirectQ, OUTPUT);
  digitalWrite(ChargeQ, LOW);
  digitalWrite(BoostQ, LOW);
  digitalWrite(DirectQ, LOW);

  Serial.begin(115200);

  button1.setClickHandler(handler);
  button1.setLongClickHandler(handler);
  button1.setDoubleClickHandler(handler);

  button2.setClickHandler(handler);
  button2.setLongClickHandler(handler);
  button2.setDoubleClickHandler(handler);

  button3.setClickHandler(handler);
  button3.setLongClickHandler(handler);
  button3.setDoubleClickHandler(handler);

  mySensor.begin(SMOOTHED_EXPONENTIAL, 5);
  mySensor2.begin(SMOOTHED_EXPONENTIAL, 5);
  mySensor3.begin(SMOOTHED_EXPONENTIAL, 5);
  mySensor4.begin(SMOOTHED_EXPONENTIAL, 5);

  mySensor.clear();
  mySensor2.clear();
  mySensor3.clear();
  mySensor4.clear();
}

void buttonsH() {
  // chcecking the buttons if anything was pressed
  button1.loop();
  button2.loop();
  button3.loop();
}

void Serialout() {

  // comunication whith the raspberry pi true UART
  // can also be done in I2C but i didn't code it in 
  // or simple 2 pin "C1" and "C2" custom comunication

  Serial.print("Vbat : "); Serial.println(Vbat);
  Serial.print("Vin : "); Serial.println(Vin);
  Serial.print("T1 : "); Serial.println(T1);
  Serial.print("T2 : "); Serial.println(T2);

  if (charge == true) {
    Serial.print("charge : "); Serial.println("ON");
    if (boost == true) {
      Serial.print("boost : "); Serial.println("boosting");
    } else {
      Serial.print("boost : "); Serial.println("direct");
    }
  } else {
    Serial.print("charge : "); Serial.println("OFF");
  }

  if (Dpower == true) {
    Serial.print("power : "); Serial.println("Direct");
  } else if (Bpower == true) {
    Serial.print("power : "); Serial.println("Battery");
  } else {
    Serial.print("power : "); Serial.println("OFF");
  }

  Serial.print("WHbat : "); Serial.println(wattHourscharge);
  Serial.print("Wat in : "); Serial.println(Win);
  Serial.print("wat bat : "); Serial.println(Wbat);
  Serial.print("Wat Usage: "); Serial.println(Wat_RPI);
  Serial.print("Ain : "); Serial.println(Ai, 4);
  Serial.print("Abat : "); Serial.println(Bi, 4);
  Serial.print("BAT : "); Serial.println(batprocent);
  Serial.print("Ahcharge : "); Serial.println(ampHourscharge);

  Serial.print("Power : "); Serial.println(PowerIsOn);

}

void READtemp() {
  T1 = smoothThermistor.temperature();
  T2 = smoothThermistor2.temperature();


//reading the temperature and checking if its too high
  if (T1 < TladMAX && T2 > UnderTemp && T2 < TladMAX) {
    overT = false;
    TladMAX = OverTempHigh;
  } else {
    overT = true;
    TladMAX = OverTempLOW;
  }
}

void charg() {
  // if we are charging but no current is going into the battery we asume its full
  if (charge == true && Bi < 0.2 && Bi > -0.2) {
    batstart = BATMAX - wattHourscharge;
  }

// checking if we can charge and if we need to use the step-up converter
  if (INpowerAviable == true && chargeAllowed == true) {
    if (Vin > VinMinCharge) {
      charge = true;
      boost = false;
      directCharge = true;
      digitalWrite(BoostQ, LOW);
      digitalWrite(ChargeQ, HIGH);
    } else {
      boost = true;
      charge = true;
      directCharge = false;
      digitalWrite(ChargeQ, LOW);
      digitalWrite(BoostQ, HIGH);
    }
  } else {
    charge = false;
    boost = false;
    directCharge = false;
    digitalWrite(ChargeQ, LOW);
    digitalWrite(BoostQ, LOW);
  }
}

void Power() {

  VinS = analogRead(VIN);
  VbatS = analogRead(BATV);
  mySensor.add(VinS);
  mySensor2.add(VbatS);

  Vin = mySensor.get();
  Vbat = mySensor2.get();

  Vin = map (analogRead(VIN), 0, 1023, 0, ADCmaxVOLTin);
  Vbat = map (analogRead(BATV), 0, 1023, 0, ADCmaxVOLTbat);

  Vin = Vin / 100;
  Vbat = Vbat / 100;

//checking for over voltage
  if (Vbat > VMAX) {
    overVolt = true;
  } else {
    overVolt = false;
  }
  //checking for under voltage
  if (Vbat < VMIN) {
    underVolt = true;
  } else {
    underVolt = false;
  }

//checking if we have enough power for to directly power the pi or we need to boost it
  if (Vin > InpowerMin) {
    INpowerAviable = true;
  } else {
    INpowerAviable = false;
  }
  if (Vin > DpowerMin) {
    DpowerAviable = true;
  } else {
    DpowerAviable = false;
  }

// determining where to draw the power from
  if (DpowerAviable == true) {
    Dpower = true;
    Bpower = false;
  } else {
    Dpower = false;
    if (batprocent > BatMinProcent ) {
      Bpower = true;
    } else if (BATignore == true) {
      Bpower = true;
    } else if (PowerIsOn == true) {
      Bpower = true;
      if (BATignore == false) {
        PowerDownRequest = true;
      }
    } else {
      Bpower = false;
    }
  }

  //over current check and shut down of the power
  if (overCurrentBat == true || overCurrentIn == true) {
    Dpower = false;
    Bpower = false;
  }

  //under voltage check and shut down of the power
  if (underVolt == true) {
    Bpower = false;
  }

  if (Dpower == false) {
    digitalWrite(DirectQ, LOW);
  } else {
    digitalWrite(DirectQ, HIGH);
  }
  if (Bpower == false) {
    digitalWrite(BatQ, LOW);
  } else {
    digitalWrite(BatQ, HIGH);
  }

}

void amp() {

  czas = millis();
  AinS = analogRead(Ain);
  AbatS = analogRead(Abat);
  mySensor3.add(AinS);
  mySensor4.add(AbatS);

  Ai = mySensor3.get();
  Bi = mySensor4.get();

// current sensors convertions
  Ai = map(Ai, 0, 1023, 0, 3300);
  Ai = Ai - 1650;
  Ai = Ai * 0.0118;
  Ai = Ai + 0.1062;

  Bi = map(Bi, 0, 1023, 0, 3300);
  Bi = Bi - 1650;
  Bi = Bi * 0.009;
  Bi = -Bi;
  Bi = Bi - 0.045;


// over current checks
  if (Ai > MAXinA || Ai < -0.2) {
    overCurrentIn = true;
  } else {
    overCurrentIn = false;
  }
  if (Bi > MAXbatA || Bi < -MAXbatA) {
    overCurrentBat = true;
  } else {
    overCurrentBat = false;
  }


// calculation for AH, WH, Charge state
  sample ++;
  sek = czas - startczas;
  sek = sek / 1000;

  totalAMPcharge = totalAMPcharge + Bi;
  averageAmpcharge = totalAMPcharge / sample;
  ampSecondcharge = averageAmpcharge * sek;
  ampHourscharge = ampSecondcharge / 3600;
  wattHourscharge = Vbat * ampHourscharge;

   Win = Vin * Ai;
  Wbat = Vbat * Bi;

  batwh = batstart + wattHourscharge;
  batprocent = map(batwh, 0, BATMAX, 0, 100);

  if (Ai > Bi) {
    Wat_RPI = Win - Wbat;
  } else {
    Wat_RPI = Wbat;
  }
}

void loopTS() {

// checks for allowing to charge the battery, over voltage and temperature and current check
  if (overT == false && overVolt == false && overCurrentBat == false && overCurrentIn == false ) {
    chargeAllowed = true;
  } else {
    chargeAllowed = false;
  }

// checking if we want to power down
  if (PowerDownRequest == true && PowerIsOn == true) {
    if (PowerDownPermition == false) {
      Serial.print("PowerDownRequest :"); Serial.println(PowerDownRequest);
      //waiting to permition from the RPI for ever
    } else {
      Poff = true;
      Pon = false;
    }
  }

// checking if we want to turn on
  if (PowerUpRequest == true && PowerIsOn == false) {
    //checking for awiable power
    if (Bpower == true || Dpower == true) {
      Pon = true;
      Poff = false;
    }
    PowerUpRequest = false;
  }


// recieving commands from the Raspberry Pi (work in progress)
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');

    if (command.equals("PDP")) {
      PowerDownPermition = true;
    }
    if (command.equals("PDR")) {
      PowerDownRequest = true;
    }
    if (command.equals("BAT")) {
      command = Serial.readStringUntil('\n');
      batstart = command.toInt();
      batstart = batstart / 10;
      Serial.print("ok, bat "); Serial.println(batstart);

    }
  }
}

void loop() {
  r.execute();
}

void PowerON() {
  if (Pon == true) {
    Poff = false;
    Pon = false;
    //checking for again aviable power before powering on
    if (Dpower == true || Bpower == true) {
      digitalWrite(ENC, LOW);
      PowerIsOn = true;
      PowerUpRequest = false;
    }
  }
}

void PowerOFF() {
  //when we power off we reset the flags
  if (Poff == true) {
    Poff = false;
    Pon = false;
    BATignore = false;
    PowerIsOn = false;
    PowerDownRequest = false;
    PowerDownPermition = false;
    delay(OFFdelay);
    digitalWrite(ENC, HIGH);
  }
}

void handler(Button2 & btn) {
  if (btn == button1) {
    switch (btn.getClickType()) {
      case SINGLE_CLICK:
        if (PowerIsOn == false) {
          PowerUpRequest = true;
          Serial.print("GU :1");
        } else {
          Serial.print("GU :1");
        }
        break;
      case DOUBLE_CLICK:
        if (PowerIsOn == true) {
          PowerDownRequest = true;
          Serial.print("GU :2");
        } else {
          Serial.print("GU :2");
        }
        break;
      case LONG_CLICK:
        if (PowerIsOn == false) {
          BATignore = true;
          PowerUpRequest = true;
          Serial.print("GU :3");
        } else {
          PowerDownRequest = true;
          PowerDownPermition = true;
          Serial.print("GU :3");
        }
        break;
    }
  } else if (btn == button2) {
    switch (btn.getClickType()) {
      case SINGLE_CLICK:
        Serial.print("GU2 :1");
        break;
      case DOUBLE_CLICK:
        Serial.print("GU2 :2");
        break;
      case LONG_CLICK:
        Serial.print("GU2 :3");
        break;
    }
  } else if (btn == button3) {
    switch (btn.getClickType()) {
      case SINGLE_CLICK:
        Serial.print("GU3 :1");
        break;
      case DOUBLE_CLICK:
        Serial.print("GU3 :2");
        break;
      case LONG_CLICK:
        Serial.print("GU3 :3");
        break;
    }
  }
}
