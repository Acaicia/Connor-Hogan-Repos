//For use on Aerogeddon, a rocket capable of self-guidance. This code's purpose is to:

//- Automatically set up sensors and detect failures, then advise the go-no-go decision
//- Detect remote initiation & ensure all fail-safes indicate a go status
//- Pressurize and inject an ignition fluid, then ignite the engine following a final commit check
//- Collect chamber pressure & temperature, exhaust temperature, thrust, and system status data
//- Convert the collected serial data to an excel database
//**NOTE - This is a simple sample of my work programming C++ for real-time embedded systems. For ITAR reasons, the full algorithm controlling the thrust vector mount/control 
//         ...surfaces is not included nor uploaded anywhere online.
int loopCount = 0;
int activated = activated;
unsigned long timer;
unsigned long millisUnmod;
unsigned long math;
unsigned long timerMS;
unsigned long startingMillis;
unsigned long startingMillis2;
int startupBegin = false;
int startupStatus = 0;
int cancelStatus = false;
int skipCancelCheck = false;
int countedTwo = false;
int countedOne = false;
int startupCountdown = true;
int countedIgnition = false;
int countedShutoffIgnitor = false;
int failed = 0;
int passed = 0;
#include <math.h>
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
const int HX711_dout = 2;
const int HX711_sck = 3;
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;
unsigned long t = 0;
int thrustRounded = 0;
unsigned long thrust = 0;
int countdownRelay = 0;
#include <Thermocouple.h>
#include <MAX6675_Thermocouple.h>
#define SCK_PIN_Internal 4
#define CS_PIN_Internal 5
#define SO_PIN_Internal 6
Thermocouple* thermocoupleInternal;
#include <Thermocouple.h>
#include <MAX6675_Thermocouple.h>
#define SCK_PIN_Exhaust 7
#define CS_PIN_Exhaust 8
#define SO_PIN_Exhaust 9
Thermocouple* thermocoupleExhaust;
const int pressureInput = A1;
const int pressureZero = 100;
const int pressureMax = 900;
const int pressuretransducermaxPSI = 300;
float pressureValue = 0;
int pressureRounded = 0;
int solenoidPin = 10;
int ignitorPin = 11;
const int actSwitch = 12;

void setup() {
  Serial.begin(9600); delay(10);
  Serial.println("CLEARSHEET");
  Serial.println("RESETTIMER");
  Serial.println("LABEL,Sample #, Timer, Thrust Grams,Exhaust Celsius,Exhaust Fahrenheit,Internal Celsius,Internal Fahrenheit,Chamber Pressure PSI, ");
  thermocoupleInternal = new MAX6675_Thermocouple(SCK_PIN_Internal, CS_PIN_Internal, SO_PIN_Internal);
  thermocoupleExhaust = new MAX6675_Thermocouple(SCK_PIN_Exhaust, CS_PIN_Exhaust, SO_PIN_Exhaust);
  LoadCell.begin();
  float calibrationValue;
#if defined(ESP8266)|| defined(ESP32)
#endif
  EEPROM.get(calVal_eepromAdress, calibrationValue);
  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Failed to read thrust load cell value, check wiring.");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue);
  }
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}
void loop() {
  while (startupCountdown == true) {
    while (digitalRead(actSwitch) == HIGH) {
      Serial.println("Activation switch already toggled on. Turn off and restart.");
      delay(1000);
    }
    thrust = LoadCell.getData();
    int thrustRounded = thrust;
    if (thrustRounded < 0) {
      thrustRounded = 0;
    }
    const double celsiusInternal = thermocoupleInternal->readCelsius();
    const double celsiusExhaust = thermocoupleInternal->readCelsius();
    pressureValue = analogRead(pressureInput);
    pressureValue = ((pressureValue - pressureZero) * pressuretransducermaxPSI) / (pressureMax - pressureZero);
    pressureRounded = round(pressureValue);
    if (pressureRounded < 0) {
      pressureRounded = 0;
    }
    Serial.print("CELL,SET,[BLOCK],Finished setup. Thrust: ");
    Serial.print(thrust);
    Serial.print(" grams. Internal Temp: ");
    Serial.print(celsiusInternal);
    Serial.print("C. Exhaust Temp: ");
    Serial.print(celsiusExhaust);
    Serial.print("C. Chamber Pressure: ");
    Serial.print(pressureRounded);
    Serial.println(" PSI.");
    Serial.println("Awaiting switch activation for automated ignition sequence.");
    while (digitalRead(actSwitch) == LOW) {   }
    Serial.println("Ignition Fluid Injection");
    Serial.println("CELL,SET,[BLOCK],Ignition Fluid Injection.");
    digitalWrite(solenoidPin, HIGH);
    delay(2000);
    digitalWrite(solenoidPin, LOW);
    Serial.println("Ignition in 3 seconds, starting data collection");
    Serial.println("CELL,SET,[BLOCK],Beginning Data Collection.");
    startupCountdown = false;
    startingMillis = millis();
  }
  timer = millis();
  timer = timer - startingMillis;
  timer = timer - startingMillis2;
  millisUnmod = millis();
  millisUnmod = millisUnmod - startingMillis;
  millisUnmod = millisUnmod - startingMillis2;
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;
  if (LoadCell.update()) {
    thrust = LoadCell.getData();
  }
  const double celsiusInternal = thermocoupleInternal->readCelsius();
  const double celsiusExhaust = thermocoupleInternal->readCelsius();
  const double fahrenheitInternal = celsiusInternal * 1.8 + 32;
  const double fahrenheitExhaust = celsiusExhaust * 1.8 + 32;
  pressureValue = analogRead(pressureInput);
  pressureValue = ((pressureValue - pressureZero) * pressuretransducermaxPSI) / (pressureMax - pressureZero);
  pressureRounded = round(pressureValue);
  if (pressureRounded < 0) {
    pressureRounded = 0;
  }
  Serial.print("DATA, ");
  loopCount = loopCount + 1;
  Serial.print(loopCount);
  Serial.print(", ");
  if (timer >= 1000) {
    timer = timer / 1000;
    math = timer * 1000;
    timerMS = millisUnmod - math;
    Serial.print(timer);
    Serial.print("s ");
    Serial.print(timerMS);
    Serial.print("ms, ");
  }
  else {
    Serial.print(timer);
    Serial.print("ms, ");
  }
  int thrustRounded = thrust;
  if (thrustRounded < 0) {
    thrustRounded = 0;
  }
  Serial.print(thrustRounded);
  Serial.print(", ");
  newDataReady = 0;
  t = millis();
  Serial.print(celsiusInternal);
  Serial.print(", ");
  Serial.print(fahrenheitInternal);
  Serial.print(", ");
  Serial.print(celsiusExhaust);
  Serial.print(", ");
  Serial.print(fahrenheitExhaust);
  Serial.print(", ");
  Serial.print(pressureRounded);
  Serial.print(", ");
  Serial.println("AUTOSCROLL_20");
  if (millisUnmod > 3000) {
    if (countedIgnition == false) {
      Serial.println("CELL,SET,[BLOCK],Ignition.");
      digitalWrite(ignitorPin, HIGH);
      countedIgnition = true;
    }
  }
  if (millisUnmod > 5000) {
    if (countedShutoffIgnitor == false) {
      digitalWrite(ignitorPin, LOW);
      countedShutoffIgnitor = true;
    }
  }
}
