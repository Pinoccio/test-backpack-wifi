
#include <SPI.h>
#include <Wire.h>
#include <Scout.h>
#include <GS.h>
#include <bitlash.h>
#include <lwm.h>
#include <js0n.h>


#define TINY_13_RESET 3
#define GS_FLASH_CS 4
#define DRIVER_FLASH_CS 5
#define WIFI_PROGRAM_SELECT 6
#define WIFI_CS 7
#define MICRO_SD_CS 8
#define BP_FLASH_CS SS

#define DEBOUNCE_TIMEOUT 1000

const int buttonPin = A3;
bool testIsRunning = false;
bool isInProgrammingMode = false;
uint32_t debounceTime;
  
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  resetSPIChipSelectPins();
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(VCC_ENABLE, OUTPUT);
  pinMode(WIFI_PROGRAM_SELECT, OUTPUT);
  debounceTime = millis();
  
  putWifiInProgramMode();
}

void loop() {
  if (digitalRead(buttonPin) == LOW && (millis() - debounceTime > DEBOUNCE_TIMEOUT)) {
  //if (digitalRead(buttonPin) == LOW) {
    debounceTime = millis();
    if (isInProgrammingMode == true) {
      putWifiInRunMode();
    } else {
      putWifiInProgramMode();
    }
  }
  
  if (Serial1.available() > 0) {
    Serial.write(Serial1.read());
  }

  // read from port 0, send to port 1:
  if (Serial.available() > 0) {
    Serial1.write(Serial.read());
  }
 
//  lcd.clearLCD();
//  delay(10);
//  Serial.println("test");
//  delay(2000);
}

void putWifiInProgramMode() {
  //Serial.println("- Putting Gainspan into program mode");
  RgbLed.turnOff();
  isInProgrammingMode = true;  
  digitalWrite(WIFI_PROGRAM_SELECT, HIGH);
  
  digitalWrite(VCC_ENABLE, LOW);
  delay(500);

  digitalWrite(VCC_ENABLE, HIGH);
  delay(500);
  
  while(Serial1.read() != -1);
  RgbLed.purple();  
}

void putWifiInRunMode() {
  //Serial.println("- Putting Gainspan into run mode");
  RgbLed.turnOff();
  isInProgrammingMode = false;
  
  digitalWrite(WIFI_PROGRAM_SELECT, LOW);
  digitalWrite(VCC_ENABLE, LOW);
  delay(500);
  
  digitalWrite(VCC_ENABLE, HIGH);
  delay(500);
  
  while(Serial1.read() != -1);
  RgbLed.red();
}

void resetSPIChipSelectPins() {
  pinMode(TINY_13_RESET, OUTPUT);
  digitalWrite(TINY_13_RESET, HIGH);
  pinMode(GS_FLASH_CS, OUTPUT);
  digitalWrite(GS_FLASH_CS, HIGH);
  pinMode(DRIVER_FLASH_CS, OUTPUT);
  digitalWrite(DRIVER_FLASH_CS, HIGH);
  pinMode(WIFI_CS, OUTPUT);
  digitalWrite(WIFI_CS, HIGH);
  pinMode(MICRO_SD_CS, OUTPUT);
  digitalWrite(MICRO_SD_CS, HIGH);
  pinMode(BP_FLASH_CS, OUTPUT);
  digitalWrite(BP_FLASH_CS, HIGH);
  delay(1);
}
