
#include <SPI.h>
//#include <Wire.h>
//#include <Scout.h>
//#include <GS.h>
//#include <bitlash.h>
//#include <lwm.h>
//#include <js0n.h>


#define TINY_13_CS 3
#define GS_FLASH_CS 4
#define DRIVER_FLASH_CS 5
#define WIFI_PROGRAM_SELECT 6
#define WIFI_CS 7
#define MICRO_SD_CS 8
#define BP_FLASH_CS SS

#define DEBOUNCE_TIMEOUT 500

const int buttonPin = A3;
bool testIsRunning = false;
bool isInProgrammingMode = false;
uint32_t debounceTime;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  resetSPIChipSelectPins();
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(WIFI_PROGRAM_SELECT, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  ledOff();
  
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  
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

  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }

  // read from port 0, send to port 1:
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }

//  lcd.clearLCD();
//  delay(10);
//  Serial.println("test");
//  delay(2000);
}

void putWifiInProgramMode() {
  //Serial.println("- Putting Gainspan into program mode");
  ledOff();
  resetSPIChipSelectPins();
  isInProgrammingMode = true;

  digitalWrite(VCC_ENABLE, LOW);
  delay(1000);
  
  digitalWrite(WIFI_PROGRAM_SELECT, HIGH);

  digitalWrite(VCC_ENABLE, HIGH);
  delay(500);

  while(Serial1.read() != -1);
  setLedPurple();
}

void putWifiInRunMode() {
  //Serial.println("- Putting Gainspan into run mode");
  ledOff();
  resetSPIChipSelectPins();
  isInProgrammingMode = false;

  digitalWrite(VCC_ENABLE, LOW);
  delay(1000);

  digitalWrite(WIFI_PROGRAM_SELECT, LOW);
 
  digitalWrite(VCC_ENABLE, HIGH);
  delay(500);

  while(Serial1.read() != -1);
  setLedRed();
}

void resetSPIChipSelectPins() {
  /*
  SPI.end();
  pinMode(TINY_13_CS, OUTPUT);
  digitalWrite(TINY_13_CS, HIGH);
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
  SPI.end();
  */ 
  SPI.end();
  pinMode(MOSI, INPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, INPUT);
  pinMode(TINY_13_CS, INPUT);
  pinMode(TINY_13_CS, INPUT);
  pinMode(GS_FLASH_CS, INPUT);
  pinMode(DRIVER_FLASH_CS, INPUT);
  pinMode(WIFI_CS, INPUT);
  pinMode(MICRO_SD_CS, INPUT);
  pinMode(BP_FLASH_CS, INPUT);

  delay(10);
}

void ledOff(void) {
  digitalWrite(LED_RED, 255);
  digitalWrite(LED_GREEN, 255);
  digitalWrite(LED_BLUE, 255);
}

void setLedRed(void) {
  ledOff();
  digitalWrite(LED_RED, 0);
}

void setLedPurple() {
  ledOff();
  digitalWrite(LED_RED, 0);
  digitalWrite(LED_BLUE, 0);
}

