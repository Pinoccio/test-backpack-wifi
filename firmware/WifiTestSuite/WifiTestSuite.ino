#include <serialGLCDlib.h>
#include <SPI.h>
#include <Wire.h>
#include <PBBP.h>
#include <Scout.h>
#include <utility/WiFiBackpack.h>
#include "programmer.h"

serialGLCD lcd;
#define TINY_13_RESET 3
#define GS_FLASH_CS 4
#define DRIVER_FLASH_CS 5
#define WIFI_PROGRAM_SELECT 6
#define WIFI_CS 7
#define MICRO_SD_CS 8
#define BP_FLASH_CS SS

const int buttonPin = A3;
bool testIsRunning = false;
bool wifiFlashUpgradeRunning = false;
bool testFailed = false;

char buffer[256];
int ctr = 0;
bool wifiResultReady = false;

uint32_t S2W_APP1_IMG_ADDR = 0x10000;
uint32_t S2W_APP2_IMG_ADDR = 0x50000;
uint32_t WFW_REL_IMG_ADDR = 0xA0000;
uint32_t WIFI_EXTERNAL_FLASH_IMG_ADDR = 0x100000;

uint32_t bpUniqueId = 0x4F;

PBBP bp;
  
void setup() {
  Serial.begin(115200);
  testJigSetup();
}

void loop() {
  testJigLoop();
}

void testJigSetup() {
  pinMode(buttonPin, INPUT_PULLUP);
  RgbLed.cyan();
  Serial.println("Wi-Fi Test Jig ready to go!");
}

void testJigLoop() {
  if (digitalRead(buttonPin) == LOW) { 
    startTest();
  }
 
//  lcd.clearLCD();
//  delay(10);
//  Serial.println("test");
//  delay(2000);
}

void startTest() {
  RgbLed.turnOff();
  if (testIsRunning == true) {
    Serial.println("Test is already running");
    return;
  }
  testIsRunning = true;
  testFailed = false;

  // getWifiMACAddress();
  // writeWifiFlash();
  // updateWifiSettings();
  // updateWifiFirmwareWFW();
  // updateWifiFirmwareApp1();
  // updateWifiFirmwareApp2();
  // writeWifiMACAddress();
  
  // configureWifi();
  // testWifi();
  
  testSerialFlash();
  
  flashBackpackBus();
  testBackpackBus();
  
  testIsRunning = false;
  
  if (testFailed == false) {
    RgbLed.green();
  } else {
    RgbLed.red();
  }
  Serial.println("Test complete");
  testJigSetup();
}

void writeWifiFlash() {
  Serial.println("- Write Wi-Fi Flash -");

  if (!putWifiToSleep()) {
    Serial.println("FAIL: Wi-Fi module did not go to sleep");
    testFailed = true;
    return;
  }
  
  byte buffer[256] = {0};
  byte readBuffer[256] = {0};
  
  FlashClass DriverFlash(DRIVER_FLASH_CS, SPI);

  uint32_t start = millis();
  
  while (!DriverFlash.available()) {
    if (millis() - start > 1000) {
      Serial.println("FAIL: Driver Serial flash chip not found");
      testFailed = true;
      return;
    }
  }
  Serial.println("--- Driver Serial flash chip found");
  DriverFlash.end();

  FlashClass GSFlash(GS_FLASH_CS, SPI);
  start = millis();
  
  while (!GSFlash.available()) {
    if (millis() - start > 1000) {
      Serial.println("FAIL: GS Serial flash chip not found");
      testFailed = true;
      return;
    }
  }
  Serial.println("--- GS Serial flash chip found");
  
  Serial.println("--- Erasing chip");
  GSFlash.bulkErase();
  
  for (uint32_t i=0; i<0x100000; i+=256) {
    if (i % 100000 == 0) {
      Serial.print("-- Writing to address: 0x");
      Serial.println(i, HEX);
    }
    
    GSFlash.end();
    Serial.println("--- Reading 256 bytes from driver flash");
    DriverFlash.begin(DRIVER_FLASH_CS, SPI);
    DriverFlash.read(WIFI_EXTERNAL_FLASH_IMG_ADDR + i, &buffer, 256);
    DriverFlash.end();
    Serial.println("--- Writing 256 bytes to GS flash");
    GSFlash.begin(GS_FLASH_CS, SPI);
    GSFlash.write(i, &buffer, 256);
    Serial.println("--- Checking that write was successful");
    GSFlash.read(0xF0000, &readBuffer, 256);
    if (strncmp((const char *)buffer, (char *)readBuffer, 256) != 0) {
      Serial.print("FAIL: GS Serial failed to write correctly at address: 0x");
      Serial.println(i, HEX);
      printHex((const uint8_t *)buffer, 256);
      Serial.println();
      printHex((const uint8_t *)readBuffer, 256);
      Serial.println();
      testFailed = true;
    }
  }
}

void configureWifi() {
  Serial.println("- Configure Wi-Fi -");
  
  if (!Gainspan.setup(9600)) {
    if (!Gainspan.setup(115200)) {
      Serial.println("FAIL: Wi-Fi not responding");
      testFailed = true;
    } 
    Gainspan.init();
  }
  Serial.println("--- passed");
  /*
  uint32_t start = millis();
  bool timedOut = false;
  
  Serial1.println("AT");
  
  while (!wifiResultReady) {
    if (millis() - start > 30) {
      timedOut = true;
      configureWifi(115200);
      return;
    }
    waitForResponse();
  }
  wifiResultReady = false;

  if (strncmp((const char*)buffer, "AT", 2) != 0) {
    Serial.println("FAIL: Wi-Fi not responding");
    testFailed = true;
  }
  
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  if (strncmp((const char*)buffer, "OK", 2) == 0) {
    Serial.println("--- AT passed");
  } else {
    Serial.println("FAIL: AT didn't return OK");
    testFailed = true;
  }
  
  */
  
  return;
}

void testWifi() {
  Serial.println("- Test Wi-Fi -");
  
  if (Gainspan.getAppVersion() != "2.5.1" ||
      Gainspan.getGepsVersion() != "2.5.1" ||
      Gainspan.getWlanVersion() != "2.5.1") {
    Serial.println("Wi-Fi module requires a firmware upgrade");
    flashWifi();
  } else {
    Serial.println("--- passed");
  }
  
  /*
  Serial1.println("AT+VER=?");
  
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  if (strncmp((const char*)buffer, "AT+VER=?", 8) != 0) {
    Serial.println("FAIL: Wi-Fi not responding");
    testFailed = true;
  }
  
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  if (strncmp((const char*)buffer, "S2W APP VERSION=2.4.3", 21) == 0) {
    Serial.println("--- Latest firmware (2.4.3) is loaded ");
  } else {
    Serial.println("FAIL: Old firmware found, entering Wi-Fi upgrade mode");
    testFailed = true;
    flashWifi();
  } 
  
  Serial1.println("AT+WS");
  
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  if (strncmp((const char*)buffer, "AT+WS", 5) != 0) {
    Serial.println("FAIL: Wi-Fi not responding");
    testFailed = true;
  }
  
  // header of output, ignore
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  // blank line
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  // blank line
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  if (strncmp((const char*)buffer, " 90:84:0d:d6:8f:ab, Pinoccio", 29) == 0) {
    Serial.println("--- Found Pinoccio access point");
  } else {
    Serial.println("FAIL: Wi-Fi AP scan failed to find Pinoccio");
    testFailed = true;
  } 
  */
}

void flashWifi() {
  putWifiInProgramMode();
  
  Serial.println("- Ready to start Wi-Fi upgrade. Close serial monitor and upgrade using gs_programmer");
  wifiFlashUpgradeRunning = true;
  
  while (wifiFlashUpgradeRunning == true) {
  
    // read from port 1, send to port 0:
    while (Serial1.available()) {
      int inByte = Serial1.read();
      Serial.write(inByte); 
    }
  
    // read from port 0, send to port 1:
    while (Serial.available()) {
      int inByte = Serial.read();
      Serial1.write(inByte); 
    }
  }
  
  return;
}

void testSerialFlash() {
  putWifiInProgramMode();
  Serial.println("- Test Serial Flash -");
  
  FlashClass Flash(BP_FLASH_CS, SPI);
  
  const int addr = 0x10000;
  uint32_t start = millis();
  
  while (!Flash.available()) {
    if (millis() - start > 1000) {
      Serial.println("FAIL: Serial flash chip not found");
      testFailed = true;
      return;
    }
  }

  Serial.println("--- Serial flash chip found");
 
  Serial.println("--- Erasing subsector");
  Flash.subSectorErase(addr);
  
  char dataToChip[128] = "Testing123456789";
  char dataFromChip[128] = "                ";
  
  memset(dataFromChip, ' ', 16);
  dataFromChip[15] = 0;

  // Write some data to flash
  Flash.write(addr, dataToChip, 16);

  // Read it back to a different buffer
  Flash.read(addr, dataFromChip, 16);

  // Write it to the serial port
  if (strcmp((const char*)dataToChip, (const char*)dataFromChip) != 0) {
    Serial.print("FAIL: Data failed to write to and read from flash at address: ");
    testFailed = true;
    Serial.println(addr, HEX);
    Serial.println(dataToChip);
    Serial.println(dataFromChip);
  }
 
  if (testFailed == false) {
    Serial.println("--- Data written to and successfully read from serial flash");
  }

  Serial.println("--- Erasing test data");
  Flash.subSectorErase(addr);
  Flash.end();
  return;
}

void flashBackpackBus() {
  putWifiInProgramMode();
  Serial.println("- Flash Backpack Bus Program -");
  
  AVRProgrammer pgm = AVRProgrammer(TINY_13_RESET, SPI, SPI_CLOCK_DIV128);
  pgm.startProgramming();
  pgm.getSignature();
  pgm.getFuseBytes();
  
  // if we found a signature, try to write flash
  if (pgm.foundSignature() != -1) {
    
    Serial.println("-- erasing chip");
    pgm.eraseChip();
    
    Serial.println("-- writing fuses");
    if (!pgm.writeFuseBytes(0x21, 0xFB, 0xFF)) {
      testFailed = true;
      Serial.println("FAIL: writing fuses failed");
      pgm.end();
      return; 
    }
    
    Serial.println("-- writing flash");
    if (!pgm.writeProgram(0x0000, attiny13a_flash, sizeof(attiny13a_flash))) {
      testFailed = true;
      Serial.println("FAIL: writing flash failed");
      pgm.end();
      return; 
    }
    
    Serial.println("-- writing EEPROM");
    const byte * eeprom = attiny13a_eeprom;
    
    for (int i=0; i<64; i++) {
//      Serial.print("0x");
//      Serial.print(i, HEX);
//      Serial.print(": ");
      if (i == 6) { // this is our unique ID, so get from settings
//        Serial.println(bpUniqueId, HEX);
        pgm.writeEeprom(i++, (bpUniqueId >> 16) & 0xFF);
        pgm.writeEeprom(i++, (bpUniqueId >> 8) & 0xFF);
        pgm.writeEeprom(i, (bpUniqueId >> 0) & 0xFF);
      } else {
        pgm.writeEeprom(i, pgm_read_byte(eeprom + i));
//        Serial.println(pgm_read_byte(eeprom + i), HEX);
      }
    }
    Serial.println("-- reading EEPROM");
    for (int i=0; i<64; i++) {
//      Serial.print("0x");
//      Serial.print(i, HEX);
//      Serial.print(": ");
      if (i == 6) { // this is our unique ID, so get from settings
        uint32_t tmpUniqueId = 0;
        tmpUniqueId |= pgm.readEeprom(i++) << 16;
        tmpUniqueId |= pgm.readEeprom(i++) << 8;
        tmpUniqueId |= pgm.readEeprom(i) << 0;
//        Serial.println(tmpUniqueId, HEX);
        if (tmpUniqueId != bpUniqueId) {
          Serial.print("FAIL: EEPROM failed to write unique ID");
          Serial.print("- Expected: ");
          Serial.println(bpUniqueId);
          Serial.print("- Read: ");
          Serial.println(tmpUniqueId);
          testFailed = true;
          return;
        }
      } else {
        if (pgm_read_byte(eeprom + i) != pgm.readEeprom(i)) {
          Serial.print("FAIL: EEPROM failed to write correctly at address: 0x");
          Serial.println(i, HEX);
          Serial.print("- Expected: ");
          Serial.println(pgm_read_byte(eeprom + i));
          Serial.print("- Read: ");
          Serial.println(pgm.readEeprom(i));
          testFailed = true;
          return;
        }
//        Serial.println(pgm.readEeprom(i), HEX);
      }
    }
  } else {
    testFailed = true;
  }

  Serial.println("-- complete");
  pgm.end();
  return;
}

void testBackpackBus() {
  putWifiInProgramMode();
  Serial.println("- Test Backpack Bus -");
  
  bp.begin(BACKPACK_BUS);
  
  if (bp.enumerate()) {
      Serial.print("Found ");
      Serial.print(bp.num_slaves);
      Serial.println(" slaves");

      for (uint8_t i = 0; i < bp.num_slaves; ++ i) {
          printHex(bp.slave_ids[i], sizeof(bp.slave_ids[0]));
          Serial.println();
          //uint8_t buf[64];
          //bp.readEeprom(i + 1, 0, buf, sizeof(buf));
          //Serial.print("EEPROM: ");
          //printHex(buf, sizeof(buf));
          //Serial.println();
      }
  } else {
      bp.printLastError(Serial);
      Serial.println();
  }
  
  return;
}

// Example code to dump backpack EEPROM contents:
void printHex(const uint8_t *buf, uint8_t len) {
    while (len--) {
        if (*buf < 0x10) Serial.print("0");
        Serial.print(*buf++, HEX);
    }
}

void waitForResponse() {
  if (Serial1.available()) {
    while (Serial1.available()) {
      int inByte = Serial1.read();
      Serial.write(inByte);
      buffer[ctr++] = inByte;
      if (inByte == '\n' || inByte == '\r' || inByte == 0) {
        if (ctr > 1) {
          wifiResultReady = true;
        }
        buffer[ctr--] = 0;
        ctr = 0;
      }
    }
  }
}

bool putWifiToSleep() {
  Serial1.begin(9600);
  
  putWifiInRunMode();
  Serial.println("- Putting Gainspan into deep sleep");
 
  uint32_t start = millis();
  bool timedOut = false;
  
  Serial1.println("AT");
  
  while (!wifiResultReady) {
    if (millis() - start > 5000) {
      timedOut = true;
      Serial.println("FAIL: Timeout while waiting for response");
      return false;
    }
    waitForResponse();
  }
  wifiResultReady = false;

  if (strncmp((const char*)buffer, "AT", 2) != 0) {
    Serial.println("FAIL: Wi-Fi not responding");
    return false;
  }
  
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  if (strncmp((const char*)buffer, "OK", 2) == 0) {
    Serial.println("--- Wi-Fi module responded to AT");
  } else {
    Serial.println("FAIL: AT didn't return OK");
    return false;
  }
  
  Serial1.println("AT+PSDPSLEEP");
  delay(1000);
  return true;
}

void putWifiInProgramMode() {
  Serial.println("- Putting Gainspan into program mode");
  resetSPIChipSelectPins();
  
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(500);
  
  pinMode(WIFI_PROGRAM_SELECT, OUTPUT);
  digitalWrite(WIFI_PROGRAM_SELECT, HIGH);
  
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000); 
}

void putWifiInRunMode() {
  Serial.println("- Putting Gainspan into run mode");
  resetSPIChipSelectPins();
  
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(500);
  
  pinMode(WIFI_PROGRAM_SELECT, OUTPUT);
  digitalWrite(WIFI_PROGRAM_SELECT, LOW);
 
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
}

void resetSPIChipSelectPins() {
  SPI.end();
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

