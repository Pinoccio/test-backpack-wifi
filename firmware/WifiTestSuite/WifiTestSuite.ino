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

FlashClass DriverFlash(DRIVER_FLASH_CS, SPI);

uint32_t S2W_APP1_IMG_ADDR = 0x10000;
uint32_t S2W_APP2_IMG_ADDR = 0x50000;
uint32_t WFW_REL_IMG_ADDR = 0xA0000;
uint32_t WIFI_EXTERNAL_FLASH_IMG_ADDR = 0x100000;

PBBP bp;
  
void setup() {
  Serial.begin(115200);
  //LeadScout.disableShell();
  //LeadScout.setup();
  testJigSetup();
}

void loop() {
  //LeadScout.loop();
  testJigLoop();
}

void testJigSetup() {
  pinMode(buttonPin, INPUT_PULLUP);
 
  resetSPIChipSelectPins(); 
  putWifiInRunMode();
  
  Serial.println("Wi-Fi Test Jig ready to go!");
}

void resetSPIChipSelectPins() {
  pinMode(GS_FLASH_CS, OUTPUT);
  digitalWrite(GS_FLASH_CS, HIGH);
  pinMode(TINY_13_RESET, OUTPUT);
  digitalWrite(TINY_13_RESET, HIGH);
  pinMode(DRIVER_FLASH_CS, OUTPUT);
  digitalWrite(DRIVER_FLASH_CS, HIGH);
  pinMode(WIFI_CS, OUTPUT);
  digitalWrite(WIFI_CS, HIGH);
  pinMode(MICRO_SD_CS, OUTPUT);
  digitalWrite(MICRO_SD_CS, HIGH);
  pinMode(BP_FLASH_CS, OUTPUT);
  digitalWrite(BP_FLASH_CS, HIGH);
}

void testJigLoop() {
  if (digitalRead(buttonPin) == HIGH) { 
    /*    
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
    */
  } else {
    RgbLed.blinkGreen();
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
  writeWifiFlash();
  // updateWifiFirmwareWFW();
  // updateWifiFirmwareApp1();
  // updateWifiFirmwareApp2();
  // writeWifiMACAddress();
  
  //configureWifi();
  //testWifi();
  
//  testSerialFlash();
  
//  flashBackpackBus();
//  testBackpackBus();
  
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
  putWifiInProgramMode();
  byte buffer[256] = {0};
                    
  FlashClass GSFlash(GS_FLASH_CS, SPI);
  
  uint32_t start = millis();
  while (!GSFlash.available()) {
    if (millis() - start > 1000) {
      Serial.println("FAIL: Serial flash chip not found");
      return;
    }
  }
  Serial.println("--- Serial flash chip found");
  
  GSFlash.bulkErase();
  
  for (uint32_t i = 0; i<0x100000; i+=256) {
    DriverFlash.begin(DRIVER_FLASH_CS, SPI);
    GSFlash.read(0xF0000, (byte *)buffer, 0x54);
    
  }
  
  if (strncmp((const char*)buffer, header1, 0x100) != 0) {
    Serial.println("FAIL: Header1 failed to write correctly");
    printHex((const uint8_t *)header1, 0x114);
    Serial.println();
    printHex((const uint8_t *)buffer, 0x114);
    Serial.println();
    if (strncmp((const char*)buffer, header2, 0x14) != 0) {
      Serial.println("FAIL: Header1 failed to write correctly");
      printHex((const uint8_t *)header2, 0x14);
      Serial.println();
      printHex((const uint8_t *)(buffer+0x100), 0x14);
      Serial.println();
    }
  } else {
    Serial.println("Success");
  }
  memset(buffer, 0, 0x114);
  
  Serial.print("- Writing footer: ");
  GSFlash.sectorErase(0xF0000);
  GSFlash.write(0xF0000, (byte *)footer, 0x54); 
  GSFlash.read(0xF0000, (byte *)buffer, 0x54);
  if (strncmp((const char*)buffer, footer, 0x54) != 0) {
    Serial.println("FAIL: Footer failed to write correctly");
    printHex((const uint8_t *)footer, 0x54);
    Serial.println();
    printHex((const uint8_t *)buffer, 0x54);
    Serial.println();
  } else {
    Serial.println("Success");
  }
  memset(buffer, 0, 0x54);
}

void configureWifi() {
  Serial.println("- Configure Wi-Fi -");
  resetSPIChipSelectPins();
  
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
  resetSPIChipSelectPins();
  
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
  resetSPIChipSelectPins();
  
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
  Serial.println("- Test Serial Flash -");
  resetSPIChipSelectPins();
  digitalWrite(VCC_ENABLE, LOW);
  delay(500);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(500);
  FlashClass Flash(BP_FLASH_CS, SPI);
  
  const int addr = 0x10000;
  
  uint32_t start = millis();
  
  bool flashFound = false;
  while (!Flash.available()) {
    if (millis() - start > 3000) {
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

  // Write some data to RAM
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
  Serial.println("- Flash Backpack Bus -");
  resetSPIChipSelectPins();
  
  digitalWrite(TINY_13_RESET, LOW);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(500);
  
  AVRProgrammer pgm = AVRProgrammer(TINY_13_RESET, SPI, SPI_CLOCK_DIV128);
  pgm.startProgramming();
  pgm.getSignature();
  pgm.getFuseBytes();
  
  // if we found a signature, try to write flash
  if (pgm.foundSignature() != -1) {
    Serial.println("-- erasing chip");
    pgm.eraseChip();
    Serial.println("-- writing fuses");
    pgm.writeFuseBytes(0x21, 0xFB, 0xFF);
    Serial.println("-- writing flash");
    pgm.writeProgram(0x0000, attiny13a_flash, sizeof(attiny13a_flash));
  } else {
    testFailed = true;
  }

  Serial.println("-- complete");
  pgm.end();
  return;
}

void testBackpackBus() {
  Serial.println("- Test Backpack Bus -");
  resetSPIChipSelectPins();
  
  if (bp.enumerate()) {
      Serial.print("Found ");
      Serial.print(bp.num_slaves);
      Serial.println(" slaves");

      for (uint8_t i = 0; i < bp.num_slaves; ++ i) {
          printHex(bp.slave_ids[i], sizeof(bp.slave_ids[0]));
          Serial.println();
          uint8_t buf[64];
          bp.readEeprom(i + 1, 0, buf, sizeof(buf));
          Serial.print("EEPROM: ");
          printHex(buf, sizeof(buf));
          Serial.println();
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
      //Serial.write(inByte);
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

void putWifiInProgramMode() {
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(500);
  
  pinMode(WIFI_PROGRAM_SELECT, OUTPUT);
  digitalWrite(WIFI_PROGRAM_SELECT, HIGH);
 
  digitalWrite(VCC_ENABLE, HIGH);
  delay(500); 
}

void putWifiInRunMode() {
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(500);
  
  pinMode(WIFI_PROGRAM_SELECT, OUTPUT);
  digitalWrite(WIFI_PROGRAM_SELECT, LOW);
 
  digitalWrite(VCC_ENABLE, HIGH);
  delay(50);
}
