#include <GS.h>
#include <serialGLCDlib.h>
#include <SPI.h>
#include <Wire.h>
#include <PBBP.h>
#include <crc.h>
#include <Scout.h>
#include <utility/WiFiBackpack.h>

#include "programmer.h"

GSModule gs;

serialGLCD lcd;
#define TINY_13_RESET 3
#define GS_FLASH_CS 4
#define DRIVER_FLASH_CS 5
#define WIFI_PROGRAM_SELECT 6
#define WIFI_CS 7
#define MICRO_SD_CS 8
#define BP_FLASH_CS SS

#define AVR_TESTSUITE_DEBUG
#ifdef AVR_TESTSUITE_DEBUG
#  define TD(x) x
#else
#  define TD(x)
#endif
FlashClass DriverFlash(DRIVER_FLASH_CS, SPI);

const int buttonPin = A3;
bool testIsRunning = false;
bool wifiFlashUpgradeRunning = false;
bool testFailed = false;

char buffer[256];
int ctr = 0;
bool wifiResultReady = false;

char macAddr[6];

const uint32_t HW_SERIAL_ADDR = 0x1000;
const uint32_t S2W_APP1_IMG_ADDR = 0x10000;
const uint32_t S2W_APP2_IMG_ADDR = 0x50000;
const uint32_t WFW_REL_IMG_ADDR = 0xA0000;
const uint32_t WIFI_EXTERNAL_FLASH_IMG_ADDR = 0x100000;

/* IDs < 1024 reserved for internal testing/use */
const bool RESET_HW_SERIAL = false;
const uint32_t HW_SERIAL_INIT = 0x400;    // 1024

uint32_t hwSerial;


PBBP bp;
  
void setup() {
  Serial.begin(115200);
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  
  getSettingsFromFlash();
  testJigSetup();
  RgbLed.cyan();
}

void loop() {
  testJigLoop();
}

void getSettingsFromFlash() {
  TD(Serial.println("-- Setting up unique ID handler"));
  resetSPIChipSelectPins();
  
  if (RESET_HW_SERIAL == true) {
    TD(Serial.print("--- Initializing unique ID to: 0x"));
    TD(Serial.println(HW_SERIAL_INIT, HEX));
    writeHwSerialToFlash(HW_SERIAL_INIT);
  }
  
  TD(Serial.print("-- Reading HW unique ID from flash: 0x"));
  hwSerial = readHwSerialFromFlash();
  TD(Serial.println(hwSerial, HEX));
  TD(Serial.println("--- Done"));
}

void testJigSetup() {
  pinMode(buttonPin, INPUT_PULLUP);
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
/*
  putWifiInProgramMode();
  getWifiMACAddress();
  getWifiSettings();
  eraseWifiModule();
  updateWifiFirmwareWFW();
  writeWifiFlash();
  updateWifiFirmwareApp1();
  updateWifiFirmwareApp2();
  writeWifiSettings();
  writeWifiMACAddress();
  putWifiInRunMode();
*/  
  putWifiInRunMode();
  testWifi();

  putWifiInProgramMode();  
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
  Serial.println();
  testJigSetup();
}

void testWifi() {
  Serial.println("- Test Wi-Fi Module -");
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.begin();
  gs.begin(WIFI_CS);
  
  Serial.print("-- Getting Version: ");
  gs.writeCommand("AT+VER=?");
  gs.readResponse(print_line, &Serial);
  if (strncmp(buffer, "S2W WLAN VERSION=2.5.1", 22) != 0) {
    testFailed = true;
    Serial.println();
    Serial.print("FAIL: Unexpected response from AT+VER=? command: ");
    Serial.println(buffer);
    return;
  } else {
    Serial.println("Success");
  }
  
  Serial.print("-- Scanning for networks: ");
  gs.writeCommand("AT+WS");
  gs.readResponse(print_line, &Serial);
  if (strncmp(buffer, "No.Of AP Found:", 15) != 0) {
    testFailed = true;
    Serial.println();
    Serial.print("FAIL: Unexpected response from AT+WS command: ");
    Serial.println(buffer);
    return;
  } else {
    Serial.println("Success");
  }
}

void testSerialFlash() {
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

  Serial.println("-- Serial flash chip found");
 
  Serial.println("-- Erasing subsector");
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
    Serial.println("-- Data written to and successfully read from serial flash");
  }

  Serial.println("-- Erasing test data");
  Flash.subSectorErase(addr);
  Flash.end();
  return;
}

void flashBackpackBus() {
  Serial.println("- Flash Backpack Bus Program -");
  
  AVRProgrammer pgm = AVRProgrammer(TINY_13_RESET, SPI, SPI_CLOCK_DIV128);
  pgm.startProgramming();
  pgm.getSignature();
  pgm.getFuseBytes();
  
  // if we found a signature, try to write flash
  if (pgm.foundSignature() != -1) {
    
    Serial.println("-- Erasing chip");
    pgm.eraseChip();
    
    Serial.println("-- Writing fuses");
    if (!pgm.writeFuseBytes(0x21, 0xFB, 0xFF)) {
      testFailed = true;
      Serial.println("FAIL: writing fuses failed");
      pgm.end();
      return; 
    }
    
    Serial.println("-- Writing flash");
    if (!pgm.writeProgram(0x0000, attiny13a_flash, sizeof(attiny13a_flash))) {
      testFailed = true;
      Serial.println("FAIL: writing flash failed");
      pgm.end();
      return; 
    }
    
    Serial.println("-- Writing EEPROM");
   
    uint8_t eepromSize = sizeof(attiny13a_eeprom);
    uint8_t idCrc;
    uint16_t eepromCrc;
    
    byte eeprom[eepromSize];
    memcpy_P(eeprom, attiny13a_eeprom, eepromSize);
    
    byte val[4];
    convertLongToBytes(val, hwSerial);
    
    // set CRC and unique ID values in eeprom
    eeprom[7] = val[2];
    eeprom[8] = val[1];
    eeprom[9] = val[0];

    // size of ID section, byte offsets 3-9
    eeprom[0x0A] = pinoccio_crc_generate_byte(0x12F, idCrc, eeprom+3, 7);
    
    // size of eeprom contents - 2 bytes for final checksum
    eepromCrc = pinoccio_crc_generate_word(0x1A7D3, eepromCrc, eeprom, 0x39);
    eeprom[0x39] = (eepromCrc >> 8) & 0xFF; 
    eeprom[0x3A] = (eepromCrc >> 0) & 0xFF;
    
    for (int i=0; i<eepromSize; i++) {
      pgm.writeEeprom(i, eeprom[i]);
    }
  
    Serial.println("-- Reading EEPROM");
    for (int i=0; i<eepromSize; i++) {
      if (eeprom[i] != pgm.readEeprom(i)) {
        Serial.print("FAIL: EEPROM failed to write correctly at address: 0x");
        Serial.println(i, HEX);
        Serial.print("- Expected: ");
        Serial.println(eeprom[i], HEX);
        Serial.print("- Read: ");
        Serial.println(pgm.readEeprom(i), HEX);
        testFailed = true;
        return;
      } else {
        //printEepromAddress(i, pgm.readEeprom(i));
      }
    }
  } else {
    testFailed = true;
  }
  
  pgm.end();
  
  if (testFailed == false) {
    incrementHwSerial();
  }
  return;
}

void testBackpackBus() {
  Serial.println("- Test Backpack Bus -");
  digitalWrite(VCC_ENABLE, HIGH);
  delay(500);
  bp.begin(BACKPACK_BUS);
  delay(250);
  
  Serial.println("-- Enumerating backpack bus");
  if (bp.enumerate()) {
    if (bp.num_slaves != 1) {
      Serial.print("FAIL: Found ");
      Serial.print(bp.num_slaves);
      Serial.println(" slaves but expected 1 slave");
      testFailed = true;
      return;
    } else {
      Serial.println("-- Found one backpack");
    }

    if (bp.slave_ids[0][1] != 0x00 || bp.slave_ids[0][2] != 0x01) {
      Serial.print("FAIL: expected to see backpack model 0x0001 but received: 0x");
      printHex((byte *)bp.slave_ids[0][1], 1);
      printHex((byte *)bp.slave_ids[0][2], 1);
      Serial.println();
      testFailed = true;
      return;
    } else {
      Serial.print("-- And it's a Wi-Fi backpack with ID: 0x");
      Serial.print(bp.slave_ids[0][4], HEX);
      Serial.print(bp.slave_ids[0][5], HEX);
      Serial.print(bp.slave_ids[0][6], HEX);
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

void putWifiInProgramMode() {
  Serial.println("- Putting Gainspan into program mode");
  resetSPIChipSelectPins();
  
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(500);
  
  pinMode(WIFI_PROGRAM_SELECT, OUTPUT);
  digitalWrite(WIFI_PROGRAM_SELECT, HIGH);
  
  digitalWrite(VCC_ENABLE, HIGH);
  delay(500);
  
  Serial1.begin(115200);
  while(Serial1.read() != -1);
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
  delay(500);
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

void printEepromAddress(int i, byte val) {
  Serial.print("0x");
  Serial.print(i, HEX);
  Serial.print(": ");
  Serial.println(val, HEX); 
}

void send(byte *buf, int length) {
  bool debug = false;
  
  for (int i=0; i<length; i++) {
    Serial1.write(buf[i]);
  }
}

void receive(byte *buf, int length, int timeout, bool debug) {
  uint32_t time = millis();
  int i;
  
//  debug ? (Serial.print("length: ")) : false;
//  debug ? (Serial.println(length)) : false;
//  debug ? (Serial.print("timeout: ")) : false;
//  debug ? (Serial.println(timeout)) : false;
  
  memset(buf, 0, length);
  i = 0;
  
//  debug ? (Serial.print("millis: ")) : false;
//  debug ? (Serial.println(millis())) : false;
//  debug ? (Serial.print("time: ")) : false;
//  debug ? (Serial.println(time)) : false;
  
  while (millis() - time < timeout) {
    if (Serial1.available() > 0) {
      buf[i] = Serial1.read();
      if (debug) {
        AVRProgrammer::showHex(buf[i], false, true);
      }
      i++;
    }
    if (i >= length) {
      break;
    }
  }
  debug ? (Serial.println()) : false;
  
//  debug ? (Serial.print("millis: ")) : false;
//  debug ? (Serial.println(millis())) : false;
//  debug ? (Serial.print("time: ")) : false;
//  debug ? (Serial.println(time)) : false;
  
  // flush buffer
  while(Serial1.read() != -1);
}

static void print_line(const uint8_t *buf, uint16_t len, void *data) {
  //static_cast<Print*>(data)->write(buf, len);
  //static_cast<Print*>(data)->println();
  strncpy(buffer, (char *)buf, len);
}

void writeHwSerialToFlash(uint32_t hwSerial) {
  char dataWrite[5];
  char dataCheck[5];
  char dataRead[5];

  digitalWrite(VCC_ENABLE, HIGH); 
  DriverFlash.begin(DRIVER_FLASH_CS, SPI);
  
  TD(Serial1.println("--- Erasing subsector"));
  DriverFlash.subSectorErase(HW_SERIAL_ADDR);
  
  TD(Serial1.print("--- Writing HW unique ID: "));
  convertLongToBytes((byte *)dataWrite, hwSerial);
  TD(Serial1.println(convertBytesToLong((byte *)dataWrite), HEX));
 
  DriverFlash.write(HW_SERIAL_ADDR, &dataWrite, 4);
  TD(Serial1.print("--- Checking previous write: "));
  DriverFlash.read(HW_SERIAL_ADDR, &dataCheck, 4);
  
  TD(Serial1.println(convertBytesToLong((byte *)dataCheck), HEX));
  
  if (strncmp(dataWrite, dataCheck, 4) != 0) {
    TD(Serial1.println("FAIL: Writing serial failed"));
    TD(Serial1.println(atoi(dataWrite), HEX));
    TD(Serial1.println(atoi(dataCheck), HEX));
  } else {
    TD(Serial1.println("--- Write succeeded"));
  }
  DriverFlash.end();
}

uint32_t readHwSerialFromFlash() {
  digitalWrite(VCC_ENABLE, HIGH);
  char dataRead[5];
  DriverFlash.begin(DRIVER_FLASH_CS, SPI);
  DriverFlash.read(HW_SERIAL_ADDR, &dataRead, 4);
  DriverFlash.end();
  return convertBytesToLong((byte *)dataRead);
}

void convertLongToBytes(byte *convBytes, uint32_t target) {
  bool debug = false;
  if (debug) {
    TD(Serial1.print("convertLongToBytes using target: ")); 
    TD(Serial1.println(target, HEX)); 
  }
  
  for (int i=0; i<4; i++) {
    convBytes[i] = (target & 0xFF);
    target = target >> 8;
    if (debug) {
      TD(Serial1.print(convBytes[i], HEX)); 
    }
  }
  if (debug) {
    TD(Serial1.println()); 
  }
  convBytes[4] = 0;
}

uint32_t convertBytesToLong(byte *convBytes) {
  uint32_t target = 0;
  
  for (int i=3; i>=0; i--) {
    target |= convBytes[i];
    if (i > 0) {
      target = target << 8;
    }
  }
  return target;
}

void convertWordToBytes(byte *convBytes, uint16_t target) {
  for (int i=0; i<2; i++) {
    convBytes[i] = (target & 0xFF);
    target = target >> 8;
  }
  convBytes[2] = 0;
}

uint16_t convertBytesToWord(byte *convBytes) {
  uint16_t target = 0;
  
  for (int i=1; i>=0; i--) {
    target |= convBytes[i];
    if (i > 0) {
      target = target << 8;
    }
  }
  return target;
}

void incrementHwSerial() {
  TD(Serial1.println("-- Increment unique ID and store to flash"));
  
  hwSerial = readHwSerialFromFlash() + 1;
  writeHwSerialToFlash(hwSerial);
  TD(Serial1.println("--- Done"));
}


void getWifiMACAddress() {
  Serial.println("- Get Wi-Fi MAC address -");
  byte rx[128];  
  
  byte  tx[] = {0xA5, 0x07, 0x00, 0x00, 0x00, 0x06, 0x00, 0xF2, 0x01, 0x00, 0x10, 0x00, 0x05, 0x20};
  send(tx, sizeof(tx));
  receive(rx, 14, 5000, false);
  
  byte tx2[] = {0xA5, 0x07, 0x00, 0x00, 0x00, 0x0A, 0x00, 0xEE, 0x04, 0x00, 0xE8, 0x01, 0x08, 0x00, 0x04, 0x00, 0x00, 0x00};
  send(tx2, sizeof(tx2));
  receive(rx, 26, 5000, true);
  
  // if address is in old memory location, save and exit
  if (rx[12] == 0x20 && rx[13] == 0xF8) { 
    saveMacAddress(rx);
    testFailed = false;
    return;
  }
  
  byte tx3[] = {0xA5, 0x07, 0x00, 0x00, 0x00, 0x0A, 0x00, 0xEE, 0x04, 0x00, 0xF8, 0x01, 0x08, 0x00, 0x04, 0x00, 0x00, 0x00};
  send(tx3, sizeof(tx3));
  receive(rx, 26, 5000, true);
  
  // if address is in new memory location, save and exit
  if (rx[12] == 0x20 && rx[13] == 0xF8) { 
    saveMacAddress(rx);
    testFailed = false;
    return;
  }
  
  testFailed = true;
  return;
}

void saveMacAddress(byte *rx) {
  Serial.print("-- Found MAC address: ");
  for (int i=0; i<6; i++) {
    macAddr[i] = rx[12+i];    
    AVRProgrammer::showHex(macAddr[i], false, false, false);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();
}
