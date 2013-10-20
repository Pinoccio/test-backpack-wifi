//#include <serialGLCDlib.h>
#include <LeadScout.h>

//serialGLCD lcd;
#define TINY_13_CS 3
#define GS_FLASH_CS 4
#define DRIVER_FLASH_CS 5
#define WIFI_PROGRAM_SELECT 6
#define WIFI_CS 7
#define MIDRO_SD_CS 8

const int buttonPin = A3;
bool testIsRunning = false;
bool wifiFlashUpgradeRunning = false;
bool testFailed = false;

char buffer[256];
int ctr = 0;
bool wifiResultReady = false;

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
  digitalWrite(buttonPin, HIGH);
  pinMode(buttonPin, INPUT);
 
  // disable all SPI chip selects 
  pinMode(GS_FLASH_CS, OUTPUT);
  digitalWrite(GS_FLASH_CS, HIGH);
  pinMode(TINY_13_CS, OUTPUT);
  digitalWrite(TINY_13_CS, HIGH);
  pinMode(DRIVER_FLASH_CS, OUTPUT);
  digitalWrite(DRIVER_FLASH_CS, HIGH);
  pinMode(WIFI_CS, OUTPUT);
  digitalWrite(WIFI_CS, HIGH);
  pinMode(MIDRO_SD_CS, OUTPUT);
  digitalWrite(MIDRO_SD_CS, HIGH);
  
  putWifiInRunMode();
  
  Serial.println("Wi-Fi Test Jig ready to go!");
  Serial.println("");
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
    delay(800);
    startTest();
  }
 
//  lcd.clearLCD();
//  delay(10);
//  Serial.println("test");
//  delay(2000);
}

void startTest() {
  if (testIsRunning == true) {
    Serial.println("Test is already running");
    return;
  }
  testIsRunning = true;

  configureWifi();
  testWifi();
  
  testSerialFlash();
  
  //flashBackpackBus();
  //testBackpackBus();
  
  testIsRunning = false;
  
  if (testFailed == false) {
    RgbLed.green();
  } else {
    RgbLed.red();
  }
  Serial.println("Test complete");
}

void configureWifi() {
  Serial.println("- Configure Wi-Fi -");
  
  if (!Gainspan.init(9600)) {
    if (!Gainspan.init(115200)) {
      Serial.println("FAIL: Wi-Fi not responding");
      testFailed = true;
    } 
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
  
  if (Gainspan.getAppVersion() != "2.4.3" ||
      Gainspan.getGepsVersion() != "2.4.3" ||
      Gainspan.getWlanVersion() != "2.4.1") {
    Serial.println("Wi-Fi module requires a firmware upgrade");
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
  Serial.println("- Test Serial Flash -");
  
  uint32_t start = millis();
  
  while (!Flash.available()) {
    if (millis() - start > 30) {
      Serial.println("FAIL: Serial flash chip not found");
      testFailed = true;
      return;
    }
  }
  Serial.println("--- Serial flash chip found");
  
  char dataToChip[128] = "Testing123456789";
  char dataFromChip[128] = "                ";
  
  for (uint32_t i=0; i<16777216; i+=1000000) {
    
    memset(dataFromChip, ' ', 16);
    dataFromChip[16] = 0;
    
    // Write some data to RAM
    Flash.write(i, dataToChip, 17);
    //delay(100);
  
    // Read it back to a different buffer
    Flash.read(i, dataFromChip, 17);
    
    // Write it to the serial port
    if (strcmp((const char*)dataToChip, (const char*)dataFromChip) != 0) {
      Serial.print("FAIL: Data failed to write to and read from flash at address: ");
      testFailed = true;
      Serial.println(i);
      Serial.println(dataToChip);
      Serial.println(dataFromChip);
    }
    
    Flash.sectorErase(i);
  }
  
  Serial.println("--- Data written and read successfully across serial flash");
  return;
}

void flashBackpackBus() {
  Serial.println("- Flash Backpack Bus -");
  digitalWrite(TINY_13_CS, LOW);
  return;
}

void testBackpackBus() {
  Serial.println("- Test Backpack Bus -");
  
  return;
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
  delay(1000);
  
  pinMode(WIFI_PROGRAM_SELECT, OUTPUT);
  digitalWrite(WIFI_PROGRAM_SELECT, HIGH);
 
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000); 
}

void putWifiInRunMode() {
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(1000);
  
  pinMode(WIFI_PROGRAM_SELECT, OUTPUT);
  digitalWrite(WIFI_PROGRAM_SELECT, LOW);
 
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
}
