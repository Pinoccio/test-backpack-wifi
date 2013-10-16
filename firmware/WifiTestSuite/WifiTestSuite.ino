//#include <serialGLCDlib.h>
//#include <LeadScout.h>

//serialGLCD lcd;
const int buttonPin = A3;
bool testIsRunning = false;
bool wifiFlashUpgradeRunning = false;

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
  
  // disable Wi-Fi SPI 
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  putWifiInRunMode();
  Serial1.begin(115200);
  
  Serial.println("Wi-Fi Test Jig ready to go!");
  Serial.println("");
  
//  delay(5000);
//  Serial.println("hi");
//  Serial.write(0x7C);
//  Serial.write(0x02);
//  Serial.write(0x02);
}

void testJigLoop() {
  if (digitalRead(buttonPin) == HIGH) {     
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
  } else {
    //RgbLed.blinkGreen();
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
  
  /*
  testSerialFlash();
  
  testPower();
  
  flashBackpackBus();
  testBackpackBus();
  */
  testIsRunning = false;
}

void configureWifi() {
  Serial.println("- Configure Wi-Fi -");

  Serial1.println("AT");
  
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  if (strncmp((const char*)buffer, "AT", 2) != 0) {
    Serial.println("FAIL: Wi-Fi not responding");
    Serial1.begin(9600);
    Serial1.println("ATB=115200");
    Serial1.begin(115200);
  }
  
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  if (strncmp((const char*)buffer, "OK", 2) == 0) {
    Serial.println("--- AT passed");
  } else {
    Serial.println("FAIL: AT didn't return OK");
  }
  
  return;
}

void testWifi() {
  Serial.println("- Test Wi-Fi -");
  
  Serial1.println("AT+VER=?");
  
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  if (strncmp((const char*)buffer, "AT+VER=?", 8) != 0) {
    Serial.println("FAIL: Wi-Fi not responding");
  }
  
  while (!wifiResultReady) {
    waitForResponse();
  }
  wifiResultReady = false;
  
  if (strncmp((const char*)buffer, "S2W APP VERSION=2.4.3", 21) == 0) {
    Serial.println("--- Latest firmware (2.4.3) is loaded ");
  } else {
    Serial.println("FAIL: Old firmware found, entering Wi-Fi upgrade mode");
    flashWifi();
  } 
}

void flashWifi() {
  Serial1.begin(115200);
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
  
  return;
}

void testPower() {
  Serial.println("- Test Power -");
  
  return;
}

void flashBackpackBus() {
  Serial.println("- Flash Backpack Bus -");
  Serial1.println("AT+VER=?");
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

void putWifiInProgramMode() {
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(1000);
  
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
 
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000); 
}

void putWifiInRunMode() {
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(1000);
  
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
 
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000); 
}
