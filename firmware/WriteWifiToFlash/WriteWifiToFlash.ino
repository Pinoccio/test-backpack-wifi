#include <LeadScout.h>
#include <SPI.h>
#include <Wire.h>

#define TINY_13_CS 3
#define GS_FLASH_CS 4
#define DRIVER_FLASH_CS 5
#define WIFI_PROGRAM_SELECT 6
#define WIFI_CS 7
#define MICRO_SD_CS 8

#define AVR_TESTSUITE_DEBUG
#ifdef AVR_TESTSUITE_DEBUG
#  define TD(x) x
#else
#  define TD(x)
#endif

FlashClass DriverFlash(DRIVER_FLASH_CS, SPI);
uint32_t start = millis();

uint32_t addr = 0x40000;
int ctr = 0;
byte dataRead[64];
byte dataWritten[64];

#define BUFSIZE 32

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  TD(Serial.println("Starting up..."));
  
  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);
  
  // put wifi in program mode
  TD(Serial.println("--- Putting Wi-Fi module in program mode"));
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(1000);
  pinMode(WIFI_PROGRAM_SELECT, OUTPUT);
  digitalWrite(WIFI_PROGRAM_SELECT, HIGH);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000); 
  
  // disable all SPI chip selects 
  pinMode(GS_FLASH_CS, OUTPUT);
  digitalWrite(GS_FLASH_CS, HIGH);
  pinMode(TINY_13_CS, OUTPUT);
  digitalWrite(TINY_13_CS, HIGH);
  pinMode(DRIVER_FLASH_CS, OUTPUT);
  digitalWrite(DRIVER_FLASH_CS, HIGH);
  pinMode(WIFI_CS, OUTPUT);
  digitalWrite(WIFI_CS, HIGH);
  pinMode(MICRO_SD_CS, OUTPUT);
  digitalWrite(MICRO_SD_CS, HIGH);
  
  while (!DriverFlash.available()) {
    if (millis() - start > 1000) {
      TD(Serial.println("FAIL: Serial flash chip not found"));
      return;
    }
  }
  TD(Serial.println("--- Serial flash chip found"));
  
  // DriverFlash.bulkErase(); 
  // Store flash hex in sector 4 (0x40000 - 0x4F000, 61,440 bytes)
  TD(Serial.println("--- Erasing sector 0x40000"));
  DriverFlash.sectorErase(addr); 
  TD(Serial.println("--- Ready for hex bytes"));
}

void loop() {
  
  // read from port 1, send to port 0:
  while (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte); 
  }
  
  while (Serial.available()) {
//    Serial.print(Serial.read());
//    Serial.print(Serial.peek());
//    Serial.print(":");
    dataRead[ctr] = (byte)Serial.read();
    Serial1.write(dataRead[ctr]); 
    ctr++;
//    Serial.println(dataRead[ctr-1], DEC);
    
    if (ctr == BUFSIZE) {
      dataRead[BUFSIZE] = 0;

      DriverFlash.write(addr, &dataRead, BUFSIZE);
      DriverFlash.read(addr, &dataWritten, BUFSIZE);
      digitalWrite(23, !digitalRead(23));
      
      if (strncmp((const char*)dataRead, (const char*)dataWritten, BUFSIZE) != 0) {
        digitalWrite(22, LOW);
        TD(Serial.print("FAIL: Data failed to write to and read from flash at address: "));
        TD(Serial1.println(addr, HEX));
        
        dataRead[ctr] = 0;
        dataWritten[ctr] = 0;
      
        TD(Serial.print("dataRead: "));
        for (int i=0; i<BUFSIZE; i++) {
          TD(Serial.print(dataRead[i], HEX));
        }
        TD(Serial.println());
        TD(Serial.print("dataWritten: "));
        for (int i=0; i<BUFSIZE; i++) {
          TD(Serial.print(dataWritten[i], HEX));
        }
        TD(Serial.println());
      } else {
        TD(Serial.println("OK"));
      }
     
      ctr = 0;
      addr += BUFSIZE;
    } 
  }
}

