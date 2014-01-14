#include <SPI.h>
#include <Wire.h>
#include <Scout.h>

int TINY13A_PIN = 3;
int GS_FLASH_PIN = 4;
int DRIVER_FLASH_PIN = 5;
int GS_PROG_MODE_PIN = 6;
int GS_MODULE_PIN = 7;
int MICROSD_PIN = 8;
int BP_FLASH_PIN = SS;

FlashClass DriverFlash(DRIVER_FLASH_PIN, SPI);

uint32_t S2W_APP1_IMG_ADDR = 0x10000;  // 127,402 bytes
uint32_t S2W_APP2_IMG_ADDR = 0x50000;  // 102,213 bytes
uint32_t WFW_REL_IMG_ADDR = 0xA0000;   // 131,044 bytes
uint32_t WIFI_EXTERNAL_FLASH_IMG_ADDR = 0x100000; // 1,048,576 bytes


uint32_t addr = WFW_REL_IMG_ADDR;


#define BUFSIZE 32
int ctr = 0;

byte dataRead[BUFSIZE];
byte dataWritten[BUFSIZE];

void setup() {
  Serial.begin(115200);
  Serial.println("Starting up...");
  
  pinMode(GS_PROG_MODE_PIN, OUTPUT);
  digitalWrite(GS_PROG_MODE_PIN, HIGH);
  
  pinMode(TINY13A_PIN, OUTPUT);
  digitalWrite(TINY13A_PIN, HIGH);
  pinMode(GS_FLASH_PIN, OUTPUT);
  digitalWrite(GS_FLASH_PIN, HIGH);
  pinMode(DRIVER_FLASH_PIN, OUTPUT);
  digitalWrite(DRIVER_FLASH_PIN, HIGH); 
  pinMode(GS_MODULE_PIN, OUTPUT);
  digitalWrite(GS_MODULE_PIN, HIGH);
  pinMode(MICROSD_PIN, OUTPUT);
  digitalWrite(MICROSD_PIN, HIGH);
  pinMode(BP_FLASH_PIN, OUTPUT);
  digitalWrite(BP_FLASH_PIN, HIGH);
  
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(500);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(500);
  
  uint32_t start = millis();
  
  DriverFlash.begin(DRIVER_FLASH_PIN, SPI);
  while (!DriverFlash.available()) {
    if (millis() - start > 1000) {
      Serial.println("FAIL: Serial flash chip not found");
      return;
    }
  }
  Serial.println("--- Serial flash chip found");
  
//  Serial.println("--- Erasing chip");
//  DriverFlash.bulkErase();
//  Serial.println("--- Erasing sector 0x10000");
//  DriverFlash.sectorErase(S2W_APP1_IMG_ADDR); 
//  Serial.println("--- Erasing sector 0x20000");
//  DriverFlash.sectorErase(S2W_APP1_IMG_ADDR+0x10000); 
//  Serial.println("--- Erasing sector 0x50000");
//  DriverFlash.sectorErase(S2W_APP2_IMG_ADDR); 
//  Serial.println("--- Erasing sector 0x60000");
//  DriverFlash.sectorErase(S2W_APP2_IMG_ADDR+0x10000); 
//  Serial.println("--- Erasing sector 0xA0000");
//  DriverFlash.sectorErase(WIFI_EXTERNAL_FLASH_IMG_ADDR); 
//  Serial.println("--- Erasing sector 0xB0000");
//  DriverFlash.sectorErase(WIFI_EXTERNAL_FLASH_IMG_ADDR+0x10000); 
  Serial.print("--- Ready for hex bytes at address: 0x");
  Serial.println(addr, HEX);
}

void loop() { 
  while (Serial.available()) {
//    Serial.print(Serial.read());
//    Serial.print(Serial.peek());
//    Serial.print(":");
    dataRead[ctr++] = (byte)Serial.read();
//    Serial.println(dataRead[ctr-1], DEC);
    
    if (ctr == BUFSIZE) {
      dataRead[BUFSIZE] = 0;

      DriverFlash.write(addr, &dataRead, BUFSIZE);
      DriverFlash.read(addr, &dataWritten, BUFSIZE);
     
      if (strncmp((const char*)dataRead, (const char*)dataWritten, BUFSIZE) != 0) {
        Serial.print("FAIL: Data failed to write to and read from flash at address: ");
        Serial.println(addr, HEX);
        
        dataRead[ctr] = 0;
        dataWritten[ctr] = 0;
      
        Serial.print("dataRead: ");
        for (int i=0; i<BUFSIZE; i++) {
          Serial.print(dataRead[i], HEX);
        }
        Serial.println();
        Serial.print("dataWritten: ");
        for (int i=0; i<BUFSIZE; i++) {
          Serial.print(dataWritten[i], HEX);
        }
        Serial.println();
      } else {
        Serial.println("OK");
      }
     
      ctr = 0;
      addr += BUFSIZE;
    } 
  }
}
