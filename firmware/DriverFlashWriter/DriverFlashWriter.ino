#include <LeadScout.h>

//FlashClass DriverFlash(5, SPI);

void setup() {
  Serial.begin(115200);
   
  // uncomment these if you want to put the WiFi module into firmware update mode
  //pinMode(6, OUTPUT);
  //digitalWrite(6, HIGH);
  
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, LOW);
  delay(1000);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
  
  uint32_t start = millis();
  
  while (!Flash.available()) {
    if (millis() - start > 30) {
      Serial.println("FAIL: Serial flash chip not found");
      return;
    }
  }
  Serial.println("--- Serial flash chip found");
  
  char dataToChip[128] = "Tester123456789";
  char dataFromChip[128] = "               ";
  
  for (uint32_t i=0; i<16777216; i+=16) {
    if (i % 1024 == 0) {
      Serial.print("Testing address: ");
      Serial.println(i, HEX);
    }
    
    memset(dataFromChip, ' ', 16);
    dataFromChip[15] = 0;
    
    // Write some data to RAM
    Flash.write(i, dataToChip, 16);
    //delay(100);
  
    // Read it back to a different buffer
    Flash.read(i, dataFromChip, 16);
    
    // Write it to the serial port
    if (strcmp((const char*)dataToChip, (const char*)dataFromChip) != 0) {
      Serial.print("FAIL: Data failed to write to and read from flash at address: ");
      Serial.println(i);
      Serial.println(dataToChip);
      Serial.println(dataFromChip);
    }
    
    Flash.sectorErase(i);  
  }
  
  Serial.println("Driver flash chip checks out fine");
}

void loop() { }
