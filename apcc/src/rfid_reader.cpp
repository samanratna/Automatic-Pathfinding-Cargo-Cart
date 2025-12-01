#include <Arduino.h>

#include <SPI.h>
#include "MFRC522.h"

#define RST_PIN 6
#define SS_PIN 53

MFRC522 mfrc522(SS_PIN, RST_PIN);

// Store your known UID tags here
byte tag1[] = {0x21, 0xE5, 0xD5, 0x5};   // Flag 1
byte tag2[] = {0x13, 0x81, 0xD5, 0x05};   // Flag 2
byte tag3[] = {0x37, 0xC2, 0xD4, 0x05};   // Flag 3

// int userFlag = 0;   // 0 = no match, 1 = tag1, 2 = tag2, ...

bool compareUID(byte *uid, byte *knownUID, byte size) {
  for (byte i = 0; i < size; i++) {
    if (uid[i] != knownUID[i]) return false;
  }
  return true;
}

void rfid_setup() {
//   Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println("RFID Ready...");
}

uint8_t getRFIDTagValue() {

  if (!mfrc522.PICC_IsNewCardPresent()) return;
  if (!mfrc522.PICC_ReadCardSerial()) return;

  // Print UID
  Serial.print("USER ID tag : ");
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    Serial.print(i < mfrc522.uid.size - 1 ? " " : "");
  }
  Serial.println();

  // Reset flag
  uint8_t userFlag = 0;

  // Check card 1
  if (compareUID(mfrc522.uid.uidByte, tag1, 4)) {
    Serial.println("TAG 1 recognized.");
    userFlag = 1;
  }
  // Check card 2
  else if (compareUID(mfrc522.uid.uidByte, tag2, 4)) {
    Serial.println("TAG 2 recognized.");
    userFlag = 2;
  }
  // Check card 3
  else if (compareUID(mfrc522.uid.uidByte, tag3, 4)) {
    Serial.println("TAG 3 recognized.");
    userFlag = 3;
  }

  // Show result
  Serial.print("FLAG = ");
  Serial.println(userFlag);

  // Stop reading
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  return userFlag;

//   delay(300);
}
