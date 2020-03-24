/**
 * Example detect tags and show their unique ID 
 * Authors: 
 *        Salvador Mendoza - @Netxing - salmg.net
 *        For Electronic Cats - electroniccats.com
 * 
 *  March 2020
 * 
 * This code is beerware; if you see me (or any other collaborator 
 * member) at the local, and you've found our code helpful, 
 * please buy us a round!
 * Distributed as-is; no warranty is given.
 */

#include "Electroniccats_PN7150.h"
#define PN7150_IRQ   (8)
#define PN7150_VEN   (7)
#define PN7150_ADDR  (0x28)

#if !defined (CARDEMU_SUPPORT)
  #error "Don't forget to define CARDEMU_SUPPORT" in the library in Electroniccats_PN7150.h !
#endif

unsigned char OK[] = {0x90, 0x00}, Cmd[256], CmdSize;

Electroniccats_PN7150 nfc(PN7150_IRQ, PN7150_VEN, PN7150_ADDR); // creates a global NFC device interface object, attached to pins 7 (IRQ) and 8 (VEN) and using the default I2C address 0x28
void setup(){
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Detect NFC readers with PN7150");
  uint8_t statusNFC = setupNFC();
  if (!statusNFC) 
    Serial.println("Set up is ok");
  else
    Serial.println("Error while setting up mode, check connections!");
}

int setupNFC(){
  Serial.println("Initializing...");
  int setupOK = nfc.connectNCI();
  if (!setupOK){
    setupOK = nfc.ConfigMode();
    if (!setupOK) setupOK = nfc.StartDiscovery();
  }
  return setupOK;
}

void loop(){
  if(nfc.CardModeReceive(Cmd, &CmdSize) == 0) { //Data in buffer?
      if ((CmdSize >= 2) && (Cmd[0] == 0x00)) { //Expect at least two bytes
          switch (Cmd[1]) {
              case 0xA4: //Something tries to select a file, meaning that it is a reader
                  Serial.println("Reader detected!");
                  break;
  
              case 0xB0: //SFI
                  break;
  
              case 0xD0: //...
                  break;
  
              default:
                  break;
          }
          nfc.CardModeSend(OK, sizeof(OK));
      }
  }
}
