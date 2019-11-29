#include <JC_Button.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>
#include "Keypad.h"
#include "usb_midi.h"
#include "usb_midi_device.h"
#include "hardware_config.h"
#include "USBMidiKliK-Control-Display.h"
#include "klikpins.h"
#include "calc.h"
#include <libMMM.h>
#include <SoftSerial.h>

#define OLED_RESET -1
#define LED_PULSE_MILLIS  500
#define MIDIUSB_ENABLE__NOPE

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_SSD1306 display2(OLED_RESET);

struct menu {
  char name[12];
  char *members[20];
};

struct transformerInfo{
  uint8_t tByte[8];
};

const struct menu menus[] = {
  {"USBMidiKlik", {"Routes", "Transformers"} },
  {"Sysex Bank",  {"Load", "Save"} }
};

uint8_t ML1 = 2;
uint8_t ML2[2] = {2, 2};
uint8_t UI_Function_L1 = 0;
uint8_t UI_Function_L2 = 0;

enum rf  { ROUTE=0x1, FILTER=0x2, TRANSFORMER=0x3 };
enum cj  { CABLE=0x0, JACK=0x1 };
enum m1  { USBMIDIKLIK=0x0, SYSEXBANK=0x1 };
enum m21 { LOADS=0x0, SAVES=0x1 };
enum m22 { ROUTES=0x0, TRANSFORMERS=0x1 };
enum pic { BROWSE_FORWARD=0x1, BROWSE_BACKWARD=0x2, LOAD_FILE=0x3, REC_MODE=0x4, DISPLAY_CURRENT=0x5 };

char keyPress;
char dialBuffer[20];
uint8_t dialBufferPos = 0;
uint8_t VX = 0, VY = 0;
uint8_t currentFlowType = 0x0; //was byte
char serialMessageBuffer[64];
uint8_t serialMessageBufferIDX = 0;
uint8_t inByte; //was byte
uint8_t packetLen = 0;
uint8_t dialCableOrJack = CABLE;
char GLB_ScreenTitle1[20] = "";
char GLB_ScreenTitle2[20] = "";
uint8_t GLB_Port = 0;
uint8_t GLB_Filter = 0;
uint16_t GLB_BMT_Cable;
uint16_t GLB_BMT_Jack;
char GLB_Filter_XO[4];
uint8_t DISP_CableOrJack = CABLE;
uint8_t DISP_Port = 0;
uint8_t DISP_TransformerSlot = 0;
uint8_t pendingConfigPackets;

midiPacket_t pk { .i = 0 };
USBMidi MidiUSB;
HardwareSerial * serialHw[SERIAL_INTERFACE_MAX] = {SERIALS_PLIST};
transformerInfo GLB_Transformers[TRANSFORMERS_PR_CHANNEL];
SoftSerial AVRSerial(SOFTWARESERIAL_RX, SOFTWARESERIAL_TX, 1);

/* Display - Fix this horror !!! */

void displayWrite(Adafruit_SSD1306* disp, const char* txt, int x, int y, bool clr)
{
  if (clr) disp->clearDisplay();
  disp->setCursor(x, y);
  disp->print(txt);
  disp->display();
}

void displayWriteRaw(Adafruit_SSD1306* disp, char* txt, int x, int y, bool clr)
{
  if (clr) disp->clearDisplay();
  disp->setCursor(x, y);
  disp->print(txt);
  disp->display();
}

void displayWriteInt(Adafruit_SSD1306* disp, uint8_t txt, int x, int y, bool clr)
{
  if (clr) disp->clearDisplay();
  disp->setCursor(x, y);
  disp->print(txt);
  disp->display();
}

void displayBlank(Adafruit_SSD1306* disp)
{
   disp->clearDisplay();
   disp->display();  
}

/* MIDI / Sysex */

void SerialWritePacket(const midiPacket_t *pk, uint8_t serialNo)
{
  uint8_t msgLen = CINToLenTable[pk->packet[0] & 0x0F];
  serialHw[serialNo]->write(&pk->packet[1], msgLen);
}

void sysexForward(uint8_t dataByte) //was byte
{
  if (dataByte == 0xF7) {

    pk.packet[0] = 5 + packetLen;
    pk.packet[packetLen + 1] = dataByte;

    #ifdef MIDIUSB_ENABLE
        MidiUSB.writePacket(&pk.i);
    #endif

    SerialWritePacket(&(pk), 0);
    SerialWritePacket(&(pk), 1);
    SerialWritePacket(&(pk), 2);

    memset(&pk, 0, sizeof(midiPacket_t));
    memset(&packetLen, 0, sizeof(uint8_t));

  } else {

    pk.packet[++packetLen] = dataByte;

    if (packetLen == 3) {
      pk.packet[0] = 4;

      #ifdef MIDIUSB_ENABLE
          MidiUSB.writePacket(&pk.i);
      #endif

      SerialWritePacket(&(pk), 0);
      SerialWritePacket(&(pk), 1);
      SerialWritePacket(&(pk), 2);

      memset(&pk, 0, sizeof(midiPacket_t));
      memset(&packetLen, 0, sizeof(uint8_t));
    }

  }
}

void sysexSendToUSB(uint8_t buff[], uint16_t sz) 
{
  midiPacket_t pk { .i = 0 };
  uint8_t b=0;
  bool endPk;
  for ( uint16_t i = 0; i != sz ; i ++ ) {
    pk.packet[++b] = buff[i];
    endPk = ( i + 2 > sz );
    if (b == 3 || endPk ) {
        pk.packet[0] = endPk ? b + 4 : 4 ;
        MidiUSB.writePacket(&pk.i);
        b=0; pk.i = 0;
    }
  }
}

void requestPortRouteDump()
{
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
 
  uint8_t sysex_cableTargets[11] =  { 0xF0, 0x77, 0x77, 0x78, 0x5, 0x1, 0x1, DISP_CableOrJack, DISP_Port, 0x0, 0xF7 };
  uint8_t sysex_jackTargets[11] =   { 0xF0, 0x77, 0x77, 0x78, 0x5, 0x1, 0x1, DISP_CableOrJack, DISP_Port, 0x1, 0xF7 };
  uint8_t sysex_filterTargets[11] = { 0xF0, 0x77, 0x77, 0x78, 0x5, 0x1, 0x2, DISP_CableOrJack, DISP_Port, 0x0, 0xF7 };   
  
  pendingConfigPackets = 3;
  
  Serial2.write(sysex_cableTargets, 11);delay(100);Serial.flush();
  Serial2.write(sysex_jackTargets, 11);delay(100);Serial.flush();
  Serial2.write(sysex_filterTargets, 11);delay(100);Serial.flush();  
}

void requestPortTransformerDump()
{
  digitalWrite(LED_BLUE, HIGH);
  
  uint8_t sysex_tformers_slot[11] = { 0xF0, 0x77, 0x77, 0x78, 0x5, 0x1, 0x3, DISP_CableOrJack, DISP_Port, DISP_TransformerSlot, 0xF7 }; 

  pendingConfigPackets = 1;

  Serial2.write(sysex_tformers_slot, 11);delay(100);Serial.flush();  
}

void resetRouteDialBuffer()
{
  dialBufferPos = 0;
  memset(dialBuffer, 0, sizeof(dialBuffer));
}

uint8_t bufferDecCharToInt(uint8_t offset)
{
  uint8_t db10 = (uint8_t)(dialBuffer[offset]-'0') * 10;
  uint8_t db1 = (uint8_t)(dialBuffer[offset+1]-'0');

  return db10 + db1;;
}

uint8_t bufferHexCharToHex(uint8_t offset)
{

  char msbchar = dialBuffer[offset];
  char lsbchar = dialBuffer[offset+1];

  uint8_t msb = (msbchar & 15) +( msbchar >> 6) * 9;
  uint8_t lsb = (lsbchar & 15) +( lsbchar >> 6) * 9;

  return (msb << 4) | lsb;
}

void processRouteDialBuffer()
{  
    uint8_t src_id = bufferDecCharToInt(0);
    uint16_t* GLB_BMT = dialCableOrJack == CABLE ? &GLB_BMT_Cable : &GLB_BMT_Jack;
  
    *GLB_BMT ^= (1 << src_id);
    
    uint8_t numChannels = countSetBits(*GLB_BMT);
    uint8_t sysex[numChannels];
    uint8_t sz = numChannels + 10;
    uint8_t numChannel = 9;
    uint8_t sysexConfig[9] = {0xF0, 0x77, 0x77, 0x78, 0x0F, 0x1, DISP_CableOrJack, DISP_Port, dialCableOrJack};
    uint8_t sysexEnd[1] = {0xF7};
    uint8_t sysexComplete[sz];
  
    memcpy(sysexComplete,sysexConfig,9*sizeof(uint8_t));
    memcpy(sysexComplete+sz-1,sysexEnd,sizeof(uint8_t)); //was byte
    
    for (uint8_t i=0;i<=15;i++){
      if (*GLB_BMT & (1 << i)){
        sysexComplete[numChannel++] = i;
      }
    }
   
    Serial2.write(sysexComplete, sz);
    delay(250);
    
    resetRouteDialBuffer();
    requestPortRouteDump();
}

void processTransformerDialBuffer()
{

  uint8_t setOrClear = 0x1;
  uint8_t slot = bufferHexCharToHex(0);
  uint8_t command = bufferHexCharToHex(2);
  uint8_t x,y,z,d,c,s;

  x = bufferHexCharToHex(4);
  y = bufferHexCharToHex(6);
  z = bufferHexCharToHex(8);
  d = bufferHexCharToHex(10);
  s = bufferHexCharToHex(12);
  c = bufferHexCharToHex(14);

  uint8_t sysex[18] = {
      0xF0, 0x77, 0x77, 0x78, 0x0F, 0x3,
      setOrClear, DISP_CableOrJack, DISP_Port, slot, command, x, y, z, d, s, c,
      0xF7
  };

  Serial.write(sysex,18);
  Serial2.write(sysex, 18);
   
  resetRouteDialBuffer();
  requestPortRouteDump();
}

void toggleFilter(uint8_t filterBit)
{
  
    GLB_Filter ^= (1 << filterBit);
    uint8_t sysexConfig[10] = {0xF0, 0x77, 0x77, 0x78, 0x0F, 0x2, DISP_CableOrJack, DISP_Port, GLB_Filter, 0xF7};

    delay(250);
  
    resetRouteDialBuffer();
    requestPortRouteDump();
   
}

/* Update UI */

uint8_t inputEnabled = 0;

void processScreen1()
{
  displayWrite(&display, menus[UI_Function_L1].name, 0, 0, 1);
  displayWrite(&display, menus[UI_Function_L1].members[UI_Function_L2], 0, 9, 0);

  for (int i=0;i<20;i++){
    Serial.print(dialBuffer[i],HEX);Serial.print(",");
  }
  Serial.println();
          
  if (inputEnabled){
      for (int i=0;i<dialBufferPos;i++){
          display.setCursor(i*6,18);
          display.print(dialBuffer[i],HEX);
          display.display();
        
      }
      display.setCursor(0, 36);
      display.print(dialBufferPos);
      
      display.display();
  
    //displayWriteRaw(&display, &dialBuffer[dialBufferPos], VX, 18, 0);
    //VX+=6;

    displayWrite(&display, "Input Enabled", 0, 27, 0);
    
  } else {
    displayWriteRaw(&display, "              ", 0, 18, 0);
    displayWrite(&display, "              ", 0, 27, 0);
    
  }

}

void processScreen2()
{   
    if (UI_Function_L1 == USBMIDIKLIK){ 
   
       if (UI_Function_L2 == ROUTES){

         char cableTargetsTxt[20] = "";
         char jackTargetsTxt[20] = "";
         uint8_t cableTargetsTxtPos = 0;
         uint8_t jackTargetsTxtPos = 0;
      
         displayBlank(&display2); 

         displayWriteRaw(&display2, GLB_ScreenTitle1, 0, 0, 0);
         displayWriteRaw(&display2, GLB_ScreenTitle2, 0, 9, 0);
         displayWriteInt(&display2, GLB_Port, 40, 9, 0);
         displayWriteRaw(&display2, GLB_Filter_XO, 75, 0, 0);
        
         displayWriteRaw(&display2,"To Cables:",0,18,0);
         displayWriteRaw(&display2,"To Jacks:",0,36,0);

         uint8_t cableTargetCount = countSetBits(GLB_BMT_Cable);
         uint8_t jackTargetCount = countSetBits(GLB_BMT_Jack);

         if (cableTargetCount > 0){
               
           for(uint8_t i=0;i<16;i++){
              if (GLB_BMT_Cable & (1 << i)){  
                      
                char dig1 = getdigit(i,0)+'0';
                char dig2 = getdigit(i,1)+'0'; 

                if (i>=10){       
                  cableTargetsTxt[cableTargetsTxtPos++] = dig2;
                  cableTargetsTxt[cableTargetsTxtPos++] = dig1;
                  cableTargetsTxt[cableTargetsTxtPos++] = ' ';
                } else {
                  cableTargetsTxt[cableTargetsTxtPos++] = dig1;
                  cableTargetsTxt[cableTargetsTxtPos++] = ' ';
                }
              }
           } 

           VX=0;
           for (int i=0;i<cableTargetCount; i++) {
              displayWriteRaw(&display2, &cableTargetsTxt[i], VX, 27, 0);
              VX+=6;
           }
                
         }
         
         if (jackTargetCount > 0){
                  
           for(int i=0;i<16;i++){
              if (GLB_BMT_Jack & (1 << i)){
                
                char dig1 = getdigit(i,0)+'0';
                char dig2 = getdigit(i,1)+'0';
         
                if (i>=10){       
                  jackTargetsTxt[jackTargetsTxtPos++] = dig2;
                  jackTargetsTxt[jackTargetsTxtPos++] = dig1;
                  jackTargetsTxt[jackTargetsTxtPos++] = ' ';               
                } else {
                  jackTargetsTxt[jackTargetsTxtPos++] = dig1;
                  jackTargetsTxt[jackTargetsTxtPos++] = ' ';
                }
                 
              }
           } 

           VX=0;          
           for (int i=0;i<jackTargetCount; i++) {
              displayWriteRaw(&display2, &jackTargetsTxt[i], VX, 45, 0);
              VX+=6;
           }    
           
         }

       } else 
       if (UI_Function_L2 == TRANSFORMERS){

          displayBlank(&display2); 
          displayWriteInt(&display2, GLB_Transformers[0].tByte[0], 0, 0, 0);

       }
    } else
    if (UI_Function_L1 == SYSEXBANK){
      
        displayBlank(&display2);
  
        displayWriteRaw(&display2, GLB_ScreenTitle1, 0, 0, 0);
        displayWriteRaw(&display2, GLB_ScreenTitle2, 0, 9, 0);
    }
    
}

/* Process Incoming Data */

void processIncomingIRByte(uint8_t inByte)
{ 
    Serial.println(inByte);

    switch(inByte){

        case 12: // CH+ plus transformerslot, plus port
          if ( UI_Function_L1 == USBMIDIKLIK ) {
            if ( UI_Function_L2 == ROUTES ) {
              if (DISP_Port != 15) DISP_Port++; 
              requestPortRouteDump();
            } else
            if ( UI_Function_L2 == TRANSFORMERS ) {
              if (DISP_TransformerSlot != 2) DISP_TransformerSlot++; 
              requestPortTransformerDump();
            }
          } else if ( UI_Function_L1 == SYSEXBANK ) {
            Serial3.print(BROWSE_FORWARD);
          }
        break;

        case 10: // CH- minus transformerslot, minus port
          if ( UI_Function_L1 == USBMIDIKLIK ) {
            if ( UI_Function_L2 == ROUTES ) {
              if (DISP_Port != 0) DISP_Port--; 
              requestPortRouteDump();
            } else 
            if ( UI_Function_L2 == TRANSFORMERS ) {
              if (DISP_TransformerSlot != 0) DISP_TransformerSlot--;
              requestPortTransformerDump();
            }
          } else if ( UI_Function_L1 == SYSEXBANK ) {
            Serial3.print(BROWSE_BACKWARD);
          }
        break;

        case 11: // CH
           menuNext();
        break;
    
        case 0: case 1: case 2: case 3: case 4:
        case 5: case 6: case 7: case 8: case 9:
          
          dialBuffer[dialBufferPos++] = (char)inByte;
        break;

        case 13: // A or blue prev
          if (UI_Function_L2 == ROUTES){
            toggleFilter(0);
          } else if (UI_Function_L2 == TRANSFORMERS){
            if (inputEnabled){
              dialBuffer[dialBufferPos++] = 'A';
            } else {
              //noop
            }
          }
        break;
           
        case 14: // B or Blue next
          if (UI_Function_L2 == ROUTES){
            toggleFilter(1);
          } else if (UI_Function_L2 == TRANSFORMERS){
            if (inputEnabled){
              dialBuffer[dialBufferPos++] = 'B';
            } else {
              //noop
            }
          }
        break;

        case 15: // C or green play/pause
          if (UI_Function_L2 == ROUTES){
            toggleFilter(2);
          } else if (UI_Function_L2 == TRANSFORMERS){
            if (inputEnabled){
              dialBuffer[dialBufferPos++] = 'C';
            } else {
              //noop
            }
          }
        break;
           
        case 16: // D or purple -
          if (UI_Function_L2 == ROUTES){
            toggleFilter(3);
          } else if (UI_Function_L2 == TRANSFORMERS){
            if (inputEnabled){
              dialBuffer[dialBufferPos++] = 'D';
            } else {
              //noop
            }
          }
        break;    

        case 17: // E or purple +     
          if (UI_Function_L2 == ROUTES){
            dialCableOrJack = CABLE;
          } else if (UI_Function_L2 == TRANSFORMERS){
            if (inputEnabled){
              dialBuffer[dialBufferPos++] = 'E';
            } else {
              //noop              
            }
          }
        break;
           
        case 18: // F , purple EQ
          if (UI_Function_L2 == ROUTES){
            dialCableOrJack = JACK;
          } else if (UI_Function_L2 == TRANSFORMERS){
            if (inputEnabled){
              dialBuffer[dialBufferPos++] = 'F';
            } else {
              //noop
            }
          }
        break;

        case 19: // 100+
        break;
        
        case 20: // 200+
          inputEnabled = !inputEnabled;
          if (!inputEnabled) resetRouteDialBuffer();
          
        break;

    }
    
//    if (dialBufferPos == 2 && UI_Function_L2 == 0) processRouteDialBuffer();
//    if (dialBufferPos == 16 && UI_Function_L2 == 1) processTransformerDialBuffer();
    
    processScreen1();
}

void processIncomingPIStorageByte(uint8_t dataByte) //was byte
{
  Serial.println(dataByte);

  if (dataByte == 0xFF) { //END
    if (currentFlowType == 0xFD) { 
      
      strncpy(GLB_ScreenTitle1, "Sysex Bank -> ", sizeof(GLB_ScreenTitle1));
      strncpy(GLB_ScreenTitle2, serialMessageBuffer, sizeof(GLB_ScreenTitle2));

      serialMessageBufferIDX = 0;
      memset(serialMessageBuffer, 0, sizeof(serialMessageBuffer));
    }
    processScreen2();
    currentFlowType = 0;

  } else if (dataByte == 0xFD || dataByte == 0xFE) { //START
    currentFlowType = dataByte;    
  } else {
    if (currentFlowType == 0xFD) {
      serialMessageBuffer[serialMessageBufferIDX++] = dataByte;
    }
    else if (currentFlowType == 0xFE) {
      sysexForward(dataByte);
    }
  }
  
}

void processIncomingMidiKlikByte(uint8_t dataByte) //was byte
{
  serialMessageBuffer[serialMessageBufferIDX++] = dataByte;

  if (UI_Function_L1 == USBMIDIKLIK) {//Received data is cable/jack routing
      
      if (dataByte == 0xF7) { //END
       
          serialMessageBufferIDX = 0;
          
          uint8_t RCV_RouteOrFilter = serialMessageBuffer[5];
          uint8_t RCV_CableOrJackSrc = serialMessageBuffer[6];
          uint8_t RCV_Port = serialMessageBuffer[7];
          uint8_t RCV_CableOrJackTgtOrFilter = serialMessageBuffer[8];
     
          strncpy(GLB_ScreenTitle1, "ROUTE", sizeof(GLB_ScreenTitle1));
          strncpy(GLB_ScreenTitle2, RCV_CableOrJackSrc == CABLE ? "CABLE" : "JACK", sizeof(GLB_ScreenTitle2));
          
          GLB_Port = RCV_Port;
      
          if (RCV_RouteOrFilter == ROUTE) { 
    
              uint8_t dtstrt = 9;
              
              if (RCV_CableOrJackTgtOrFilter == CABLE){
                  digitalWrite(LED_BLUE, LOW);
                  GLB_BMT_Cable = 0x0;
                  while (serialMessageBuffer[dtstrt] != 0xF7){
                    GLB_BMT_Cable |= (1 << serialMessageBuffer[dtstrt++]);
                  }
              } else 
              if (RCV_CableOrJackTgtOrFilter == JACK){
                  digitalWrite(LED_RED, LOW);
                  GLB_BMT_Jack = 0x0;
                  while (serialMessageBuffer[dtstrt] != 0xF7){
                    GLB_BMT_Jack |= (1 << serialMessageBuffer[dtstrt++]);
                  }
              }
               
          } else
          if (RCV_RouteOrFilter == FILTER) {  
              digitalWrite(LED_GREEN, LOW);
              GLB_Filter = RCV_CableOrJackTgtOrFilter;
              for (int i=0;i<4;i++){
                GLB_Filter_XO[i] = RCV_CableOrJackTgtOrFilter & (1 << i) ? 'X' : '-';
              }
          } else 
          if (RCV_RouteOrFilter == TRANSFORMER){
                
                uint8_t RCV_TransformerSlot = serialMessageBuffer[9];

                strncpy(GLB_ScreenTitle1, "TRANSFORMERS", sizeof(GLB_ScreenTitle1));
                strncpy(GLB_ScreenTitle2, (const char *)RCV_TransformerSlot, sizeof(GLB_ScreenTitle2));

                GLB_Transformers[RCV_TransformerSlot].tByte[0] = RCV_TransformerSlot;      // slot
                GLB_Transformers[RCV_TransformerSlot].tByte[1] = serialMessageBuffer[10];  // command
                GLB_Transformers[RCV_TransformerSlot].tByte[2] = serialMessageBuffer[11];  // x
                GLB_Transformers[RCV_TransformerSlot].tByte[3] = serialMessageBuffer[12];  // y
                GLB_Transformers[RCV_TransformerSlot].tByte[4] = serialMessageBuffer[13];  // z
                GLB_Transformers[RCV_TransformerSlot].tByte[5] = serialMessageBuffer[14];  // d
                GLB_Transformers[RCV_TransformerSlot].tByte[6] = serialMessageBuffer[15];  // s
                GLB_Transformers[RCV_TransformerSlot].tByte[7] = serialMessageBuffer[16];  // c
          }
          
          if (--pendingConfigPackets == 0)
          processScreen2();
          
      } 
  }
  else if (UI_Function_L1 == SYSEXBANK) { //received data is data for pi storage, - forward
    Serial3.print(dataByte); 
    
  }

}

/* Main */

void menuNext()
{
  UI_Function_L2 = UI_Function_L2 == ML2[UI_Function_L1] -1 ? 0 : UI_Function_L2 + 1;
  if (UI_Function_L2 == 0) UI_Function_L1 = UI_Function_L1 == ML1 - 1 ? 0 : UI_Function_L1 + 1; 
  
  if ( UI_Function_L1 == USBMIDIKLIK ) {
      
  } else if ( UI_Function_L1 == SYSEXBANK ) {

      if (UI_Function_L2 == SAVES){
        Serial3.print(REC_MODE);
      } else if (UI_Function_L2 == LOADS){
        Serial3.print(DISPLAY_CURRENT);
      }
      
  }
  
}

void processSerial()
{
  if (Serial2.available() > 0)
  {
    inByte = Serial2.read();
    processIncomingMidiKlikByte(inByte);
  }

  if (Serial3.available() > 0)
  {
    inByte = Serial3.read();
    processIncomingPIStorageByte(inByte);
  }

  if (AVRSerial.available() > 0)
  {
    inByte = AVRSerial.read();
    processIncomingIRByte(inByte);

  }
}

void loop()
{
  processSerial();   
  delay(500);
}

void setup()
{
  
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);

  Serial.begin(9600);
  Serial1.begin(31250);
  Serial2.begin(31250);
  Serial3.begin(9600);
  AVRSerial.begin(9600);
  
  delay(4000);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  displayWrite(&display, "READY> ", 0, 0, 1);

  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display2.setTextSize(1);
  display2.setTextColor(WHITE);    
  displayWrite(&display2, "READY> ", 0, 0, 1);

#ifdef MIDIUSB_ENABLE
  usb_midi_set_vid_pid(0x2710, 0x1973);
  usb_midi_set_product_string("sysEXBOX");
  MidiUSB.begin();
  delay(4000);
#endif

  processScreen1();
  requestPortRouteDump(); 
  
}
