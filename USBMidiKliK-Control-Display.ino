#include <JC_Button.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>
#include "Keypad.h"
#include "usb_midi.h"
#include "usb_midi_device.h"
#include "hardware_config.h"
#include "USBMidiKliK-Control-Display.h"
#include "klikpins.h"

#define OLED_RESET -1
#define LED_PULSE_MILLIS  500
#define MIDIUSB_ENABLE__NOPE

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_SSD1306 display2(OLED_RESET);

ToggleButton tbRed(B_RED, false, 250, false, true);
ToggleButton tbBlack(B_BLACK, false, 250, false, true);
ToggleButton tbBlue(B_BLUE, false, 250, false, true);
ToggleButton tbYellow(B_YELLOW, false, 250, false, true);

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, 4, 4);

struct menu {
  char name[12];
  char *members[20];
};

const struct menu menus[] = {
  {"USBMidiKlik", {"Routes"} },
  {"Sysex Bank",  {"Load", "Save"} }
};

uint8_t ML1 = 2;
uint8_t ML2[2] = {1, 2};
uint8_t UI_Function_L1 = 0;
uint8_t UI_Function_L2 = 0;

enum rf {ROUTE=0x1, FILTER=0x2};
enum cj {CABLE=0x0, JACK=0x1};
enum m1 {USBMIDIKLIK=0x0, SYSEXBANK=0x1};
enum m21 {LOADS=0x0, SAVES=0x1};
enum m22 {ROUTES=0x0, FILTERS=0x1};

char keyPress;
char dialBuffer[3];
uint8_t dialBufferPos = 0, nums = 0;
uint8_t VX = 0, VY = 0;

byte currentFlowType = 0x0;
char serialMessageBuffer[64];
uint8_t serialMessageBufferIDX = 0;
byte inByte;

USBMidi MidiUSB;
HardwareSerial * serialHw[SERIAL_INTERFACE_MAX] = {SERIALS_PLIST};

uint8_t packetLen = 0;
midiPacket_t pk { .i = 0 };

uint8_t dialCableOrJack = CABLE;

char GLB_ScreenTitle1[20] = "-ROUTES-";
char GLB_ScreenTitle2[20] = "CABLE: ";
uint8_t GLB_Port = 0;
uint8_t GLB_Filter = 0;
uint16_t GLB_BMT_Cable;
uint16_t GLB_BMT_Jack;
char GLB_Filter_XO[4];

uint8_t DISP_CableOrJack = CABLE;
uint8_t DISP_Port = 0;
bool clearDisp = true;


/* Utilities */

void displayWrite(Adafruit_SSD1306* disp, const char* txt, int x, int y, bool clr)
{
  if (clr) disp->clearDisplay();

  disp->setTextSize(1);
  disp->setTextColor(WHITE);
  disp->setCursor(x, y);
  disp->print(txt);
  disp->display();
}

void displayWriteRaw(Adafruit_SSD1306* disp, char* txt, int x, int y, bool clr)
{
  if (clr) disp->clearDisplay();

  disp->setTextSize(1);
  disp->setTextColor(WHITE);
  disp->setCursor(x, y);
  disp->print(txt);
  disp->display();
}

void displayWriteInt(Adafruit_SSD1306* disp, uint8_t txt, int x, int y, bool clr)
{
  if (clr) disp->clearDisplay();

  disp->setTextSize(1);
  disp->setTextColor(WHITE);
  disp->setCursor(x, y);
  disp->print(txt);
  disp->display();
}

void displayBlank(Adafruit_SSD1306* disp)
{
   disp->clearDisplay();
   disp->display();  
}

uint8_t countSetBits(uint16_t n) 
{ 
    uint8_t count = 0; 
    while (n) 
    { 
      n &= (n-1); 
      count++; 
    } 
    return count; 
} 

void goRed()
{
   digitalWrite(LED_RED, HIGH);
}

void goGreen()
{
    digitalWrite(LED_GREEN, HIGH);
}

/* MIDI */

static void SerialWritePacket(const midiPacket_t *pk, uint8_t serialNo)
{
  uint8_t msgLen = CINToLenTable[pk->packet[0] & 0x0F];
  serialHw[serialNo]->write(&pk->packet[1], msgLen);
}

void sysexForward(byte dataByte)
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

void sysexSend(uint8_t buff[], uint16_t sz) 
{
  midiPacket_t pk { .i = 0};
  uint8_t b=0;
  bool endPk;
  for ( uint16_t i = 0; i != sz ; i ++ ) {
    pk.packet[++b] = buff[i];
    endPk = ( i+2 > sz );
    if (b == 3 ||  endPk ) {
        pk.packet[0]  = endPk ?  b + 4 : 4 ;
        MidiUSB.writePacket(&pk.i);
        b=0; pk.i = 0;
    }
  }
}

void requestChannelDump()
{
  uint8_t sysex_cableTargets[10] = {0xF0, 0x77, 0x77, 0x78, 0x04, 0x1, DISP_CableOrJack, DISP_Port, 0x0, 0xF7};
  uint8_t sysex_jackTargets[10] = {0xF0, 0x77, 0x77, 0x78, 0x04, 0x1, DISP_CableOrJack, DISP_Port, 0x1, 0xF7};
  uint8_t sysex_filterTargets[10] = {0xF0, 0x77, 0x77, 0x78, 0x04, 0x2, DISP_CableOrJack, DISP_Port, 0x0, 0xF7};   
  
  clearDisp = true;
  
  Serial2.write(sysex_cableTargets, 10);delay(50);
  Serial2.write(sysex_jackTargets, 10);delay(50);
  Serial2.write(sysex_filterTargets, 10);delay(50);   
  
}

/* Keypad */

void resetRouteDialBuffer()
{
  nums = 0;
  dialCableOrJack = CABLE;
  dialBufferPos = 0;
  memset(dialBuffer, 0, sizeof(dialBuffer));
}

void processRouteDialBuffer()
{
  uint8_t db10 = (uint8_t)(dialBuffer[0]-'0')*10;
  uint8_t db1 = (uint8_t)(dialBuffer[1]-'0');
  uint8_t src_id = db10 + db1; 
  uint16_t* GLB_BMT = dialCableOrJack == CABLE ? &GLB_BMT_Cable : &GLB_BMT_Jack;

  *GLB_BMT ^= (1 << src_id);
  
  uint8_t numChannels = countSetBits(*GLB_BMT);
  uint8_t sysex[numChannels];
  uint8_t sz = numChannels + 10;
  uint8_t numChannel = 9;
  uint8_t sysexConfig[9] = {0xF0, 0x77, 0x77, 0x78, 0x0F, 0x1, dialCableOrJack, DISP_Port, 0x0};
  uint8_t sysexEnd[1] = {0xF7};
  uint8_t sysexComplete[sz];

  memcpy(sysexComplete,sysexConfig,9*sizeof(uint8_t));
  memcpy(sysexComplete+sz-1,sysexEnd,sizeof(byte));
  
  for (uint8_t i=0;i<=15;i++){
    if (*GLB_BMT & (1 << i)){
      sysexComplete[numChannel++] = i;
    }
  }

  for (int i=0;i<sz;i++){
    Serial.println(sysexComplete[i],HEX);
  }
  
  Serial2.write(sysexComplete, sz);
  delay(50);
  resetRouteDialBuffer();
  requestChannelDump();
  
}

void processRouteDialKeypress()
{
  displayWrite(&display, &keyPress, 0, 0, 1);

  switch (keyPress)
  {
    case NO_KEY:
      break;
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
      nums++;
      dialBuffer[dialBufferPos++] = keyPress;
      if (nums == 2) processRouteDialBuffer();
      break;

    case '*': 
      dialCableOrJack = CABLE;
      break;
         
    case '#':
      dialCableOrJack = JACK;
      break;

  }

}

/* UI */

void processScreen1()
{

  displayWrite(&display, menus[UI_Function_L1].name, 0, 0, 1);
  displayWrite(&display, menus[UI_Function_L1].members[UI_Function_L2], 0, 9, 0);

  if ( UI_Function_L1 == USBMIDIKLIK ) {
      displayWrite(&display, "Channels <-U,Y->", 0, 27, 0);
      displayWrite(&display, "Pages B->>", 0, 36, 0);
      displayWrite(&display, "Keypad to config", 0, 45, 0);
  }
  else    
  if ( UI_Function_L1 == SYSEXBANK ) {
      
      if (UI_Function_L2==LOADS){
        
        displayWrite(&display, "Files <-U,Y->", 0, 27, 0);
        displayWrite(&display, "Pages B->>", 0, 36, 0);
        displayWrite(&display, "R to send", 0, 45, 0);
        
      } else {
        if (UI_Function_L2==SAVES){
          displayWrite(&display, "<Ready...>", 0, 18, 0);
          
        }
      } 
  }

}

void processScreen2()
{   
    if (UI_Function_L1 == 0){ //klik
      
       uint8_t cableTargetCount = countSetBits(GLB_BMT_Cable);
       uint8_t jackTargetCount = countSetBits(GLB_BMT_Jack);
    
       char cableTargetsTxt[20] = "";
       char jackTargetsTxt[20] = "";
       uint8_t cableTargetsTxtPos = 0;
       uint8_t jackTargetsTxtPos = 0;
    
       if (clearDisp) {
          displayBlank(&display2); 
    
          displayWriteRaw(&display2, GLB_ScreenTitle1, 0, 0, 0);
          displayWriteRaw(&display2, GLB_ScreenTitle2, 0, 9, 0);
          displayWriteInt(&display2, GLB_Port, 40, 9, 0);
          displayWriteRaw(&display2, GLB_Filter_XO, 75, 0, 0);
          
          displayWriteRaw(&display2,"To Cables:",0,18,0);
          displayWriteRaw(&display2,"To Jacks:",0,36,0);
          
          clearDisp = false;
       }
    
       if (cableTargetCount){
         VX=0; 
       
         for(int i=0;i<16;i++){
            if (GLB_BMT_Cable & (1 << i)){
              cableTargetsTxt[cableTargetsTxtPos++] = i+'0';
              cableTargetsTxt[cableTargetsTxtPos++] = ' ';
            }
         } 
         for (int i=0;i<cableTargetCount; i++) {
            displayWrite(&display2, &cableTargetsTxt[i], VX, 27, 0);
            VX+=6;
         }     
       }
       
       if (jackTargetCount){
         VX=0;     
             
         for(int i=0;i<16;i++){
            if (GLB_BMT_Jack & (1 << i)){
              jackTargetsTxt[jackTargetsTxtPos++] = i+'0';
              jackTargetsTxt[jackTargetsTxtPos++] = ' ';
            }
         } 
         
         for (int i=0;i<jackTargetCount; i++) {
            displayWrite(&display2, &jackTargetsTxt[i], VX, 45, 0);
            VX+=6;
         }    
       }
       
    } else
    if (UI_Function_L1 == 1){ //bank
      
        displayBlank(&display2);
  
        displayWriteRaw(&display2, GLB_ScreenTitle1, 0, 0, 0);
        displayWriteRaw(&display2, GLB_ScreenTitle2, 0, 9, 0);
    }
    
}

/* (Pre-) Process Incoming Data */

void processIncomingPIStorageByte(byte dataByte)
{
  if (dataByte == 0xFF) { //END
    goGreen();
    if (currentFlowType == 0xFD) { 
      
      strncpy(GLB_ScreenTitle1, "Sysex Bank -> ", sizeof(GLB_ScreenTitle1));
      strncpy(GLB_ScreenTitle2, serialMessageBuffer, sizeof(GLB_ScreenTitle2));

      serialMessageBufferIDX = 0;
      memset(serialMessageBuffer, 0, sizeof(serialMessageBuffer));
    }
    processScreen2();
    currentFlowType = 0;

  } else if (dataByte == 0xFD || dataByte == 0xFE) { //START
    goRed();
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

void processIncomingSerial2Byte(byte dataByte)
{
  serialMessageBuffer[serialMessageBufferIDX++] = dataByte;

  if (dataByte == 0xF7) { //END
      goGreen();
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
              GLB_BMT_Cable = 0x0;
              while (serialMessageBuffer[dtstrt] != 0xF7){
                GLB_BMT_Cable |= (1 << serialMessageBuffer[dtstrt++]);
              }
          } else 
          if (RCV_CableOrJackTgtOrFilter == JACK){
              GLB_BMT_Jack = 0x0;
              while (serialMessageBuffer[dtstrt] != 0xF7){
                GLB_BMT_Jack |= (1 << serialMessageBuffer[dtstrt++]);
              }
          }
         
          
      } else
      if (RCV_RouteOrFilter == FILTER) {  
          GLB_Filter = RCV_CableOrJackTgtOrFilter;
          for (int i=0;i<4;i++){
            GLB_Filter_XO[i] = RCV_CableOrJackTgtOrFilter & (1 << i) ? 'X' : '-';
          }
      }

      processScreen2();
      
  } else {
    goRed();
  }

}

/* Navigation */

void menuNext()
{
  UI_Function_L2 = UI_Function_L2 == ML2[UI_Function_L1] - 1 ? 0 : UI_Function_L2 + 1;
  if (UI_Function_L2 == 0) UI_Function_L1 = UI_Function_L1 == ML1 - 1 ? 0 : UI_Function_L1 + 1; 
  
  if ( UI_Function_L1 == USBMIDIKLIK ) {
      digitalWrite(LED_YELLOW, HIGH);
      digitalWrite(LED_BLUE, LOW);

  } else if ( UI_Function_L1 == SYSEXBANK ) {
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_BLUE, HIGH);
      
      display2.clearDisplay();
      display2.display();
    
      Serial3.print(0x1);
  }
  
}

/* Main */

void processButtons()
{
  tbRed.read();
  tbBlack.read();
  tbBlue.read();
  tbYellow.read();

  if (tbBlack.changed()) {
     menuNext();
     processScreen1();
  }
  
  if (tbRed.changed()) {
     requestChannelDump(); 
  }
  
  if (tbYellow.changed()) {
    if ( UI_Function_L1 == USBMIDIKLIK ) {
      if (DISP_Port != 15) DISP_Port++; else {
        DISP_CableOrJack = !DISP_CableOrJack;
        DISP_Port = 0;
      }
      requestChannelDump();
    } else if ( UI_Function_L1 == SYSEXBANK ) {
      Serial3.print(0x1);
    }
  }

  if (tbBlue.changed()) {
    if ( UI_Function_L1 == USBMIDIKLIK ) {
      if (DISP_Port != 0) DISP_Port--; else {
        DISP_CableOrJack = !DISP_CableOrJack;
        DISP_Port = 0;        
      }
      requestChannelDump();
    } else if ( UI_Function_L1 == SYSEXBANK ) {
      Serial3.print(0x2);
    }
  }

  keyPress = customKeypad.getKey();
  if (keyPress) processRouteDialKeypress();
  
}

void processSerial()
{
  if (Serial2.available() > 0)
  {
    inByte = Serial2.read();
    processIncomingSerial2Byte(inByte);

  }

  if (Serial3.available() > 0)
  {
    inByte = Serial3.read();
    processIncomingPIStorageByte(inByte);
  }

}

void loop()
{
  processButtons();
  processSerial();
}

void setup()
{
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);

  pinMode(B_RED, INPUT);
  pinMode(B_BLACK, INPUT);
  pinMode(B_BLUE, INPUT);
  pinMode(B_YELLOW, INPUT);

  tbRed.begin();
  tbBlack.begin();
  tbBlue.begin();
  tbYellow.begin();

  Serial.begin(9600);
  Serial1.begin(31250);
  Serial2.begin(31250);
  Serial3.begin(9600);
  delay(4000);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  displayWrite(&display, "READY> ", 0, 0, 1);

  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  displayWrite(&display2, "READY> ", 0, 0, 1);

#ifdef MIDIUSB_ENABLE
  usb_midi_set_vid_pid(0x2710, 0x1973);
  usb_midi_set_product_string("sysEXBOX");
  MidiUSB.begin();
  delay(4000);
#endif

  processScreen1();
  requestChannelDump(); 
  
}
