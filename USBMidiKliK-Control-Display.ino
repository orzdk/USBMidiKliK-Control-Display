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
#define MIDIUSB_ENABLE__NOPE

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_SSD1306 display2(OLED_RESET);

ToggleButton tbRed(B_RED, false, 250, false, true);
ToggleButton tbBlack(B_BLACK, false, 250, false, true);
ToggleButton tbBlue(B_BLUE, false, 250, false, true);
ToggleButton tbYellow(B_YELLOW, false, 250, false, true);

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, 4, 4);

enum rf {ROUTE=0x1, FILTER=0x2};
enum cj {CABLE=0x0, JACK=0x1};
enum m1 {USBMIDIKLIK=0x0, SYSEXBANK=0x1};
enum m21 {LOADS=0x0, SAVES=0x1};

struct menu {
  char name[12];
  char *members[20];
};

const struct menu menus[] = {
  {"USBMidiKlik", {"Routes", "Filters"} },
  {"Sysex Bank",  {"Load", "Save"} }
};

uint8_t ML1 = 2;
uint8_t ML2[2] = {2, 2};
uint8_t UI_Function_L1 = 0;
uint8_t UI_Function_L2 = 0;

char keyPress;
char dialBuffer[3];
uint8_t dialBufferPos = 0, nums = 0;

uint8_t routeTable[] = {0, 1, 0};
uint8_t VX = 0, VY = 0;

byte currentFlowType = 0x0;
char serialMessageBuffer[64];
uint8_t serialMessageBufferIDX = 0;
byte inByte;

USBMidi MidiUSB;
HardwareSerial * serialHw[SERIAL_INTERFACE_MAX] = {SERIALS_PLIST};

uint8_t packetLen = 0;
midiPacket_t pk { .i = 0 };

char dialCableOrJack = 'R';

char GLB_ScreenTitle1[20];
char GLB_ScreenTitle2[20];
uint8_t GLB_Port = 0;
uint8_t GLB_Filter = 0;
uint16_t GLB_BMT_Cable;
uint16_t GLB_BMT_Jack;

uint8_t DISP_RouteOrFilter = ROUTE;
uint8_t DISP_CableOrJack = CABLE;
uint8_t DISP_Port = 0;

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

void SysexSend(uint8_t buff[], uint16_t sz) 
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

void resetDialBuffer()
{
  nums = 0;
  dialCableOrJack = 'R';
  dialBufferPos = 0;
  memset(dialBuffer, 0, sizeof(dialBuffer));
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

void processRouteDialBuffer()
{
  uint8_t db0 = (uint8_t)(dialBuffer[0]-'0')*10;
  uint8_t db1 = (uint8_t)(dialBuffer[1]-'0');
  uint8_t src_id = db0 + db1; 
  uint16_t* GLB_BMT = dialCableOrJack == CABLE ? &GLB_BMT_Cable : &GLB_BMT_Jack;

  *GLB_BMT ^= (1UL << src_id-1);
  
  uint8_t numChannels = countSetBits(*GLB_BMT);
  uint8_t sysex[numChannels];
  uint8_t sz = numChannels + 10;
  uint8_t numChannel = 9;
  uint8_t sysexConfig[9] = {0xF0, 0x77, 0x77, 0x78, 0x0F, DISP_RouteOrFilter, DISP_CableOrJack, DISP_Port, 0x0};
  uint8_t sysexEnd[1] = { 0xF7 };
  uint8_t sysexComplete[sz];

  memcpy(sysexComplete,sysexConfig,9*sizeof(uint8_t));
  memcpy(sysexComplete+sz-1,sysexEnd,sizeof(byte));
  
  for (int i=0;i<=15;i++){
    if (*GLB_BMT & (1UL << i)){
      sysexComplete[numChannel++] = i;
    }
  }
  
  Serial2.write(sysexComplete, sz);
  resetDialBuffer();
  requestCurrent();
  
}

void processRouteDialKeypad()
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

void processPIStorageInput(byte dataByte)
{
  // Dataflow from PI Storage
  // currentFlowType: 0xFD = internal system message, 0xFF = sysex
  // dataByte: 0xFF = message/sysex end, 0xFE = sysex start, 0xFD = message start

  if (dataByte == 0xFF) { //END

    if (currentFlowType == 0xFD) { // Not sysex
      
      VX=0;
      for (int i=0; i<serialMessageBufferIDX-1; i++) {
        displayWrite(&display2, &serialMessageBuffer[i], VX, 9, 0);
        VX += 6;
      }

      serialMessageBufferIDX = 0;
      memset(serialMessageBuffer, 0, sizeof(serialMessageBuffer));
    }
    
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

void processGLB()
{  
   uint8_t cableTargets = countSetBits(GLB_BMT_Cable);
   uint8_t jackTargets = countSetBits(GLB_BMT_Jack);

   char cableTargetsTxt[20];
   char jackTargetsTxt[20];
   uint8_t cableTargetsTxtPos = 0;
   uint8_t jackTargetsTxtPos = 0;
   
   displayWriteRaw(&display2, GLB_ScreenTitle1, 0, 0, 1);
   displayWriteRaw(&display2, GLB_ScreenTitle2, 0, 9, 0);
   displayWriteRaw(&display2, &serialMessageBuffer[7], 50, 9, 0);
   
   for(int i=0;i<16;i++){
      if (GLB_BMT_Cable & (1 << i)){
        cableTargetsTxt[cableTargetsTxtPos++] = i+'0';
        cableTargetsTxt[cableTargetsTxtPos++] = ',';
      }
   }

   for(int i=0;i<16;i++){
      if (GLB_BMT_Jack & (1 << i)){
        jackTargetsTxt[jackTargetsTxtPos++] = i+'0';
        jackTargetsTxt[jackTargetsTxtPos++] = ',';
      }
   }

   displayWriteRaw(&display2,"CABLE TARGETS:",0,18,0);
   VX=0;  
   
   for (int i=0;i<cableTargets; i++) {
      displayWriteRaw(&display2, &cableTargetsTxt[i], VX, 27, 0);
      VX += 6;
   }

   displayWriteRaw(&display2,"JACK TARGETS:",0,36,0);
   VX=0;
   
   for (int i=0;i<jackTargets; i++) {
      displayWriteRaw(&display2, &jackTargetsTxt[i], VX, 45, 0);
      VX += 6;
   }
  
   
}

void processMidiKlikConfigDumpData(byte dataByte)
{

  serialMessageBuffer[serialMessageBufferIDX++] = dataByte;

  if (dataByte == 0xF7) { //END

    uint8_t RCV_RouteOrFilter = serialMessageBuffer[5];
    uint8_t RCV_CableOrJackSrc = serialMessageBuffer[6];
    uint8_t RCV_Port = serialMessageBuffer[7];
    uint8_t RCV_CableOrJackTgtOrFilter = serialMessageBuffer[8];

    strncpy(GLB_ScreenTitle1, RCV_RouteOrFilter == ROUTE ? "ROUTE" : "FILTER", sizeof(GLB_ScreenTitle1));
    strncpy(GLB_ScreenTitle2, RCV_CableOrJackSrc == CABLE ? "CABLE" : "JACK", sizeof(GLB_ScreenTitle2));
    
    GLB_Port = RCV_Port;

    if (RCV_RouteOrFilter == ROUTE) { 
      
        if (RCV_CableOrJackTgtOrFilter == CABLE){
            GLB_BMT_Cable = 0x0;
            for (int i = 9; i < serialMessageBufferIDX - 1; i++) {
               GLB_BMT_Cable |= (1UL << serialMessageBuffer[i]);
            }
        } else 
        if (RCV_CableOrJackTgtOrFilter == CABLE){
            GLB_BMT_Jack = 0x0;
            for (int i = 9; i < serialMessageBufferIDX - 1; i++) {
               GLB_BMT_Jack |= (1UL << serialMessageBuffer[i]);
            }
        }
        
    } else
    if (RCV_RouteOrFilter == FILTER) { 
        GLB_Filter = RCV_CableOrJackTgtOrFilter;
    }
   
    processGLB();
  }
  else {
    
  }
}

void requestChannelJackTargets(uint8_t cblOrJack, uint8_t port)
{
  uint8_t sysex[10] = {0xF0, 0x77, 0x77, 0x78, 0x04, 0x1, cblOrJack, port, 0x1, 0xF7};
  Serial2.write(sysex, 10);
}

void requestChannelCableTargets(uint8_t cblOrJack, uint8_t port)
{
  uint8_t sysex[10] = {0xF0, 0x77, 0x77, 0x78, 0x04, 0x1, cblOrJack, port, 0x0, 0xF7};
  Serial2.write(sysex, 10);
}

void requestChannelFilters(uint8_t cblOrJack, uint8_t port)
{
  uint8_t sysex[9] = {0xF0, 0x77, 0x77, 0x78, 0x04, 0x2, cblOrJack, port, 0xF7};
  Serial2.write(sysex, 9);
}

void requestCurrent()
{

  if (DISP_RouteOrFilter == ROUTE){
    uint8_t sysex[10] = {0xF0, 0x77, 0x77, 0x78, 0x04, 0x1, DISP_CableOrJack, DISP_Port, 0x0, 0xF7};
    Serial2.write(sysex, 10);

    uint8_t sysex2[10] = {0xF0, 0x77, 0x77, 0x78, 0x04, 0x1, DISP_CableOrJack, DISP_Port, 0x1, 0xF7};
    Serial2.write(sysex, 10);

  }else
  if (DISP_RouteOrFilter == FILTER){
    uint8_t sysex[9] = {0xF0, 0x77, 0x77, 0x78, 0x04, 0x2, DISP_CableOrJack, DISP_Port, 0xF7};
    Serial2.write(sysex, 9);
  }
  
}

void processScreens()
{
 
  Serial.println("processScreens()");

  displayWrite(&display, menus[UI_Function_L1].name, 0, 0, 1);
  displayWrite(&display, menus[UI_Function_L1].members[UI_Function_L2], 0, 9, 0);

  if ( UI_Function_L1 == SYSEXBANK ) {
      
      if (UI_Function_L2==LOADS){
        
        displayWrite(&display, "<-[B][Y]->", 0, 27, 0);
        displayWrite(&display, "[RED] to send", 0, 36, 0);
        
        Serial3.print(0x1);
        
      } else {
        if (UI_Function_L2==SAVES){
          displayWrite(&display, "Ready to receive", 0, 18, 0);
          
        }
      } 
  }

}

void menuSelect()
{
  requestChannelCableTargets(DISP_CableOrJack, DISP_Port); 
}

void menuNext()
{
  UI_Function_L2 = UI_Function_L2 == ML2[UI_Function_L1] - 1 ? 0 : UI_Function_L2 + 1;
  if (UI_Function_L2 == 0) UI_Function_L1 = UI_Function_L1 == ML1 - 1 ? 0 : UI_Function_L1 + 1;

  processScreens();
}

void processButtons()
{

  tbRed.read();
  tbBlack.read();
  tbBlue.read();
  tbYellow.read();

  if (tbRed.changed()) {
     menuSelect();
  }
  
  if (tbBlack.changed()) {
     menuNext();
  }
  
  if (tbYellow.changed()) {

    if ( UI_Function_L1 == SAVES ) {

      DISP_RouteOrFilter = UI_Function_L2+1;
      if (DISP_Port != 15) DISP_Port++;
      
      requestChannelCableTargets(DISP_CableOrJack, DISP_Port);
      requestChannelJackTargets(DISP_CableOrJack, DISP_Port);
      
    } else if ( UI_Function_L1 == LOADS ) {
      
      displayWrite(&display, "Selecting file", 0, 0, 1);
      displayWrite(&display, "[BLUE] to send", 0, 0, 0);
      Serial3.print(0x1);
    }

  }

  if (tbBlue.changed()) {

    if ( UI_Function_L1 == SAVES ) {

      DISP_RouteOrFilter = UI_Function_L2+1;
      if (DISP_Port != 0) DISP_Port--;
      
      requestChannelCableTargets(DISP_CableOrJack, DISP_Port);
      requestChannelJackTargets(DISP_CableOrJack, DISP_Port);

    } else if ( UI_Function_L1 == LOADS ) {
      displayWrite(&display, "Sending file", 0, 0, 1);
      Serial3.print(0x2);
    }

  }

  if (UI_Function_L1 == 0){
    keyPress = customKeypad.getKey();
    if (keyPress) processRouteDialKeypad();
  }

}

void processSerial()
{
  if (Serial2.available() > 0)
  {
    inByte = Serial2.read();
    processMidiKlikConfigDumpData(inByte);

  }

  if (Serial3.available() > 0)
  {
    inByte = Serial3.read();
    processPIStorageInput(inByte);
  }

}

void loop()
{
  processButtons();
  processSerial();
}

void setup()
{
  Serial.println(sizeof(HEX));
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

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

}
