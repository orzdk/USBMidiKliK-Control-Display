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

struct menu {
  char name[12];
  char *members[20];
};

const struct menu menus[] = {
  {"USBMidiKlik", {"Routes", "Filters"} },
  {"Sysex Bank",  {"Save", "Load"} }
};

uint8_t ML1 = 2;
uint8_t ML2[2] = {2, 2};
uint8_t UI_Function_L1 = 0;
uint8_t UI_Function_L2 = 0;

char keyPress;
char lastKeyPress;
char dialBuffer[3];
char rawBuffer[256];
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

uint8_t i,ii,iii = 0;

uint8_t D_routeOrFilter = ROUTE;
uint8_t D_cableOrJack = CABLE;
uint8_t D_cableOrJack_t = JACK;
uint8_t D_port = 0;

uint16_t currentBmt = 0x00000000;
char dialCableOrJack = 'R';

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
      n &= (n-1) ; 
      count++; 
    } 
    return count; 
} 

void currentBmtToSysex()
{

  uint8_t numChannels = countSetBits(currentBmt);
  uint8_t sysex[numChannels];
  uint8_t numChannel;
  
  
  for (ii=0;ii<=15;ii++){
    if (currentBmt & (1UL << ii)){
      sysex[numChannel++] = ii;
    }
  }

  for (iii=0;iii<numChannels;iii++){
    Serial.println(sysex[iii]);  
  }
  
}

void processRouteDialBuffer()
{
  
  uint8_t db0 = (uint8_t)(dialBuffer[0]-'0')*10;
  uint8_t db1 = (uint8_t)(dialBuffer[1]-'0');
  uint8_t src_id = db0 + db1; 

  currentBmt ^= 1UL << src_id-1;
  
  uint8_t numChannels = countSetBits(currentBmt);
  uint8_t sysex[numChannels];
  uint8_t sz = numChannels + 10;
  uint8_t datapos = 9;
 
  uint8_t sysexConfig[9] = {0xF0, 0x77, 0x77, 0x78, 0x0F, D_routeOrFilter, D_cableOrJack, D_port, D_cableOrJack_t};
  uint8_t sysexEnd[1] = { 0xF7 };
  uint8_t sysexComplete[sz];
 
  memcpy(sysexComplete,sysexConfig,9*sizeof(uint8_t));
  memcpy(sysexComplete+sz-1,sysexEnd,sizeof(byte));
  
  for (ii=0;ii<=15;ii++){
    if (currentBmt & (1UL << ii)){
      sysexComplete[datapos++] = ii;
    }
  }
       
   for (int i = 0; i<sz; i++){
      Serial.print(i);Serial.print(" : ");
      if (i>=0 && i<5) Serial.print("Header ");
      if (i==5) Serial.print("RouteOrFilter ");
      if (i==6) Serial.print("CableOrJack ");
      if (i==7) Serial.print("Port ");
      if (i==8) Serial.print("CableOrJackT ");
      if (i>8) Serial.print("DATA ");
      if ( sysexComplete[i] < 0x10 ) Serial.print("0");
      Serial.println(sysexComplete[i], HEX);
   }

     for (int i = 0; i<sz; i++){
      if ( sysexComplete[i] < 0x10 ) Serial.print("0");
      Serial.print(sysexComplete[i], HEX);
      Serial.print(" ");
     }
   
  Serial2.write(sysexComplete, sz);
  resetDialBuffer();
  
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

void displayWrite2(Adafruit_SSD1306* disp, char* txt, int x, int y, bool clr)
{
  if (clr) disp->clearDisplay();

  disp->setTextSize(1);
  disp->setTextColor(WHITE);
  disp->setCursor(x, y);
  disp->print(txt[0], HEX);
  disp->display();
}

void dumpBufferToScreen(char buf[], Adafruit_SSD1306* screen, uint8_t startline, uint8_t clr, uint8_t startbyte)
{
  uint8_t v_add = 0;

  for (int i = startbyte; i < serialMessageBufferIDX - 1; i++) {

    v_add = 12;

    if ( buf[i] < 0x10 ) {
      displayWrite(screen, "0", VX, startline, 0);
      VX += 6;
      v_add = 6;
    }

    displayWrite2(screen, &buf[i], VX, startline, 0);
    VX += v_add + 6;

  }

  VX = 0;
}

void dumpBufferToScreen2(char buf[], Adafruit_SSD1306* screen, uint8_t startline, uint8_t clr, uint8_t startbyte)
{
  if (clr == 1) screen->clearDisplay();

  for (int i = startbyte; i < serialMessageBufferIDX - 1; i++) {

    displayWrite(screen, &buf[i], VX, startline, 0);
    VX += 6;
  }

  VX = 0;
}

void dumpBufferToSerial(char buf[], uint8_t buflen)
{
     Serial.println(buf[0], HEX); 
     
     for (int i = 1; i<buflen; i++){
        if ( buf[i] < 0x10 ) Serial.print("0");
        Serial.println(buf[i], HEX);
     }

}

void processSerial3Input(byte dataByte)
{
  // Dataflow from PI Storage
  // currentFlowType: 0xFD = internal system message, 0xFF = sysex
  // dataByte: 0xFF = message/sysex end, 0xFE = sysex start, 0xFD = message start

  if (dataByte == 0xFF) { //END

    if (currentFlowType == 0xFD) {
      dumpBufferToScreen2(serialMessageBuffer, &display2, 18, 1, 0);
      dumpBufferToSerial(serialMessageBuffer, sizeof(serialMessageBuffer));
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

void processSerial2Input(byte dataByte)
{
  serialMessageBuffer[serialMessageBufferIDX++] = dataByte;

  if (dataByte == 0xF7) { //END
   
    uint8_t routeOrFilter = serialMessageBuffer[5];
    uint8_t cableOrJackS = serialMessageBuffer[6];
    uint8_t portNum = serialMessageBuffer[7];
    uint8_t cableOrJackT_or_Filter = serialMessageBuffer[8];

    if (routeOrFilter == ROUTE && cableOrJackT_or_Filter == JACK)
    
    
    displayWrite2(&display2, (char*)&D_port, 50, 9, 0);
    
    if (cableOrJackS == CABLE) {
      displayWrite(&display2, "CABLE: ", 0, 9, 0);
    } else if (cableOrJackS == JACK) {
      displayWrite(&display2, "JACK: ", 0, 9, 0);
    }
    
    displayWrite2(&display2, &serialMessageBuffer[7], 40, 9, 0);
      
    if (routeOrFilter == ROUTE) {
      
      currentBmt = 0x0;
      
      for (int i = 9; i < serialMessageBufferIDX - 1; i++) {
         currentBmt |= (1UL << serialMessageBuffer[i]);
      }
      
      displayWrite(&display2, "ROUTES", 0, 0, 0);
    
      if (cableOrJackT_or_Filter == CABLE) {
        displayWrite(&display2, "CABLE: ", 0, 18, 0);
        dumpBufferToScreen(serialMessageBuffer, &display2, 27, 0, 9);
      } else if (cableOrJackT_or_Filter == JACK) {
        displayWrite(&display2, "JACK: ", 0, 36, 0);
        dumpBufferToScreen(serialMessageBuffer, &display2, 45, 0, 9);
      }
      
    } else if (routeOrFilter == FILTER) {
      
      displayWrite(&display2, "FILTERS", 0, 0, 0); 
      displayWrite2(&display2, (char*)cableOrJackT_or_Filter, 0, 18, 0);
    
    }

    serialMessageBufferIDX = 0;
    memset(serialMessageBuffer, 0, sizeof(serialMessageBuffer));
  }

}

void requestCableOrJackInfo(uint8_t routeOrFilter, uint8_t cableOrSerial_Src, uint8_t portSelect, uint8_t cableOrSerial_Tgt)
{
  uint8_t i;
  uint8_t cst = routeOrFilter == 0x01 ? cableOrSerial_Tgt : 0x0;
  uint8_t sysex[10] = {0xF0, 0x77, 0x77, 0x78, 0x04, routeOrFilter, cableOrSerial_Src, portSelect, cst, 0xF7};

  Serial2.write(sysex, 10);
}

void processScreens()
{
  display.clearDisplay();
  display2.clearDisplay();
  
  displayWrite(&display, menus[UI_Function_L1].name, 0, 0, 0);
  displayWrite(&display, menus[UI_Function_L1].members[UI_Function_L2], 0, 9, 0);

  if ( UI_Function_L1 == 0 ) {

    D_routeOrFilter = UI_Function_L2 == 0 ? ROUTE : FILTER;

    requestCableOrJackInfo(D_routeOrFilter, D_cableOrJack, D_port, 0x1);
    requestCableOrJackInfo(D_routeOrFilter, D_cableOrJack, D_port, 0x0);

  } else if ( UI_Function_L1 == 1 ) {

      displayWrite(&display, "Selecting file", 0, 0, 1);
      displayWrite(&display, "[BLUE] to send", 0, 0, 0);
      Serial3.print(0x1);
      
  }
}

void menuNext()
{
  UI_Function_L2 = UI_Function_L2 == ML2[UI_Function_L1] - 1 ? 0 : UI_Function_L2 + 1;
  if (UI_Function_L2 == 0) UI_Function_L1 = UI_Function_L1 == ML1 - 1 ? 0 : UI_Function_L1 + 1;

  processScreens();
}

void processButtons()
{
  if (UI_Function_L1 == 0){
    keyPress = customKeypad.getKey();
    if (keyPress) processRouteDialKeypad();
  }

  tbRed.read();
  tbBlack.read();
  tbBlue.read();
  tbYellow.read();

  if (tbRed.changed()) {
    menuNext();
  }

  if (tbYellow.changed()) {

    if ( UI_Function_L1 == 0 ) {

      D_routeOrFilter = UI_Function_L2 == 0 ? 0x1 : 0x2;
      D_port = D_port == 15 ? 0 : D_port + 1;
      displayWrite2(&display2, (char*)&D_port, 50, 9, 1);
      if (D_port == 0) D_cableOrJack = !D_cableOrJack;

      requestCableOrJackInfo(D_routeOrFilter, D_cableOrJack, D_port, D_cableOrJack_t);

      
    } else if ( UI_Function_L1 == 1 ) {
      displayWrite(&display, "Selecting file", 0, 0, 1);
      displayWrite(&display, "[BLUE] to send", 0, 0, 0);
      Serial3.print(0x1);
    }

  }

  if (tbBlue.changed()) {

    if ( UI_Function_L1 == 0 ) {

      D_routeOrFilter = UI_Function_L2 == 0 ? 0x1 : 0x2;
      D_port = D_port == 0 ? 15 : D_port - 1;
      if (D_port == 0) D_cableOrJack = !D_cableOrJack;
      displayWrite2(&display2, (char*)&D_port, 50, 9, 1);
      requestCableOrJackInfo(D_routeOrFilter, D_cableOrJack, D_port, D_cableOrJack_t);

    } else if ( UI_Function_L1 == 1 ) {
      displayWrite(&display, "Sending file", 0, 0, 1);
      Serial3.print(0x2);
    }

  }

  if (tbBlack.changed()) {
  }

}

void processSerial()
{
  if (Serial2.available() > 0)
  {
    inByte = Serial2.read();
    processSerial2Input(inByte);

  }

  if (Serial3.available() > 0)
  {
    inByte = Serial3.read();
    processSerial3Input(inByte);
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
  usb_midi_set_product_string("SysexBOX");
  MidiUSB.begin();
  delay(4000);
#endif

  processScreens();

}
