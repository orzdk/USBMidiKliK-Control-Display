#include <JC_Button.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>

#include <usbh_midi.h>
#include <usbhub.h>

#include "Keypad.h"
#include "usb_midi.h"
#include "usb_midi_device.h"
#include "hardware_config.h"
#include "USBMidiKliK-Control-Display.h"
#include "RingBuffer.h";

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
Adafruit_SSD1306 display2(OLED_RESET);

static const uint8_t CINToLenTable[] = {
  0,
  0,
  2, // 0x02
  3, // 0x03
  3, // 0x04
  1, // 0x05
  2, // 0x06
  3, // 0x07
  3, // 0x08
  3, // 0x09
  3, // 0x0A
  3, // 0x0B
  2, // 0x0C
  2, // 0x0D
  3, // 0x0E
  1  // 0x0F
};

char hexaKeys[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};


byte colPins[4] = {PC14, PC15,0,1};
byte rowPins[4] = {7, 6, 5, 4};

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, 4, 4); 

//#define B_RING_BUFFER_PACKET_SIZE  16*sizeof(midiPacket_t)
//RingBuffer<uint8_t,B_RING_BUFFER_PACKET_SIZE> serialRingBuffer;
//serialRingBuffer.write((uint8_t *)&pk,sizeof(midiPacket_t));
//serialRingBuffer.available();
//serialRingBuffer.readBytes(pk.packet,sizeof(midiPacket_t));

char keyPress;
char lastKeyPress;
char dialBuffer[6];
char rawBuffer[256]; 

int d;
uint8_t dialBufferPos = 0, rawBufferPos = 0, tags = 0, nums = 0;
uint8_t mode = 2; 
uint8_t boxMode = 0;

HardwareSerial * serialHw[SERIAL_INTERFACE_MAX] = {SERIALS_PLIST};
USBMidi MidiUSB;

const int B_YELLOW =PB14 ; 
const int B_BLUE =PB12 ;  
const int B_BLACK = PB13; 

const int B_RED = PB15;  
const int LED_GREEN = PB8; 
const int LED_RED = PB9;  

ToggleButton tbRed(B_RED, false, 250, false, true);
ToggleButton tbBlack(B_BLACK, false, 250, false, true);
ToggleButton tbBlue(B_BLUE, false, 250, false, true);
ToggleButton tbYellow(B_YELLOW, false, 250, false, true);

uint8_t sysexID = 3;
byte flowDt = 0x0;
char serialBuffer[128];
uint8_t serialBufferIDX = 0;
int ired,iblack;
uint8_t pkLen = 0;
byte inByte;
uint8_t tlx = 0;
uint8_t tly = 10;

midiPacket_t pk { .i = 0 };

static void SerialWritePacket(const midiPacket_t *pk, uint8_t serialNo) 
{
    uint8_t msgLen = CINToLenTable[pk->packet[0] & 0x0F];
    serialHw[serialNo]->write(&pk->packet[1],msgLen);
}

void SendSysexToSerialAndUSB(const uint8_t *data, size_t size)
{
    if (data == NULL || size == 0) return;

    size_t midiDataSize = (size+2)/3*4;
    uint8_t midiData[midiDataSize];
    const uint8_t *d = data;
    uint8_t *p = midiData;
    
    size_t bytesRemaining = size;

    while (bytesRemaining > 0) {
      switch (bytesRemaining) {
      case 1:
          *p++ = 5;   // SysEx ends with following single byte
          *p++ = *d;
          *p++ = 0;
          *p = 0;
          bytesRemaining = 0;
          break;
      case 2:
          *p++ = 6;   // SysEx ends with following two bytes
          *p++ = *d++;
          *p++ = *d;
          *p = 0;
          bytesRemaining = 0;
          break;
      case 3:
          *p++ = 7;   // SysEx ends with following three bytes
          *p++ = *d++;
          *p++ = *d++;
          *p = *d;
          bytesRemaining = 0;
          break;
      default:
          *p++ = 4;   // SysEx starts or continues
          *p++ = *d++;
          *p++ = *d++;
          *p++ = *d++;
          bytesRemaining -= 3;
          break;
      }
    }

    for (int i=0;i<midiDataSize;i+=4){
      uint32_t package = (midiData[i+3] << 24) | (midiData[i+2] << 16)| (midiData[i+1] << 8) | (midiData[i] | (1 << 4));
      midiPacket_t lpk = { .i = package };
      
      MidiUSB.writePacket(&(lpk.i));
      SerialWritePacket(&(lpk),0);
    }
}

void processSysexFlow(byte dataByte)
{
   if (dataByte == 0xF7){
    
    pk.packet[0] = 5 + pkLen;  
    pk.packet[pkLen+1] = dataByte;
    Serial.print("PCK ");
    Serial.println(pk.i,HEX);

    display2.setTextColor(WHITE);
    display2.setCursor(tlx,tly);
    display2.println(pk.i,HEX);
    display2.display();     
      
    //MidiUSB.wPacket(&pk.i);
    pkLen=0;pk.i = 0;  tly+=10;
    
  } else {
    
      pk.packet[++pkLen] = dataByte;
      if (pkLen == 3) {
          tly+=10;
          tlx=0;        
          pk.packet[0] = 4;
          Serial.print("PCK ");
          Serial.println(pk.i,HEX);

          display2.setTextColor(WHITE);
          display2.setCursor(tlx,tly);
          display2.println(pk.i,HEX);
          display2.display();

          //MidiUSB.writePacket(&pk.i);
          pkLen=0;pk.i = 0;
      }
       
  } 
}

void processDialBuffer()
{
   uint16_t cableMask = 0x0; /*Get from MidiKlik */
   uint16_t jackMask = 0x0; /*Get from MidiKlik */
   
   uint8_t sysex[14] = {0xF0, 0x77, 0x77, 0x78, 0x0F, 0x01,};
   
   int styp = dialBuffer[0] == '#' ? 0 : 1;
   int ttyp = dialBuffer[3] == '#' ? 0 : 1;
   
   int src_id = ((int)(dialBuffer[1]) * 10) + (int)(dialBuffer[2]); 
   int tgt_id = ((int)(dialBuffer[4]) * 10) + (int)(dialBuffer[4]); 
  
   cableMask ^= !ttyp * (1UL << tgt_id);
   jackMask ^= ttyp * (1UL << tgt_id);
   
   sysex[6] = 0xFF & styp;
   sysex[7] = 0xFF & src_id;
   sysex[8] = 0x0F;
   sysex[9] = cableMask >> 8;
   sysex[10] = cableMask & 0xFF;
   sysex[11] = jackMask >> 8;
   sysex[12] = jackMask & 0xFF;
   sysex[13] = 0xF7;

   int i;
   for (i=0; i<sizeof(sysex)/sizeof(sysex[0]);i++){
      Serial.print(sysex[i], HEX);
      Serial.print(" ");
   }
   Serial.println(".......................");
   
   SendSysexToSerialAndUSB(sysex, 14);
   resetBuffers();
}

void processKeypress()
{
    displayWrite(&display, &keyPress,0,0,1);
   
    if (mode==2){
      dialBuffer[dialBufferPos++] = '#';
      dialBuffer[dialBufferPos++] = '1';
      dialBuffer[dialBufferPos++] = '1';
      dialBuffer[dialBufferPos++] = '#';
      dialBuffer[dialBufferPos++] = '1';
      dialBuffer[dialBufferPos++] = '1';
      processDialBuffer();
    }
    else if (mode==1){
      char key = keyPress == '*'?'E':keyPress == '#'?'F':keyPress;
      rawBuffer[rawBufferPos++] = key;
    } 
    else if (mode==0){
      
        switch (keyPress)
        {
          case NO_KEY:
            break;
          
          case '0': case '1': case '2': case '3': case '4':
          case '5': case '6': case '7': case '8': case '9':
            nums++;
            dialBuffer[dialBufferPos++] = keyPress;  
            if (nums == 3 || (nums > 0 && tags == 0)) resetBuffers();
            if (tags == 2 && nums == 2) processDialBuffer();
            break;
    
          case '#': case '*':
            if (lastKeyPress == '#' || lastKeyPress == '*') {
                resetBuffers();
            } else {
               tags++;nums=0;
               dialBuffer[dialBufferPos++] = keyPress;
            }
            break;
    
        }

        lastKeyPress = keyPress;   
    }

}

void resetBuffers()
{
  dialBufferPos = 0;
  rawBufferPos = 0;
  tags=0;
  nums=0;
  keyPress = 'R';
  memset(dialBuffer, 0, sizeof(dialBuffer));
  memset(rawBuffer, 0, sizeof(rawBuffer));
}

void displayWrite(Adafruit_SSD1306 *disp, char *txt, int x, int y, bool clr)
{
  
    if(clr) disp->clearDisplay();
    
    disp->setTextSize(1);
    disp->setTextColor(WHITE);
    disp->setCursor(x,y);
    disp->println(txt);
    disp->display();
}

void processSerialInput(byte dataByte)
{
   
  if (dataByte == 0xFF){ //message or sysex end
    
    if (flowDt == 0xFD){ //message
      
      for (int i = 0; i < serialBufferIDX; i++) Serial.print((char)serialBuffer[i]);
      Serial.println("");     

      display2.clearDisplay();
      display2.setTextSize(1);
      display2.setTextColor(WHITE);
     
      for (int i = 0; i < serialBufferIDX; i++) {
         display2.setCursor(i*6,0);
         display2.println((char)serialBuffer[i]);
      }
      display2.display();
       
      memset(serialBuffer, 0, sizeof(serialBuffer));
      serialBufferIDX = 0;
    }    
    
    flowDt = 0x0;
      
  } else if (dataByte == 0xFD || dataByte == 0xFE){ //message (0xFD) or sysex (0xFE)
     flowDt = dataByte;
  
  } else{
      if (flowDt == 0xFD){                          //message, add to buffer, wait for 0xFF
        serialBuffer[serialBufferIDX++] = dataByte;
      }
      else if (flowDt == 0xFE){                     //sysex, forward
        processSysexFlow(dataByte);
      }
  }

}

void setup() 
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  displayWrite(&display, "KliKController D 1!",0,0,1);
 
  display2.begin(SSD1306_SWITCHCAPVCC, 0x3D); 
  displayWrite(&display2,"KliKController D 2!",0,0,1);

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
  Serial3.begin(9600);
  
//  usb_midi_set_vid_pid(0x2710,0x1973);
//  usb_midi_set_product_string("SysexBOX");
//  MidiUSB.begin();
//  delay(4000);

//  serialHw[0]->begin(31250);
   
}

void loop() 
{
   delay(10);
  
   tbRed.read();
   tbBlack.read();
   tbBlue.read();
   tbYellow.read();

   if (tbRed.changed()) {
    Serial.println("bRed");  
    boxMode = boxMode == 3 ? 0 : boxMode+1;
    char bm = boxMode + '0';
    
    displayWrite(&display, "Boxmode",0,0,1);
    displayWrite(&display,&bm,5,10,0);
 
   }
  
   if (tbBlack.changed()) {
    Serial.println("bBlack");  
   }
  
   if (tbYellow.changed()) {
    Serial.println("bYellow");  

    displayWrite(&display, "Select file",0,0,1);
    
    Serial3.print(0x1);
   }
  
   if (tbBlue.changed()) {
    Serial.println("bBlue");  
    
    displayWrite(&display, "Show file",0,0,1);

    tlx = 0;
    tly = 10;
    
    Serial3.print(0x2);  
   }
  
   if (boxMode == 0)
   {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, HIGH);
   } else if (boxMode == 1)
   {
      digitalWrite(LED_RED, HIGH);
      digitalWrite(LED_GREEN, LOW);  
   } else if (boxMode == 2)
   {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN,HIGH );      
   } else if (boxMode == 3)
   {
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, LOW);     
   }
  
   if (Serial3.available() > 0)
   {
      inByte = Serial3.read();
      processSerialInput(inByte);   
   }

  
  keyPress = customKeypad.getKey();  
  if (keyPress) processKeypress(); 

}
        



  
