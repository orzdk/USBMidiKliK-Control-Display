/* 
   This is the control & display slave for UsbMidiKliK 
   Unit receives I2C data and displays it on 7-segment and 8x8 led matrix displays 
   and sends midiconfigurations back
*/

#include <LiquidCrystal.h> 
#include<Wire_slave.h> 
#include "Keypad.h"

const byte ROWS = 4;
const byte COLS = 4;

char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {3, 2, 1, 0};
byte colPins[COLS] = {7, 6, 5, 4};

Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

const int rs = PB11, en = PB10, d4 = PB0, d5 = PB1, d6 = PC13, d7 = PC14;
LiquidCrystal messagelcd(rs, en, d4, d5, d6, d7); 

int key;
char sMsg;
char keyBuffer[32]; 
int keyBufferPos;
char keyPress;
char commandToSlave[8];

void setup() 
{                     
  Wire.begin(8);  
  Wire.onReceive(receiveEvent);              //Function call when Slave Arduino receives value from master STM32
  Wire.onRequest(requestEvent);              //Function call when Master STM32 request value from Slave Arduino
  
  Serial.begin(9600);
  messagelcd.begin(16, 2);  

  resetBuffer();
}

void requestEvent() /* USBMidiKlik request keybuffer */                           
{
  if (commandToSlave) Wire.write(commandToSlave);                     
}

void receiveEvent (int howMany)  /* Control surface receives display data */             
{
  byte a = Wire.read();                      
  messagelcd.setCursor(0, 0); 
  messagelcd.print(a);
}

void resetBuffer()
{
  memset(keyBuffer, 0, sizeof(keyBuffer));
  keyBufferPos = 0;   
}

int twoByteDecimal(int idx)
{
  return (keyBuffer[idx] - '0') * 10 + (keyBuffer[idx+1] - '0');
}

void processBuffer()
{
  int s_cableserial = twoByteDecimal(0);    
  int s_cableserial_id = twoByteDecimal(2); 
  int t_cableserial = twoByteDecimal(4); 
  int t_cableserial_id = twoByteDecimal(6); 

  uint16_t currentCables = 0xFFFF;
  uint16_t currentJacks = 0xFFFF;
  
  currentCables ^= !t_cableserial * (1UL << t_cableserial_id);
  currentJacks ^= t_cableserial * (1UL << t_cableserial_id);

  char sysex[13] = {
    0xF0, 
    0x77, 
    0x77, 
    0x78, 
    0x0F, 
    0x01,
    s_cableserial,
    s_cableserial_id,
    0xFF,
    currentCables >> 8,
    currentCables & 0xFF,
    currentJacks >> 8,
    currentJacks & 0xFF
  };

  Wire.write(sysex);
  messagelcd.setCursor(0, 1); 
  messagelcd.print(sysex);

}

void loop() 
{
   delay(100);
   keyPress = customKeypad.getKey();
   
   switch (keyPress)
   {
      case NO_KEY:
      break;
      
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
      keyBuffer[keyBufferPos++] = keyPress;
      messagelcd.setCursor(0, 0); 
      messagelcd.print(keyBuffer);
      break;

      case '*':
      processBuffer();
      break;
   }

   
}






  
