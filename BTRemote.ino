
/*************************************************** 
  This is the Code for Arduino Spider with PCA9685 PWM controller.
  3D Design Data and Buildup Guide is found here:
  https://www.thingiverse.com/thing:4070234
  The Spider uses pins see chapter pinmapping below

  BSD license, all text above must be included in any redistribution
 ****************************************************/
#define ENABLE_DEBUG_OUTPUT
#include <Wire.h>
#define ENABLE_DEBUG_OUTPUT
#include <EEPROM.h>
#include <FlexiTimer2.h>//to set a timer to manage all servos
#include <SoftwareSerial.h>
#include "Spider_Hardware.h"
#include "MoveCore.h"


/**********************************
 * Global Variables / Class inits
 *********************************/
#define BT_TX 11
#define BT_RX 13
#define BT_EN 2
#define JOYSWITCH 3
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111111, B11111000,
  B01110011, B10011111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001111, B11110000,
  B00011011, B10100000,
  B00111100, B01100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

SoftwareSerial BSerial(BT_TX, BT_RX); // RX, TX

#define UARTFAST 57600
#define BAUDBTCONF 38400
#define BAUDBTDEFAULT 9600
#define BAUDBTCOM 57600
#define INPUT_SIZE 32
#define DEBUG_COMMANDS 1

bool EEPROMUPDATESENABLED = false;

//Structure to store servo related calibration data
typedef struct {
  uint8_t BTName[32];
  uint8_t EEPROMUpdatesDisabled;
} EEPROM_T;
EEPROM_T * NVM=0;

/**********************************
 * Global Variables / Class inits
 *********************************/

typedef struct {
  int16_t X;
  int16_t Y;
  int16_t Z;
} SENSOR_T;
SENSOR_T JoyRef;
SENSOR_T Joy;

void setup() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  testdrawchar();      // Draw characters of the default font
  JoyRef.X=analogRead(A6);
  JoyRef.Y=analogRead(A7);
  pinMode(BT_EN,OUTPUT);  
  pinMode(JOYSWITCH,INPUT_PULLUP);
  // Open serial communications and wait for port to open:
  Serial.begin(UARTFAST);
  BSerial.begin(BAUDBTCOM);
  BSerial.setTimeout(2);
  Serial.setTimeout(2);
  Serial.println(F("Welcome to Bluetooth Remote!"));
  if (UpdatesEnabled()){
    Serial.print(F("EEPROM Writes Enabled. Send PROTECT to switch off\n"));
  }else{
    PrintUpdateMessage();
  }
//  FlexiTimer2::start();
//  PrintHelp();
  Serial.print("Sketch:   ");   Serial.println(F(__FILE__));
  Serial.print("Uploaded: ");   Serial.println(F(__DATE__));
  Serial.println(F("Ready for Command:"));
}

#define MAGICEEPROMUPDATE_ENABLED 0x1337
bool UpdatesEnabled(){
  uint16_t Magic;
  EEPROM.get((int)&NVM->EEPROMUpdatesDisabled, Magic);
  return Magic==MAGICEEPROMUPDATE_ENABLED;
}
void EnableUpdates(){
    EEPROM.put((int)&NVM->EEPROMUpdatesDisabled,(uint16_t)MAGICEEPROMUPDATE_ENABLED);
}
void DisableUpdates(){
    EEPROM.put((int)&NVM->EEPROMUpdatesDisabled,(uint16_t)!MAGICEEPROMUPDATE_ENABLED);
}
void PrintUpdateMessage(){
    Serial.println(F("Updates are Disabled, send UNPROTECT to activate"));
}

void printHEX(uint8_t len, char *input){
    Serial.print(F("Input Data:"));
    for (uint8_t i=0;i<len;i++){
      Serial.write(input[i]);
    }
    Serial.print(F("\nin Hex:"));
    for (uint8_t i=0;i<len;i++){
      Serial.write(' ');
      if (input[i]<16) Serial.print('0');
      Serial.print(input[i],HEX);
    }
    Serial.write('\n');
}

void USB_BT_Transparent(uint8_t Mode, long Speed){
  uint8_t x;
  BSerial.end();
  digitalWrite(BT_EN,Mode);
  Serial.print(F("Bluetooth Terminal:EN="));
  Serial.print(Mode);
  BSerial.begin(Speed);
  Serial.println(Speed);
  while(1){
    // Keep reading from HC-05 and send to Arduino Serial Monitor
    if (BSerial.available())
      Serial.write(BSerial.read());
  
    // Keep reading from Arduino Serial Monitor and send to HC-05
    if (Serial.available()){
      x=Serial.read();
      if (x=='^')break;
      if (x=='@'){
        Serial.print(F("Setting EN to "));
        digitalWrite(BT_EN,!digitalRead(BT_EN));
        delay(100);
        Serial.println(digitalRead(BT_EN));
      }else{
        BSerial.write(x);
      }
    }
  }
  Serial.println(F("Transparent Mode terminated"));
}

void loop() {
  char input[INPUT_SIZE+1];
  char cmd;
  uint8_t ServoID;
  int8_t RX_X;
  int8_t RX_Y;
  uint16_t Value;
  String inp;
  bool Fast;

  for (int i = 0; i < sizeof(input);i++){
    input[i]=0;
  }

  Joy.X=analogRead(A6)-JoyRef.X;
  Joy.Y=analogRead(A7)-JoyRef.Y;
  
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  display.print(F("BTRemote "));
  display.println(F(__DATE__));
  display.print((int)Joy.X);
  display.print(F(" "));
  display.print((int)Joy.Y);
  display.print(F(" "));
  display.print(digitalRead(3));
  display.print(F(" "));
  display.print(analogRead(A6));
  display.print(F(" "));
  display.println(analogRead(A7));

  Fast=((Joy.X>400)||(Joy.X<-400)||(Joy.Y>400)||(Joy.Y<-400));
  if (Fast)
    display.print(F("Fast "));
  if (Joy.Y>60) {
    display.print(F("Forward"));
    if (Fast){
      TransmitWaitReady("FF");
    }else{
      TransmitWaitReady("ff");
      }
  }
  if (Joy.Y<-60){
    display.print(F("Reverse"));
    if (Fast){
      TransmitWaitReady("PP");
    }else{
      TransmitWaitReady("pp");
    }
  }
  display.print(F(" "));
  if (Joy.X>100){
    display.print(F("Right"));
    if (Fast){
      TransmitWaitReady("LL");
    }else{
      TransmitWaitReady("ll");
    }
  }
  if (Joy.X<-100){
    display.print(F("Left"));
    if (Fast){
      TransmitWaitReady("MM");
    }else{
      TransmitWaitReady("mm");
    }
  }
  
  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
//  for(int16_t i=0;  i<256; i++) {
//    if(i == '\n') display.write(' ');
//    else          display.write(i);
//  }

  display.display();

  byte rxlen=0;
  while(Serial.available()){
    delay(2);
    uint8_t rc=Serial.read();
    if (rc==' '||(rc>43)||(rc<58)||(rc>64)||(rc<123)) //valid characters: a-zA-Z,.-0-9
    { 
      input[rxlen++]=rc;
    }else{
      input[rxlen]=0;
      Serial.end();
      break;
    }
  }
  if (rxlen>0){
    printHEX(rxlen,input);
  }
  //others
  if (rxlen>0){
    ServoID=15;
    Value=65535;
    char* command = strtok(input, " ;\n");
    cmd=command[0];
    char* separator= strtok(NULL,' ');
    if (separator != NULL) ServoID=atoi(separator);
    ++separator;
    Value=atoi(++separator);
    Serial.print(F("Command:"));Serial.print(cmd);
    Serial.print(F(",ServoID:"));Serial.print(ServoID);
    Serial.print(F(",Value:"));Serial.println(Value);
    switch (cmd){
      case 'A': USB_BT_Transparent(BT_CONFIG,BAUDBTCONF); break;
      case 'a': USB_BT_Transparent(BT_COM,BAUDBTCOM); break;
      case 'S': USB_BT_Transparent(BT_CONFIG,9600); break;
      case 'X': break;
      case 'U': if ((input[1]=='N')&&(input[2]=='P')&&(input[3]=='R')&&(input[4]=='O')&&(input[5]=='T')){
                  Serial.println(input);
                  EnableUpdates();
                };
                break;
      case 'P': if ((input[1]=='R')&&(input[2]=='O')&&(input[3]=='T')&&(input[4]=='E')&&(input[5]=='C')&&(input[6]=='T')){
                  Serial.println(input);
                  DisableUpdates();};break;
      case '^': asm volatile ("  jmp 0");break;
    }
    Serial.println("Done");
    command = strtok(0, 10);
  }
}

void TransmitWaitReady(String TX){
  Serial.print(TX);
  BSerial.print(TX);
  display.print("TX:");
  display.print(TX);
  //Receive some bytes, check if it contains °RY=EndMarker
  //Timeout after 2000ms
  unsigned long time;
  bool EndMarker=false;
//  time=millis();
  delay(50);
//  if (BSerial.available()){//flush intial message
//    BSerial.read();
//  }
//  while(1){
//    if (BSerial.available())
//    {
//      if (BSerial.read()=='°')//end marker candidate
//        if (BSerial.read()=='R')
//          if (BSerial.read()=='Y')
//          {
//            EndMarker=true;
//            break;//End marker found from Robot
//          }
//    }
//    if ((millis()-time)>5000) break;//timeout
//  }
//  if (EndMarker){
//    display.print(F("Ready"));
//  }else{
//    display.print(F("Timeout"));
//  }         
}

void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
  delay(2000);
}
