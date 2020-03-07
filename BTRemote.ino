
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
#include "BTSerial.h"

/**********************************
 * Global Variables / Class inits
 *********************************/
#define BT_TX 11
#define BT_RX 13
#define BT_EN 2
#define JOYSWITCH 3
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "i2c_MMA8451.h"

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
static Message_T Msg;

int16_t JoyRefX;
int16_t JoyRefY;

MMA8451 mma8451; 
static long int Time;

#define UPDATEINTERVAL 50
#define NUMREADINGS 3
static int16_t Readings[5][NUMREADINGS+1]={0};//X,Y,Z,JoyX,JoyY
static uint8_t ReadIndex;

void setup() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  ShowSplash();
  analogReference(DEFAULT);
  delay(150);
  for (int i=0;i<NUMREADINGS*2;i++){
    ComposeMessage();
  }
  JoyRefX=Msg.JoyX;
  JoyRefY=Msg.JoyY;
  pinMode(BT_EN,OUTPUT);  
  pinMode(JOYSWITCH,INPUT_PULLUP);   
  // Open serial communications and wait for port to open:
  Serial.begin(UARTFAST);
  BSerial.begin(BAUDBTCOM);
  BSerial.setTimeout(2);
  Serial.setTimeout(2);
  Serial.println(F("Welcome to Bluetooth Remote!"));
//  if (UpdatesEnabled()){
//    Serial.print(F("EEPROM Writes Enabled. Send PROTECT to switch off\n"));
//  }else{
//    PrintUpdateMessage();
//  }
//  FlexiTimer2::start();
//  PrintHelp();
  Serial.print("Sketch:   ");   Serial.println(F(__FILE__));
  Serial.print("Uploaded: ");   Serial.println(F(__DATE__));
  if (mma8451.initialize()){
    Serial.println(F("MMA8451 Sensor found!"));
    display.println(F("MMA8451 Sensor found!"));
    display.display();
  }
  delay(2000);
  Serial.println(F("Ready for Command:"));
  Time=millis();
}

void loop() {
  char input[INPUT_SIZE+1];
  delay(millis()-Time+UPDATEINTERVAL);
  for (int i = 0; i < sizeof(input);i++){
    input[i]=0;
  }
  ComposeMessage();
  SendMessage();
  Time=millis();
  MessageToDisplay();
  delay(UPDATEINTERVAL/2);
  byte rxlen=ReceiveUSBSerial(input);
  //others
  if (rxlen>0){
    HandleUSBSerial(input, rxlen);
  }
  String RX = "Timeout";
  if (BSerial.available()){
    delay(2);
    RX = BSerial.readString();
  }
  display.print(RX);
  display.display();
}

byte ReceiveUSBSerial(char * input){
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
//  if (rxlen>0){
//    printHEX(rxlen,input);
//  }
  return rxlen;
}

void HandleUSBSerial(char * input, byte rxlen){
  if (rxlen==0)return;
  switch (input[0]){
    case 'A': USB_BT_Transparent(BT_CONFIG,BAUDBTCONF); break;
    case 'a': USB_BT_Transparent(BT_COM,BAUDBTCOM); break;
    case 'S': USB_BT_Transparent(BT_CONFIG,9600); break;
    case 'X': break;
//    case 'U': if ((input[1]=='N')&&(input[2]=='P')&&(input[3]=='R')&&(input[4]=='O')&&(input[5]=='T')){
//                Serial.println(input);
//                EnableUpdates();
//              };
//              break;
//    case 'P': if ((input[1]=='R')&&(input[2]=='O')&&(input[3]=='T')&&(input[4]=='E')&&(input[5]=='C')&&(input[6]=='T')){
//                Serial.println(input);
//                DisableUpdates();};break;
//    case '^': asm volatile ("  jmp 0");break;
  }
}

void ShowSplash(){
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font  
  display.println(F("BTRemote"));
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.println(F(__DATE__));
  display.display();
}

void SendMessage(){
  for (int i = 0;i<sizeof(Msg);i++){
    BSerial.write (((uint8_t*)&Msg)[i]);
  }
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

int16_t MovingAverage(int16_t * Readings, int16_t Value){
  Readings[NUMREADINGS]-=Readings[ReadIndex];//substract oldest
  Readings[ReadIndex]=Value;
  Readings[NUMREADINGS]+=Readings[ReadIndex];
  return Readings[NUMREADINGS]/NUMREADINGS;
}

void ComposeMessage(){
  Msg.Ident='%';
  Msg.JoyX=MovingAverage(Readings[3],analogRead(A6)-JoyRefX);
  Msg.JoyY=MovingAverage(Readings[4],analogRead(A7)-JoyRefY);
  long int S=sq((long int)Msg.JoyX)+sq((long int)Msg.JoyY);//Radius 
  if (S>261121){S=261121;}//511^2
  Msg.JoySpeed=sqrt(S);
  Msg.JoyAngle=atan2(Msg.JoyX,Msg.JoyY)*5729.779;//1/100 Grad
  if (Msg.JoySpeed<3) Msg.JoyAngle=0;//filter LSB Angle changes
  int16_t xyz_g[3]; 
  mma8451.getMeasurement(xyz_g);
  Msg.GyroX=MovingAverage(Readings[0],xyz_g[0]);
  Msg.GyroY=MovingAverage(Readings[1],xyz_g[1]);
  Msg.GyroZ=MovingAverage(Readings[2],xyz_g[2]);
  long int G=sqrt((long int)Msg.GyroX*(long int)Msg.GyroX+(long int)Msg.GyroY*(long int)Msg.GyroY+(long int)Msg.GyroZ*(long int)Msg.GyroZ);
  Msg.GyroG=G;
  Msg.Switches=digitalRead(3);

  ReadIndex++;
  if (ReadIndex==NUMREADINGS)ReadIndex=0;
  
  Msg.LRC=0;
  for (int i = 0;i<sizeof(Msg)-1;i++){
    uint8_t b = ((uint8_t*)&Msg)[i];
    Msg.LRC+=b;
  }
}

void MessageToDisplay(){
  display.clearDisplay();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  char Gyro[50];
  sprintf(Gyro,"X%+5dY%+5dZ%+5d S%d\nx%+4dy%+4d S=%4d\nJ:%+4d^ g:%3d ",
                Msg.GyroX,Msg.GyroY,Msg.GyroZ,Msg.Switches,
                Msg.JoyX,Msg.JoyY,Msg.JoySpeed,Msg.JoyAngle/100,Msg.GyroG/111);
  display.println(Gyro);
//  sprintf(Gyro,"Grav:%4d TX:",Msg.GyroG);
//  display.print(Gyro);
//  displayHEX(sizeof(Msg),(char*)&Msg);
}

//void printHEX(uint8_t len, char *input){
//    Serial.print(F("Input Data:"));
//    for (uint8_t i=0;i<len;i++){
//      Serial.write(input[i]);
//    }
//    Serial.print(F("\nin Hex:"));
//    for (uint8_t i=0;i<len;i++){
//      Serial.write(' ');
//      if (input[i]<16) Serial.print('0');
//      Serial.print(input[i],HEX);
//    }
//    Serial.write('\n');
//}
//
//void displayHEX(uint8_t len, char *input){
//    for (uint8_t i=0;i<len;i++){
////      display.write('');
//      if ((uint8_t)input[i]<16) display.print('0');
//      display.print((uint8_t)input[i],HEX);
//    }
//    Serial.write(';');
//}
//#define MAGICEEPROMUPDATE_ENABLED 0x1337
//bool UpdatesEnabled(){
//  uint16_t Magic;
//  EEPROM.get((int)&NVM->EEPROMUpdatesDisabled, Magic);
//  return Magic==MAGICEEPROMUPDATE_ENABLED;
//}
//void EnableUpdates(){
//    EEPROM.put((int)&NVM->EEPROMUpdatesDisabled,(uint16_t)MAGICEEPROMUPDATE_ENABLED);
//}
//void DisableUpdates(){
//    EEPROM.put((int)&NVM->EEPROMUpdatesDisabled,(uint16_t)!MAGICEEPROMUPDATE_ENABLED);
//}
//void PrintUpdateMessage(){
//    Serial.println(F("Updates are Disabled, send UNPROTECT to activate"));
//}
//
