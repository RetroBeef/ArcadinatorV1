#include <SPI.h>
#include "src/nRF24L01.h"
#include "src/RF24.h"

#include <Keyboard.h>

RF24 radio(PB0, PA4); // CE, CSN on Blue Pill
const uint64_t address = 0xF0F0F0F0E1LL;

#pragma pack(push,1)

//#define SERIAL_DEBUG
#define RXMODE

#define JS1_UP '1'
#define JS1_DOWN '2'
#define JS1_LEFT '3'
#define JS1_RIGHT '4'
#define JS1_B01 '5'
#define JS1_B02 '6'
#define JS1_B03 '7'
#define JS1_B04 '8'
#define JS1_B05 '9'
#define JS1_B06 '0'
#define JS1_BSTART 'q'

#define JS_EXIT 'w'

#define JS2_UP 'e'
#define JS2_DOWN 'r'
#define JS2_LEFT 't'
#define JS2_RIGHT 'z'
#define JS2_B01 'u'
#define JS2_B02 'i'
#define JS2_B03 'o'
#define JS2_B04 'p'
#define JS2_B05 'a'
#define JS2_B06 's'
#define JS2_BSTART 'd'

#define B01 PB12
#define B02 PB13
#define B03 PB14
#define B04 PB15
#define B05 PA8
#define B06 PA9
#define B07 PA10
#define B08 PA15
#define B09 PB3
#define B10 PB4
#define B11 PB5
#define B12 PB6
#define B13 PB11
#define B14 PB10
#define B15 PB1
#define B16 PA3
#define B17 PA2
#define B18 PA1
#define B19 PC15
#define B20 PC14
#define B21 PC13
#define B22 PB7
#define B23 PB8
#define B24 PB9

uint8_t allPins[] = {B01, B02, B03, B04, B05, B06, B07, B08, B09, B10, B11, B12, B13, B14, B15, B16, B17, B18, B19, B20, B21, B22, B23, B24};

typedef struct{
  uint8_t joyUp : 1;
  uint8_t joyDown : 1;
  uint8_t joyLeft : 1;
  uint8_t joyRight : 1;
  uint8_t button01 : 1;
  uint8_t button02 : 1;
  uint8_t button03 : 1;
  uint8_t button04 : 1;
  uint8_t button05 : 1;
  uint8_t button06 : 1;
  uint8_t buttonStart : 1;
} PlayerData_s;

typedef struct{
  PlayerData_s player1;
  PlayerData_s player2;
  uint8_t exitButton : 1;
} PanelData_s;

typedef union{
  PanelData_s structured;
  uint8_t bytes[sizeof(structured)];
} PanelData_t;

#pragma pack(pop)

PanelData_t panelState = {0};
PanelData_t lastPanelState = {0};

#if defined(SERIAL_DEBUG)
void printPlayerData(PlayerData_s* playerData){
  Serial.printf("jUp(%u),jDown(%u),jLeft(%u),jRight(%u),b01(%u), b02(%u), b03(%u), b04(%u), b05(%u), b06(%u), start(%u)\r\n",playerData->joyUp,playerData->joyDown,playerData->joyLeft,playerData->joyRight,playerData->button01,playerData->button02,playerData->button03,playerData->button04,playerData->button05,playerData->button06,playerData->buttonStart);
}

void printPanelData(PanelData_s* panelData){
  Serial.println("ControlPanelData:");
  Serial.print("Player 1: ");
  printPlayerData(&panelData->player1);
  Serial.print("Player 2: ");
  printPlayerData(&panelData->player2);
  Serial.printf("exitButton(%u)\r\n\r\n", panelData->exitButton);
}
#endif

void setupInput(){
  for(uint8_t pin : allPins){
    pinMode(pin, INPUT_PULLUP);
  }
}

void setup() {
#if defined(SERIAL_DEBUG)
  Serial.begin(115200);
#endif
#if defined(RXMODE)
  Keyboard.begin();
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
#else
  setupInput();
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
#endif
}

void input(){
  panelState.structured.player1.joyUp = !digitalRead(B01);
  panelState.structured.player1.joyDown = !digitalRead(B02);
  panelState.structured.player1.joyLeft = !digitalRead(B03);
  panelState.structured.player1.joyRight = !digitalRead(B04);
  panelState.structured.player1.button01 = !digitalRead(B05);
  panelState.structured.player1.button02 = !digitalRead(B06);
  panelState.structured.player1.button03 = !digitalRead(B07);
  panelState.structured.player1.button04 = !digitalRead(B08);
  panelState.structured.player1.button05 = !digitalRead(B09);
  panelState.structured.player1.button06 = !digitalRead(B10);
  panelState.structured.player1.buttonStart = !digitalRead(B11);

  panelState.structured.player2.joyUp = !digitalRead(B12);
  panelState.structured.player2.joyDown = !digitalRead(B13);
  panelState.structured.player2.joyLeft = !digitalRead(B14);
  panelState.structured.player2.joyRight = !digitalRead(B15);
  panelState.structured.player2.button01 = !digitalRead(B16);
  panelState.structured.player2.button02 = !digitalRead(B17);
  panelState.structured.player2.button03 = !digitalRead(B18);
  panelState.structured.player2.button04 = !digitalRead(B19);
  panelState.structured.player2.button05 = !digitalRead(B20);
  panelState.structured.player2.button06 = !digitalRead(B21);
  panelState.structured.player2.buttonStart = !digitalRead(B22);

  panelState.structured.exitButton = !digitalRead(B23);
}

void tx(){
  radio.write(panelState.bytes, sizeof(PanelData_t));
}

uint32_t lastRadioCommMs = 0;
const uint32_t lastRadioCommTimeoutMs = 500;

void rx(){
  if (radio.available()){
    lastRadioCommMs = millis();
    radio.read(panelState.bytes, sizeof(PanelData_t));
#if defined(SERIAL_DEBUG)
    printPanelData(&panelState.structured);
#endif
    if(panelState.structured.player1.joyUp != lastPanelState.structured.player1.joyUp){
      if(panelState.structured.player1.joyUp){
        Keyboard.press(JS1_UP);
      }else{
        Keyboard.release(JS1_UP);
      }
    }
    if(panelState.structured.player1.joyDown != lastPanelState.structured.player1.joyDown){
      if(panelState.structured.player1.joyDown){
        Keyboard.press(JS1_DOWN);
      }else{
        Keyboard.release(JS1_DOWN);
      }
    }
    if(panelState.structured.player1.joyLeft != lastPanelState.structured.player1.joyLeft){
      if(panelState.structured.player1.joyLeft){
        Keyboard.press(JS1_LEFT);
      }else{
        Keyboard.release(JS1_LEFT);
      }
    }
    if(panelState.structured.player1.joyRight != lastPanelState.structured.player1.joyRight){
      if(panelState.structured.player1.joyRight){
        Keyboard.press(JS1_RIGHT);
      }else{
        Keyboard.release(JS1_RIGHT);
      }
    }
    if(panelState.structured.player1.button01 != lastPanelState.structured.player1.button01){
      if(panelState.structured.player1.button01){
        Keyboard.press(JS1_B01);
      }else{
        Keyboard.release(JS1_B01);
      }
    }
    if(panelState.structured.player1.button02 != lastPanelState.structured.player1.button02){
      if(panelState.structured.player1.button02){
        Keyboard.press(JS1_B02);
      }else{
        Keyboard.release(JS1_B02);
      }
    }
    if(panelState.structured.player1.button03 != lastPanelState.structured.player1.button03){
      if(panelState.structured.player1.button03){
        Keyboard.press(JS1_B03);
      }else{
        Keyboard.release(JS1_B03);
      }
    }
    if(panelState.structured.player1.button04 != lastPanelState.structured.player1.button04){
      if(panelState.structured.player1.button04){
        Keyboard.press(JS1_B04);
      }else{
        Keyboard.release(JS1_B04);
      }
    }
    if(panelState.structured.player1.button05 != lastPanelState.structured.player1.button05){
      if(panelState.structured.player1.button05){
        Keyboard.press(JS1_B05);
      }else{
        Keyboard.release(JS1_B05);
      }
    }
    if(panelState.structured.player1.button06 != lastPanelState.structured.player1.button06){
      if(panelState.structured.player1.button06){
        Keyboard.press(JS1_B06);
      }else{
        Keyboard.release(JS1_B06);
      }
    }
    if(panelState.structured.player1.buttonStart != lastPanelState.structured.player1.buttonStart){
      if(panelState.structured.player1.buttonStart){
        Keyboard.press(JS1_BSTART);
      }else{
        Keyboard.release(JS1_BSTART);
      }
    }

    if(panelState.structured.player2.joyUp != lastPanelState.structured.player2.joyUp){
      if(panelState.structured.player2.joyUp){
        Keyboard.press(JS2_UP);
      }else{
        Keyboard.release(JS2_UP);
      }
    }
    if(panelState.structured.player2.joyDown != lastPanelState.structured.player2.joyDown){
      if(panelState.structured.player2.joyDown){
        Keyboard.press(JS2_DOWN);
      }else{
        Keyboard.release(JS2_DOWN);
      }
    }
    if(panelState.structured.player2.joyLeft != lastPanelState.structured.player2.joyLeft){
      if(panelState.structured.player2.joyLeft){
        Keyboard.press(JS2_LEFT);
      }else{
        Keyboard.release(JS2_LEFT);
      }
    }
    if(panelState.structured.player2.joyRight != lastPanelState.structured.player2.joyRight){
      if(panelState.structured.player2.joyRight){
        Keyboard.press(JS2_RIGHT);
      }else{
        Keyboard.release(JS2_RIGHT);
      }
    }
    if(panelState.structured.player2.button01 != lastPanelState.structured.player2.button01){
      if(panelState.structured.player2.button01){
        Keyboard.press(JS2_B01);
      }else{
        Keyboard.release(JS2_B01);
      }
    }
    if(panelState.structured.player2.button02 != lastPanelState.structured.player2.button02){
      if(panelState.structured.player2.button02){
        Keyboard.press(JS2_B02);
      }else{
        Keyboard.release(JS2_B02);
      }
    }
    if(panelState.structured.player2.button03 != lastPanelState.structured.player2.button03){
      if(panelState.structured.player2.button03){
        Keyboard.press(JS2_B03);
      }else{
        Keyboard.release(JS2_B03);
      }
    }
    if(panelState.structured.player2.button04 != lastPanelState.structured.player2.button04){
      if(panelState.structured.player2.button04){
        Keyboard.press(JS2_B04);
      }else{
        Keyboard.release(JS2_B04);
      }
    }
    if(panelState.structured.player2.button05 != lastPanelState.structured.player2.button05){
      if(panelState.structured.player2.button05){
        Keyboard.press(JS2_B05);
      }else{
        Keyboard.release(JS2_B05);
      }
    }
    if(panelState.structured.player2.button06 != lastPanelState.structured.player2.button06){
      if(panelState.structured.player2.button06){
        Keyboard.press(JS2_B06);
      }else{
        Keyboard.release(JS2_B06);
      }
    }
    if(panelState.structured.player2.buttonStart != lastPanelState.structured.player2.buttonStart){
      if(panelState.structured.player2.buttonStart){
        Keyboard.press(JS2_BSTART);
      }else{
        Keyboard.release(JS2_BSTART);
      }
    }

    memcpy(&lastPanelState, &panelState, sizeof(PanelData_t));
  }else if(lastRadioCommMs){
    if(millis()-lastRadioCommMs > lastRadioCommTimeoutMs){
      lastRadioCommMs = 0;
      Keyboard.releaseAll();
    }
  }
}

void loop(){
#if defined(RXMODE)
  rx();
#else
  input();
  tx();
#endif
}
