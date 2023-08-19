#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#pragma pack(push,1)
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

typedef union{
  PlayerData_s obj;
  uint8_t bytes[sizeof(PlayerData_s)];
} PlayerData_t;


typedef struct{
  PlayerData_t player1;
  PlayerData_t player2;
  uint8_t exitButton : 1;
} PanelData_s;

typedef union{
  PanelData_s obj;
  uint8_t bytes[sizeof(PanelData_s)];
} PanelData_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif
