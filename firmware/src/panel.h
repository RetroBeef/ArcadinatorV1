#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

typedef enum{
    SINGLE_PANEL = 0,
	DUAL_PANEL = 1
} panelType_t;

#define HATSWITCH_UP            0x00
#define HATSWITCH_UPRIGHT       0x01
#define HATSWITCH_RIGHT         0x02
#define HATSWITCH_DOWNRIGHT     0x03
#define HATSWITCH_DOWN          0x04
#define HATSWITCH_DOWNLEFT      0x05
#define HATSWITCH_LEFT          0x06
#define HATSWITCH_UPLEFT        0x07
#define HATSWITCH_NONE          0x0F

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
  uint8_t buttonExtra : 1;
  uint8_t reservedA : 1;
  uint8_t reservedB : 1;
  uint8_t reservedC : 1;
  uint8_t reservedD : 1;
} PlayerData_s;

typedef union{
  PlayerData_s obj;
  uint8_t bytes[sizeof(PlayerData_s)];
} PlayerData_t;


typedef struct{
  PlayerData_t player1;
  PlayerData_t player2;
} PanelData_s;

typedef union{
  PanelData_s obj;
  uint8_t bytes[sizeof(PanelData_s)];
} PanelData_t;

typedef struct{
	uint8_t y : 1;
	uint8_t b : 1;
	uint8_t a : 1;
	uint8_t x : 1;
	uint8_t l : 1;
	uint8_t r : 1;
	uint8_t zl : 1;
	uint8_t zr : 1;
	uint8_t minus : 1;
	uint8_t plus : 1;
	uint8_t leftClick : 1;
	uint8_t rightClick : 1;
	uint8_t home : 1;
	uint8_t capture : 1;
	uint8_t reserved1: 2;
	uint8_t hat;
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
    uint8_t reserved3;
} PlayerDataNX_s;

typedef union{
  PlayerDataNX_s obj;
  uint8_t bytes[sizeof(PlayerDataNX_s)];
} PlayerDataNX_t;


typedef struct{
  PlayerDataNX_t player1;
  PlayerDataNX_t player2;
} PanelDataNX_s;

typedef union{
  PanelDataNX_s obj;
  uint8_t bytes[sizeof(PanelDataNX_s)];
} PanelDataNX_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif
