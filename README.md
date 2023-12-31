# Arcadinator V1
Arcadinator consists of a breakout board and a dual joystick/gamepad usb composite firmware.  
The breakout board is designed for an STM32F103 Bluepill, an NRF24L01 Board and 26 JST XH connectors.  

# Motivation
Arcadinator is meant to be a cheap and open alternative for [arcade control panel](/extras/img/2L12B.jpg)-to-usb boards like "zero delay".  
Additionally, Arcadinator supports two players and you can optionally use it wirelessly via NRF24L01.

# Hardware
I designed the breakout board with an Altium License from work,  
but I will try to design the next iteration with Kicad.  
There is also an import of the files on https://oshwlab.com/retrobeefhw/arcadinatorv1,  
a backup of the import [here](/hardware/EasyEDA_Backup.zip) 
and you can find the gerber files I sent to JLCPCB [here](/extras/production/gerber.zip)  

# Software
The firmware is still work in progress, but enumeration as dual gamepad usb composite device and hid reports on button presses already work. I also started porting NRF24 code from STM32 HAL to libopencm3.  
Before I decided to use libopencm3, I experimented with stm32duino,  
but could not get a joystick composite device to work quickly.  
You can find a working wireless keyboard experiment [here](/extras/old/ArcadinatorKeyboard/)

# Cloning, Building, Flashing
[libopencm3 prerequisites](https://github.com/libopencm3/libopencm3#prerequisites)  
[OpenOCD](https://openocd.org/pages/getting-openocd.html)
```
git clone --recursive https://github.com/RetroBeef/ArcadinatorV1
cd ArcadinatorV1/firmware
make -C libopencm3 #you need to do this only once
make flash
```

# Licenses
The hardware is licensed under [CERN Open Hardware Licence Version 2 - Strongly Reciprocal](/hardware/LICENSE)  
and the firmware is licensed under [GNU GENERAL PUBLIC LICENSE Version 3](/firmware/LICENSE)  

# Assembled Board
![Assembled Board](extras/img/board.jpg?raw=true "Assembled Board")

# 2L12B Arcade Control Panel
![2L12B](extras/img/2L12B.jpg?raw=true "2L12B Arcade Control Panel")
