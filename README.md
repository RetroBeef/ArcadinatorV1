# Arcadinator V1
Arcadinator consists of a breakout board and a dual joystick usb composite firmware.  
The breakout board is designed for an STM32F103, an NRF24L01 and 26 JST XH connectors.  
Arcadinator is meant to be a cheap and open replacement to  
for arcade control panels like [this](/extras/img/2L12B.jpg) to usb boards like "zero delay".  
Additionally, Arcadinator supports two players and you can optionally use it wirelessly via NRF24L01.

I designed the breakout board with an Altium License from work,  
but I will try to design the next iteration with Kicad.  
There is also an import of the files on https://oshwlab.com/retrobeefhw/arcadinatorv1
and a backup of the import [here](/hardware/EasyEDA_Backup.zip)

You can find the gerber files I sent to JLCPCB [here](/extras/production/gerber.zip)  

The firmware is still work in progress, but enumeration as dual gamepad usb composite device already works  
and I started porting NRF24 code from STM32 HAL to libopencm3  
Before I decided to use libopencm3, I experimented with stm32arduino,  
but could not get a joystick composite device to work  
You can find a working wireless keyboard experiment [here](/extras/old/ArcadinatorKeyboard/)

The hardware is licensed under "CERN Open Hardware Licence Version 2 - Strongly Reciprocal"  
and the firmware is licensed under "GNU GENERAL PUBLIC LICENSE Version 2"  

![Assembled Board](extras/img/board.jpg?raw=true "Assembled Board")

