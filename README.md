# Arcadinator V1
Arcadinator consists of a breakout board and a dual joystick usb composite firmware.  
The breakout board is designed for an STM32F103, an NRF24L01 and 26 JST XH connectors.  
Arcadinator is meant to be a cheap and open replacement  
for arcade control panel to usb boards like "zero delay usb encoder".  
Additionally, Arcadinator supports two players instead of only one and  
you can optionally use it wirelessly via NRF24L01.

I designed the breakout board with an Altium License from work,  
but I will try to design the next iteration with Kicad.  

You can find the gerber files I sent to JLCPCB [here](/extras/production/gerber.zip)  

The firmware is still work in progress, but dual joystick usb composite functionality already works.  
Before I decided to use libopencm3, I experimented with stm32arduino.  
You can find a working wireless keyboard experiment [here](/extras/old/ArcadinatorKeyboard/)

The hardware is licensed under "CERN Open Hardware Licence Version 2 - Strongly Reciprocal"  
and the firmware is licensed under "GNU GENERAL PUBLIC LICENSE Version 2"  

![Assembled Board](extras/img/board.jpg?raw=true "Assembled Board")

