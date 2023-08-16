#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>

#if !defined(USBCON) || !defined(USBD_USE_HID_COMPOSITE)

#error "USB HID not enabled! Select 'HID' in the 'Tools->USB interface' menu."

#else
 
class Joystick_ {
private:
public:
	Joystick_(void);
	void begin(void);
	void end(void);
	
	void press(uint8_t key);
	void release();
};
extern Joystick_ Joystick;

#endif
#endif
