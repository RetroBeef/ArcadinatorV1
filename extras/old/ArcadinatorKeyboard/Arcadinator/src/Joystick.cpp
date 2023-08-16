#include "Joystick.h"

#if defined(USBCON)
#include "usbd_hid_consumer_if.h"

uint8_t m[3] = {0x02, 0x00, 0x00 };

Joystick_::Joystick_(void) {
}
 
void Joystick_::begin(void) {
	HID_Composite_Init(HID_CONSUMER);
}
 
void Joystick_::end(void) {
	HID_Composite_DeInit(HID_CONSUMER);
}

void Joystick_::press(uint8_t mediaKey) {
	m[1] = mediaKey;
	HID_Composite_consumer_sendReport(m, 3);
}

void Joystick_::release() {
	delay(10);
	m[1] = 0x00;
	HID_Composite_consumer_sendReport(m, 3);
}

Joystick_ Joystick;

#endif
