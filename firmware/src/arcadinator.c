#include <stdlib.h>
#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

#include "nrf24l01.h"

uint8_t rxAddress[NRF24L01_ADDR_WIDTH] = {0x32,0x4E,0x6F,0x64,0x65};
uint8_t txAddress[NRF24L01_ADDR_WIDTH] = {0x11,0x22,0x33,0x44,0x55};

typedef enum{
    RX_MODE = 0,
    TX_MODE = 1
} xMode_t;

uint8_t xMode = TX_MODE;
extern uint8_t RX_BUF[];
extern uint8_t TX_BUF[];


#define PACKET_SIZE 14

static usbd_device *usbd_dev;

const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x1209,
	.idProduct = 0xcade,//todo: submit to https://pid.codes/
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
  0x05, 0x01, // USAGE_PAGE (Generic Desktop)
  0x09, 0x08, //JOYSTICK_TYPE_MULTI_AXIS, // USAGE (Multi axis)
  0xA1, 0x01, // COLLECTION (Application)
	//================================Input Report======================================//
  	// WheelReport
  	0x85, 0x03, //JOYSTICK_DEFAULT_REPORT_ID, // REPORT_ID (default 3)
  	//0xA1, 0x00, // COLLECTION (Physical)
  	0x05, 0x09, // USAGE_PAGE  (Button)
  	0x19, 0x01, // USAGE_MINIMUM (Button 1)
  	0x29, 0x08, // USAGE_MAXIMUM (Button 8)
  	0x15, 0x00, // LOGICAL_MINIMUM (0)
  	0x25, 0x01, // LOGICAL_MAXIMUM (1)
  	0x75, 0x01, // REPORT_SIZE (1)
  	0x95, 0x08, // REPORT_COUNT (8)
  	0x55, 0x00, // UNIT_EXPONENT (0)
  	0x65, 0x00, // UNIT (None)
  	0x81, 0x02, //INPUT (Data,Var,Abs)

  	//3 Axis: X, Y, Z
  	0x05, 0x01, // USAGE_PAGE (Generic Desktop)
  	0x09, 0x01, // USAGE (Pointer)
  	0x16, 0x01, 0x80, //LOGICAL_MINIMUM (-32768)
  	0x26, 0xFF, 0x7F, //LOGICAL_MAXIMUM (32767)
  	0x75, 0x10, // REPORT_SIZE (16)
  	0x95, 0x03, // REPORT_COUNT (3)
  	0xA1, 0x00, // COLLECTION (Physical)
  		0x09, 0x30, // USAGE (X)
  		0x09, 0x31, // USAGE (Y)
  		0x09, 0x32, // USAGE (Z)
  		0x81, 0x02, // INPUT (Data,Var,Abs)
  	0xc0, // END_COLLECTION (Physical)

  	//simulation 3 Axis
  	0x05, 0x02, // USAGE_PAGE (Simulation Controls)
  	0x16, 0x01, 0x80, //LOGICAL_MINIMUM (-32768)
  	0x26, 0xFF, 0x7F, //LOGICAL_MAXIMUM (32767)
  	0x75, 0x10, // REPORT_SIZE (16)
  	0x95, 0x03, // REPORT_COUNT (3)
  	0xA1, 0x00, // COLLECTION (Physical)
  		0x09, 0xC4, // USAGE (Accelerator)
  		0x09, 0xC5, // USAGE (Brake)
  		0x09, 0xC8, // USAGE (Steering)
  		0x81, 0x02, // INPUT (Data,Var,Abs)
  	0xc0, // END_COLLECTION 

  0xC0, // END COLLECTION ()
};

static const uint8_t hid_report_descriptor2[] = {
  0x05, 0x01, // USAGE_PAGE (Generic Desktop)
  0x09, 0x08, //JOYSTICK_TYPE_MULTI_AXIS, // USAGE (Multi axis)
  0xA1, 0x01, // COLLECTION (Application)
	//================================Input Report======================================//
  	// WheelReport
  	0x85, 0x03, //JOYSTICK_DEFAULT_REPORT_ID, // REPORT_ID (default 3)
  	//0xA1, 0x00, // COLLECTION (Physical)
  	0x05, 0x09, // USAGE_PAGE  (Button)
  	0x19, 0x01, // USAGE_MINIMUM (Button 1)
  	0x29, 0x08, // USAGE_MAXIMUM (Button 8)
  	0x15, 0x00, // LOGICAL_MINIMUM (0)
  	0x25, 0x01, // LOGICAL_MAXIMUM (1)
  	0x75, 0x01, // REPORT_SIZE (1)
  	0x95, 0x08, // REPORT_COUNT (8)
  	0x55, 0x00, // UNIT_EXPONENT (0)
  	0x65, 0x00, // UNIT (None)
  	0x81, 0x02, //INPUT (Data,Var,Abs)

  	//3 Axis: X, Y, Z
  	0x05, 0x01, // USAGE_PAGE (Generic Desktop)
  	0x09, 0x01, // USAGE (Pointer)
  	0x16, 0x01, 0x80, //LOGICAL_MINIMUM (-32768)
  	0x26, 0xFF, 0x7F, //LOGICAL_MAXIMUM (32767)
  	0x75, 0x10, // REPORT_SIZE (16)
  	0x95, 0x03, // REPORT_COUNT (3)
  	0xA1, 0x00, // COLLECTION (Physical)
  		0x09, 0x30, // USAGE (X)
  		0x09, 0x31, // USAGE (Y)
  		0x09, 0x32, // USAGE (Z)
  		0x81, 0x02, // INPUT (Data,Var,Abs)
  	0xc0, // END_COLLECTION (Physical)

  0xC0, // END COLLECTION ()
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
	.hid_descriptor = {
		.bLength = sizeof(hid_function),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	}
};

const struct usb_endpoint_descriptor hid_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = PACKET_SIZE,
	.bInterval = 0x20,
};

const struct usb_endpoint_descriptor hid_endpoint2 = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = PACKET_SIZE,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 2, /* mouse */
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_interface_descriptor hid_iface2 = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 2, /* mouse */
	.iInterface = 0,

	.endpoint = &hid_endpoint2,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}, {
	.num_altsetting = 1,
	.altsetting = &hid_iface2,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 2,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"RetroBeef Arcadinator",
	"DualJoystiX",
	"V1"
};

// Buffer to be used for control requests.
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void (**complete)(usbd_device *, struct usb_setup_data *)){
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x81) || (req->bRequest != USB_REQ_GET_DESCRIPTOR) || (req->wValue != 0x2200)) return USBD_REQ_NOTSUPP;

	// Handle the HID report descriptor.
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

	return USBD_REQ_HANDLED;
}

static enum usbd_request_return_codes hid_control_request2(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void (**complete)(usbd_device *, struct usb_setup_data *)){
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x82) || (req->bRequest != USB_REQ_GET_DESCRIPTOR) || (req->wValue != 0x2200)) return USBD_REQ_NOTSUPP;

	// Handle the HID report descriptor
	*buf = (uint8_t *)hid_report_descriptor2;
	*len = sizeof(hid_report_descriptor2);

	return USBD_REQ_HANDLED;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue){
	(void)wValue;
	(void)dev;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);
    usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

	usbd_register_control_callback(dev, USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, hid_control_request);
	usbd_register_control_callback(dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, hid_control_request2);

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	// SysTick interrupt every N clock pulses: set reload to N-1
	systick_set_reload(99999);
	systick_interrupt_enable();
	systick_counter_enable();
}

static void delay_setup(void){
	rcc_periph_clock_enable(RCC_TIM6);
	timer_set_prescaler(TIM6, rcc_apb1_frequency / 1000000 - 1);
	timer_set_period(TIM6, 0xffff);
	timer_one_shot_mode(TIM6);
}

static void delay_us(uint16_t us){
	TIM_ARR(TIM6) = us;
	TIM_EGR(TIM6) = TIM_EGR_UG;
	TIM_CR1(TIM6) |= TIM_CR1_CEN;
	while (TIM_CR1(TIM6) & TIM_CR1_CEN);
}

static void delay_ms(uint32_t ms){
    for(uint32_t i=ms;i>0;i--){
        delay_us(1000);    
    }
}

int main(void){
	rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSI_48MHZ]);

    delay_setup();

	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (unsigned i = 0; i < 800000; i++) {
		__asm__("nop");
	}

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);

    NRF24L01_Init();

    while(NRF24L01_Check() != 0) {
      delay_ms(2000);
    }

    if(xMode == TX_MODE) {
      NRF24L01_TX_Mode(rxAddress, txAddress);
    } else if(xMode == RX_MODE) {
      NRF24L01_RX_Mode(rxAddress, txAddress);
    }
    while(1) {
      if(xMode == TX_MODE) {
        uint8_t tmp[] = {0x1f, 
          0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
          0x21, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x28,
          0x31, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x38,
          0x41, 0x12, 0x13, 0x14, 0x15, 0x16, 0x47, 0x48
        };
        NRF24L01_TxPacket(tmp, 32);
        delay_ms(3000);
      } else if(xMode == RX_MODE) {
        NRF24L01_RxPacket(RX_BUF);
      }
    }
	while (1) usbd_poll(usbd_dev);
}

void sys_tick_handler(void){
	static int x = 0;
	static int dir = 1;
	uint8_t buf[4] = {0, 0, 0, 0};

	buf[1] = dir;
	x += dir;
	if (x > 30)
		dir = -dir;
	if (x < -30)
		dir = -dir;

	usbd_ep_write_packet(usbd_dev, 0x81, buf, 4);
    usbd_ep_write_packet(usbd_dev, 0x82, buf, 4);
}
