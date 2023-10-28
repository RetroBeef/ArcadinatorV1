#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

#include "systick.h"
#include "nrf24l01.h"
#include "buttons.h"
#include "panel.h"

//#define NX_MODE

typedef enum{
    RX_MODE = 0,
    TX_MODE = 1,
	USB_MODE = 2
} xMode_t;

xMode_t xMode = USB_MODE;
panelType_t panelType = DUAL_PANEL;

uint8_t rxAddress[NRF24L01_ADDR_WIDTH] = {0x11,0x22,0x33,0x44,0x55};//todo
uint8_t txAddress[NRF24L01_ADDR_WIDTH] = {0x11,0x22,0x33,0x44,0x55};
uint8_t nrfBuf[NRF24L01_PLOAD_WIDTH] = {0};

PanelData_t panelState = {0};
PanelDataNX_t panelStateNx = {0};

static usbd_device *usbd_dev;

static uint8_t doButtonUpdates = 0;

const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
#if defined(NX_MODE)
	.idVendor = (0x1209 - 0x02fc),
	.idProduct = (0xcade - 0xca1d),
#else
	.idVendor = 0x1209,
	.idProduct = 0xcade,
#endif
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
#if defined(NX_MODE)
	.iSerialNumber = 0,//todo check if really needed
#else
	.iSerialNumber = 3,
#endif
	.bNumConfigurations = 1,
};

#if defined(NX_MODE)
static const uint8_t hid_report_descriptor[] = {
	0x05, 0x01, /* USAGE_PAGE (Generic Desktop)         */
	0x09, 0x05, /* USAGE (Gamepad)                      */
	0xa1, 0x01, /* COLLECTION (Application)             */
	0x15, 0x00, /*     LOGICAL_MINIMUM (0)              */
	0x25, 0x01, /*     LOGICAL_MAXIMUM (1)              */
	0x35, 0x00, /*     PHYSICAL_MINIMUM (0)             */
	0x45, 0x01, /*     PHYSICAL_MAXIMUM (1)             */
	0x75, 0x01, /*     REPORT_SIZE (1)                  */
	0x95, 0x0e, /*     REPORT_COUNT (14)                */
	0x05, 0x09, /*     USAGE_PAGE (Buttons)             */
	0x19, 0x01, /*     USAGE_MINIMUM (Button 1)         */
	0x29, 0x0e, /*     USAGE_MAXIMUM (Button 14)        */
	0x81, 0x02, /*     INPUT (Data,Var,Abs)             */
	0x95, 0x02, /*     REPORT_COUNT (2)                 */
	0x81, 0x01, /*     INPUT (Data,Var,Abs)             */
	0x05, 0x01, /*     USAGE_PAGE (Generic Desktop Ctr) */
	0x25, 0x07, /*     LOGICAL_MAXIMUM (7)              */
	0x46, 0x3b, 0x01, /*     PHYSICAL_MAXIMUM (315)     */
	0x75, 0x04, /*     REPORT_SIZE (4)                  */
	0x95, 0x01, /*     REPORT_COUNT (1)                 */
	0x65, 0x14, /*     UNIT (20)                        */
	0x09, 0x39, /*     USAGE (Hat Switch)               */
	0x81, 0x42, /*     INPUT (Data,Var,Abs)             */
	0x65, 0x00, /*     UNIT (0)                         */
	0x95, 0x01, /*     REPORT_COUNT (1)                 */
	0x81, 0x01, /*     INPUT (Data,Var,Abs)             */
	0x26, 0xff, 0x00, /*     LOGICAL_MAXIMUM (255)      */
	0x46, 0xff, 0x00, /*     PHYSICAL_MAXIMUM (255)     */
	0x09, 0x30, /*     USAGE (Direction-X)              */
	0x09, 0x31, /*     USAGE (Direction-Y)              */
	0x09, 0x32, /*     USAGE (Direction-Z)              */
	0x09, 0x35, /*     USAGE (Rotate-Z)                 */
	0x75, 0x08, /*     REPORT_SIZE (8)                  */
	0x95, 0x04, /*     REPORT_COUNT (4)                 */
	0x81, 0x02, /*     INPUT (Data,Var,Abs)             */
	0x75, 0x08, /*     REPORT_SIZE (8)                  */
	0x95, 0x01, /*     REPORT_COUNT (1)                 */
	0x81, 0x01, /*     INPUT (Data,Var,Abs)             */
	0xc0,       /*   END_COLLECTION                     */
};
#else
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,         // Usage Page (Generic Desktop)
    0x09, 0x05,         // Usage (Game Pad)
    0xA1, 0x01,         // Collection (Application)
    0x15, 0x00,         // Logical Minimum (0)
    0x25, 0x01,         // Logical Maximum (1)
    0x35, 0x00,         // Physical Minimum (0)
    0x45, 0x01,         // Physical Maximum (1)
    0x75, 0x01,         // Report Size (1)
    0x95, 0x10,         // Report Count (16)
    0x05, 0x09,         // Usage Page (Button)
    0x19, 0x01,         // Usage Minimum (Button 1)
    0x29, 0x10,         // Usage Maximum (Button 16)
    0x81, 0x02,         // Input (Data, Variable, Absolute)

    0xC0                // End Collection
};
#endif

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
#if defined(NX_MODE)
		.bcdHID = 0x0111,//todo check if really needed
#else
		.bcdHID = 0x0100,
#endif
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	}
};

const struct usb_endpoint_descriptor hid_endpoint_player1 = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
#if defined(NX_MODE)
	.wMaxPacketSize = sizeof(panelStateNx.obj.player1.bytes),
#else
	.wMaxPacketSize = sizeof(panelState.obj.player1.bytes),
#endif
	.bInterval = 1
};

const struct usb_endpoint_descriptor hid_endpoint_player2 = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
#if defined(NX_MODE)
	.wMaxPacketSize = sizeof(panelStateNx.obj.player2.bytes),
#else
	.wMaxPacketSize = sizeof(panelState.obj.player2.bytes),
#endif
	.bInterval = 1
};

const struct usb_interface_descriptor hid_iface_player1 = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = &hid_endpoint_player1,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_interface_descriptor hid_iface_player2 = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = &hid_endpoint_player2,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_interface ifaces_single[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface_player1,
}};

const struct usb_interface ifaces_dual[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface_player1,
}, {
	.num_altsetting = 1,
	.altsetting = &hid_iface_player2,
}};

const struct usb_config_descriptor config_single = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces_single,
};

const struct usb_config_descriptor config_dual = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces_dual,
};

static const char *usb_strings_single[] = {
	"RetroBeef",
	"Arcadinator FS",
	"V1"
};

static const char *usb_strings_dual[] = {
	"RetroBeef",
	"Arcadinator Dual",
	"V1"
};

static const char *usb_strings_single_rf[] = {
	"RetroBeef",
	"Arcadinator FS Wireless",
	"V1"
};

static const char *usb_strings_dual_rf[] = {
	"RetroBeef",
	"Arcadinator Dual Wireless",
	"V1"
};

uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void (**complete)(usbd_device *, struct usb_setup_data *)){
	(void)complete;
	(void)dev;

	if((req->bRequest != USB_REQ_GET_DESCRIPTOR) || (req->wValue != 0x2200)){
        return USBD_REQ_NOTSUPP;
    }else if(req->bmRequestType == 0x81 || req->bmRequestType == 0x82){
	    *buf = (uint8_t *)hid_report_descriptor;
	    *len = sizeof(hid_report_descriptor);
        return USBD_REQ_HANDLED;
    }

    return USBD_REQ_NOTSUPP;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue){
	(void)wValue;
	(void)dev;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, sizeof(panelState.obj.player1.bytes), NULL);
	if(panelType == DUAL_PANEL){
		usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_INTERRUPT, sizeof(panelState.obj.player2.bytes), NULL);
	}
    usbd_register_control_callback(dev, USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, hid_control_request);

    doButtonUpdates = 1;
}

static void buttons_update(void){
    panelState.obj.player1.obj.joyUp = !digitalRead(B01);
    panelState.obj.player1.obj.joyDown = !digitalRead(B02);
    panelState.obj.player1.obj.joyLeft = !digitalRead(B03);
    panelState.obj.player1.obj.joyRight = !digitalRead(B04);
    panelState.obj.player1.obj.button01 = !digitalRead(B05);
    panelState.obj.player1.obj.button02 = !digitalRead(B06);
    panelState.obj.player1.obj.button03 = !digitalRead(B07);
    panelState.obj.player1.obj.button04 = !digitalRead(B08);
    panelState.obj.player1.obj.button05 = !digitalRead(B09);
    panelState.obj.player1.obj.button06 = !digitalRead(B10);
    panelState.obj.player1.obj.buttonStart = !digitalRead(B11);
    panelState.obj.player1.obj.buttonExtra = !digitalRead(B12);

	if(panelType == DUAL_PANEL){
		panelState.obj.player2.obj.joyUp = !digitalRead(B13);
		panelState.obj.player2.obj.joyDown = !digitalRead(B14);
		panelState.obj.player2.obj.joyLeft = !digitalRead(B15);
		panelState.obj.player2.obj.joyRight = !digitalRead(B16);
		panelState.obj.player2.obj.button01 = !digitalRead(B17);
		panelState.obj.player2.obj.button02 = !digitalRead(B18);
		panelState.obj.player2.obj.button03 = !digitalRead(B19);
		panelState.obj.player2.obj.button04 = !digitalRead(B20);
		panelState.obj.player2.obj.button05 = !digitalRead(B21);
		panelState.obj.player2.obj.button06 = !digitalRead(B22);
		panelState.obj.player2.obj.buttonStart = !digitalRead(B23);
		panelState.obj.player2.obj.buttonExtra = !digitalRead(B24);
	}
}

#if defined(NX_MODE)
static void translateToNx(void){
	uint8_t up = panelState.obj.player1.obj.joyUp;
	uint8_t down = panelState.obj.player1.obj.joyDown;
    uint8_t left = panelState.obj.player1.obj.joyLeft;
    uint8_t right = panelState.obj.player1.obj.joyRight;
    panelStateNx.obj.player1.obj.hat = HATSWITCH_NONE;
    if(up){
    	if(right) panelStateNx.obj.player1.obj.hat = HATSWITCH_UPRIGHT;
    	else if(left) panelStateNx.obj.player1.obj.hat = HATSWITCH_UPLEFT;
    	else panelStateNx.obj.player1.obj.hat = HATSWITCH_UP;
    }else if(down){
    	if(right) panelStateNx.obj.player1.obj.hat = HATSWITCH_DOWNRIGHT;
    	else if(left) panelStateNx.obj.player1.obj.hat = HATSWITCH_DOWNLEFT;
    	else panelStateNx.obj.player1.obj.hat = HATSWITCH_DOWN;
    }else if(left){
    	panelStateNx.obj.player1.obj.hat = HATSWITCH_LEFT;
    }else if(right){
    	panelStateNx.obj.player1.obj.hat = HATSWITCH_RIGHT;
    }

    panelStateNx.obj.player1.obj.y = panelState.obj.player1.obj.button01;
    panelStateNx.obj.player1.obj.b = panelState.obj.player1.obj.button02;
    panelStateNx.obj.player1.obj.a = panelState.obj.player1.obj.button03;
    panelStateNx.obj.player1.obj.x = panelState.obj.player1.obj.button04;
    panelStateNx.obj.player1.obj.l = panelState.obj.player1.obj.button05;
    panelStateNx.obj.player1.obj.r = panelState.obj.player1.obj.button06;
    panelStateNx.obj.player1.obj.home = panelState.obj.player1.obj.buttonStart;
    panelStateNx.obj.player1.obj.plus = panelState.obj.player1.obj.buttonExtra;
    panelStateNx.obj.player1.obj.lx = 127;
    panelStateNx.obj.player1.obj.ly = 127;
    panelStateNx.obj.player1.obj.rx = 127;
    panelStateNx.obj.player1.obj.ry = 127;
    if(panelType == DUAL_PANEL){
		up = panelState.obj.player2.obj.joyUp;
		down = panelState.obj.player2.obj.joyDown;
		left = panelState.obj.player2.obj.joyLeft;
		right = panelState.obj.player2.obj.joyRight;
		panelStateNx.obj.player2.obj.hat = HATSWITCH_NONE;
		if(up){
			if(right) panelStateNx.obj.player2.obj.hat = HATSWITCH_UPRIGHT;
			else if(left) panelStateNx.obj.player2.obj.hat = HATSWITCH_UPLEFT;
			else panelStateNx.obj.player2.obj.hat = HATSWITCH_UP;
		}else if(down){
			if(right) panelStateNx.obj.player2.obj.hat = HATSWITCH_DOWNRIGHT;
			else if(left) panelStateNx.obj.player2.obj.hat = HATSWITCH_DOWNLEFT;
			else panelStateNx.obj.player2.obj.hat = HATSWITCH_DOWN;
		}else if(left){
			panelStateNx.obj.player2.obj.hat = HATSWITCH_LEFT;
		}else if(right){
			panelStateNx.obj.player2.obj.hat = HATSWITCH_RIGHT;
		}
		panelStateNx.obj.player2.obj.y = panelState.obj.player2.obj.button01;
		panelStateNx.obj.player2.obj.b = panelState.obj.player2.obj.button02;
		panelStateNx.obj.player2.obj.a = panelState.obj.player2.obj.button03;
		panelStateNx.obj.player2.obj.x = panelState.obj.player2.obj.button04;
		panelStateNx.obj.player2.obj.l = panelState.obj.player2.obj.button05;
		panelStateNx.obj.player2.obj.r = panelState.obj.player2.obj.button06;
		panelStateNx.obj.player2.obj.home = panelState.obj.player2.obj.buttonStart;
		panelStateNx.obj.player2.obj.plus = panelState.obj.player2.obj.buttonExtra;
		panelStateNx.obj.player2.obj.lx = 127;
		panelStateNx.obj.player2.obj.ly = 127;
		panelStateNx.obj.player2.obj.rx = 127;
		panelStateNx.obj.player2.obj.ry = 127;
    }
}
#endif

void usb_lp_can_rx0_isr(){
	usbd_poll(usbd_dev);
}

static void usb_setup(void) {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
    gpio_clear(GPIOA, GPIO12);
	for (unsigned i = 0; i < 800000; i++) {
		__asm__("nop");
	}

	if(panelType == DUAL_PANEL){
		if(xMode == USB_MODE){
			usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config_dual, usb_strings_dual, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
		}else{
			usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config_dual, usb_strings_dual_rf, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
		}
	}else{
		if(xMode == USB_MODE){
			usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config_single, usb_strings_single, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
		}else{
			usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config_single, usb_strings_single_rf, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
		}
	}
	usbd_register_set_config_callback(usbd_dev, hid_set_config);

    nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 2 << 6);
    nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
}

static void clocks_setup(void){
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    systick_setup();

    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
}

int main(void){
    clocks_setup();

    if(xMode == RX_MODE || xMode == USB_MODE){
    	usb_setup();
    }

    if(xMode == TX_MODE || xMode == USB_MODE){
    	buttons_setup();
    }

    if(xMode == RX_MODE || xMode == TX_MODE){
    	nrf24_init();
        while(nrf24_check() != 0) {
        	__asm("nop");
        }
    }

    if(xMode == TX_MODE) {
    	nrf24_tx_mode(rxAddress, txAddress);
    } else if(xMode == RX_MODE) {
    	nrf24_rx_mode(rxAddress, txAddress);
        gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
        gpio_set(GPIOC, GPIO13);
    }

	while (1){
        if(xMode == RX_MODE || xMode == USB_MODE){
        	if(!doButtonUpdates)continue;
        }

        if(xMode == TX_MODE || xMode == USB_MODE){
        	buttons_update();
        }

        if(xMode == TX_MODE) {
        	if(panelType==DUAL_PANEL){
        		memcpy(nrfBuf, panelState.bytes, sizeof(panelState.bytes));
        	}else{
        		memcpy(nrfBuf, panelState.obj.player1.bytes, sizeof(panelState.obj.player1.bytes));
        	}
            nrf24_tx_packet(nrfBuf, NRF24L01_PLOAD_WIDTH);
        } else if(xMode == RX_MODE || xMode == USB_MODE) {
        	if(xMode == RX_MODE){
        		if(!IRQ){
        			gpio_clear(GPIOC, GPIO13);
        			nrf24_rx_packet(nrfBuf, NRF24L01_PLOAD_WIDTH);
        			memcpy(panelState.bytes, nrfBuf, sizeof(panelState.bytes));
            	}else{
            		gpio_set(GPIOC, GPIO13);
            	}
#if defined(NX_MODE)
        		translateToNx();
            	usbd_ep_write_packet(usbd_dev, 0x81, panelStateNx.obj.player1.bytes, sizeof(panelStateNx.obj.player1.bytes));
    			if(panelType==DUAL_PANEL){
    				usbd_ep_write_packet(usbd_dev, 0x82, panelStateNx.obj.player2.bytes, sizeof(panelStateNx.obj.player2.bytes));
    			}
#else
            	usbd_ep_write_packet(usbd_dev, 0x81, panelState.obj.player1.bytes, sizeof(panelState.obj.player1.bytes));
    			if(panelType==DUAL_PANEL){
    				usbd_ep_write_packet(usbd_dev, 0x82, panelState.obj.player2.bytes, sizeof(panelState.obj.player2.bytes));
    			}
#endif
            }else{
#if defined(NX_MODE)
            	translateToNx();
				while(!usbd_ep_write_packet(usbd_dev, 0x81, panelStateNx.obj.player1.bytes, sizeof(panelStateNx.obj.player1.bytes))){
					__asm("nop");//timeout?
				}
				if(panelType==DUAL_PANEL){
					while(!usbd_ep_write_packet(usbd_dev, 0x82, panelStateNx.obj.player2.bytes, sizeof(panelStateNx.obj.player2.bytes))){
						__asm("nop");//timeout?
					}
				}
#else
				while(!usbd_ep_write_packet(usbd_dev, 0x81, panelState.obj.player1.bytes, sizeof(panelState.obj.player1.bytes))){
					__asm("nop");//timeout?
				}
				if(panelType==DUAL_PANEL){
					while(!usbd_ep_write_packet(usbd_dev, 0x82, panelState.obj.player2.bytes, sizeof(panelState.obj.player2.bytes))){
						__asm("nop");//timeout?
					}
				}
#endif
            }
        }
    }
}
