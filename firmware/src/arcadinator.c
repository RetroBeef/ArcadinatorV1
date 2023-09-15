#include <stdlib.h>
#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

#include "systick.h"
#include "buttons.h"
#include "panel.h"

PanelData_t panelState = {0};

static usbd_device *usbd_dev;

static uint8_t doButtonUpdates = 0;
static uint32_t lastButtonsUpdateMs = 0;
const uint32_t buttonUpdateIntervalMs = 8;

const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0000, //0x1209: sent PR to https://pid.codes/
	.idProduct = 0xcade,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,         // Usage Page (Generic Desktop)
    0x09, 0x05,         // Usage (Game Pad)
    0xA1, 0x01,         // Collection (Application)
    0x15, 0x00,         // Logical Minimum (0)
    0x25, 0x01,         // Logical Maximum (1)
    0x35, 0x00,         // Physical Minimum (0)
    0x45, 0x01,         // Physical Maximum (1)
    0x75, 0x01,         // Report Size (1)
    0x95, 0x02,         // Report Count (2)
    0x05, 0x09,         // Usage Page (Button)
    0x19, 0x01,         // Usage Minimum (Button 1)
    0x29, 0x02,         // Usage Maximum (Button 2)
    0x81, 0x02,         // Input (Data, Variable, Absolute)

    0xC0                // End Collection
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

const struct usb_endpoint_descriptor hid_endpoint_player1 = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = sizeof(panelState.obj.player1.bytes),
	.bInterval = 10,
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

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface_player1,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"RetroBeef",
	"Arcadinator Shifter",
	"V1"
};

uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, void (**complete)(usbd_device *, struct usb_setup_data *)){
	(void)complete;
	(void)dev;

	if((req->bRequest != USB_REQ_GET_DESCRIPTOR) || (req->wValue != 0x2200)){
        return USBD_REQ_NOTSUPP;
    }else if(req->bmRequestType == 0x81){
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

    usbd_register_control_callback(dev, USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE, USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, hid_control_request);

    doButtonUpdates = 1;
}

static void buttons_update(void){
    panelState.obj.player1.obj.joyUp = !digitalRead(B23);
    panelState.obj.player1.obj.joyDown = !digitalRead(B24);
}

static void usb_setup(void) {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
    gpio_clear(GPIOA, GPIO12);
	for (unsigned i = 0; i < 800000; i++) {
		__asm__("nop");
	}

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);
}

static void clocks_setup(void){
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    systick_setup();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
}

int main(void){
    clocks_setup();
    usb_setup();
    buttons_setup();
	while (1){
        usbd_poll(usbd_dev);
        if(!doButtonUpdates)continue;
        if(millis() - lastButtonsUpdateMs >= buttonUpdateIntervalMs){
            buttons_update(); 
            usbd_ep_write_packet(usbd_dev, 0x81, panelState.obj.player1.bytes, sizeof(panelState.obj.player1.bytes));
            lastButtonsUpdateMs = millis();
        }
    }
}
