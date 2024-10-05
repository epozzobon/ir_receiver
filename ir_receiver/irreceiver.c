#include <stdlib.h>
#include <stdio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/hid.h>

/* Define this to include the DFU APP interface. */
#define INCLUDE_DFU_INTERFACE

#ifdef INCLUDE_DFU_INTERFACE
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dfu.h>
#include "usb_hid_keys.h"
#endif

typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
} subKeyBoard_t;

subKeyBoard_t keyBoardHIDsub = {0,0,0,0,0,0,0,0};

static usbd_device *usbd_dev;

const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5710,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
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
	.wMaxPacketSize = sizeof(subKeyBoard_t),
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
	.bInterfaceProtocol = 1, /* keyboard */
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

#ifdef INCLUDE_DFU_INTERFACE
const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor dfu_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	.iInterface = 0,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};
#endif

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
#ifdef INCLUDE_DFU_INTERFACE
}, {
	.num_altsetting = 1,
	.altsetting = &dfu_iface,
#endif
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
#ifdef INCLUDE_DFU_INTERFACE
	.bNumInterfaces = 2,
#else
	.bNumInterfaces = 1,
#endif
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	"HID Demo",
	"DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x81) ||
	   (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
	   (req->wValue != 0x2200))
		return USBD_REQ_NOTSUPP;

	/* Handle the HID report descriptor. */
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

	return USBD_REQ_HANDLED;
}

#ifdef INCLUDE_DFU_INTERFACE
static void dfu_detach_complete(usbd_device *dev, struct usb_setup_data *req)
{
	(void)req;
	(void)dev;

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO10);
	gpio_set(GPIOA, GPIO10);
	scb_reset_core();
}

static enum usbd_request_return_codes dfu_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)buf;
	(void)len;
	(void)dev;

	if ((req->bmRequestType != 0x21) || (req->bRequest != DFU_DETACH))
		return USBD_REQ_NOTSUPP; /* Only accept class request. */

	*complete = dfu_detach_complete;

	return USBD_REQ_HANDLED;
}
#endif

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
	(void)wValue;
	(void)dev;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, sizeof(subKeyBoard_t), NULL);

	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				hid_control_request);
#ifdef INCLUDE_DFU_INTERFACE
	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				dfu_control_request);
#endif

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(99999);
	systick_interrupt_enable();
	systick_counter_enable();
}

static void periph_setup(void)
{
	rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_HSI_48MHZ]);

	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_TIM1);

	gpio_set_mode(GPIO_BANK_USART2_TX, GPIO_MODE_OUTPUT_50_MHZ,
	              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

	gpio_set_mode(GPIO_BANK_TIM1_CH1, GPIO_MODE_INPUT,
				  GPIO_CNF_INPUT_FLOAT, GPIO_TIM1_CH1);
	gpio_set_mode(GPIO_BANK_TIM1_CH2, GPIO_MODE_INPUT,
				  GPIO_CNF_INPUT_FLOAT, GPIO_TIM1_CH2);

	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
				  GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);

	rcc_periph_reset_pulse(RST_TIM1);
    timer_set_prescaler(TIM1, 27288 / 16);
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT,
	    		   TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(TIM1, 65535);
    timer_set_clock_division(TIM1, 0x0);
	timer_enable_preload(TIM1);
	timer_continuous_mode(TIM1);

	timer_ic_set_prescaler(TIM1, TIM_IC1, TIM_IC_PSC_OFF);
	timer_ic_set_input(TIM1, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_filter(TIM1, TIM_IC1, TIM_IC_OFF);
	timer_ic_set_polarity(TIM1, TIM_IC1, 0);
	timer_ic_enable(TIM1, TIM_IC1);

	timer_ic_set_prescaler(TIM1, TIM_IC2, TIM_IC_PSC_OFF);
	timer_ic_set_input(TIM1, TIM_IC2, TIM_IC_IN_TI1);
	timer_ic_set_filter(TIM1, TIM_IC2, TIM_IC_OFF);
	timer_ic_set_polarity(TIM1, TIM_IC2, 1);
	timer_ic_enable(TIM1, TIM_IC2);

	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_set_priority(NVIC_TIM1_CC_IRQ, 1);
	timer_enable_irq(TIM1, TIM_DIER_CC1IE | TIM_DIER_CC2IE);

	timer_enable_counter(TIM1);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

static int usartprint(const char *str) {
	int c = 0;
	while (*str != 0) {
		usart_send_blocking(USART2, *str);
		str++;
		c++;
	}
	return c;
}

typedef struct {
	unsigned short ccr;
	bool edge;
} pulse_t;

static volatile struct {
	unsigned short head;
	unsigned short tail;
	pulse_t buffer[512];
} ringbuf;

static volatile bool overflow = false;
static char tmpbuf[513];
static volatile unsigned timestamp = 0;

enum ir_command {
	IRCMD_UNKNOWN = -1,
	IRCMD_NETFLIX,
	IRCMD_HOME,
	IRCMD_WHAT,
	IRCMD_NUM_0,
	IRCMD_NUM_1,
	IRCMD_NUM_2,
	IRCMD_NUM_3,
	IRCMD_NUM_4,
	IRCMD_NUM_5,
	IRCMD_NUM_6,
	IRCMD_NUM_7,
	IRCMD_NUM_8,
	IRCMD_NUM_9,
	IRCMD_DISNEY,
	IRCMD_MUTE,
	IRCMD_POWER,
	IRCMD_SOURCE,
	IRCMD_INFO,
	IRCMD_TXT,
	IRCMD_UP,
	IRCMD_VOL_UP,
	IRCMD_P_UP,
	IRCMD_DOWN,
	IRCMD_VOL_DOWN,
	IRCMD_P_DOWN,
	IRCMD_OK,
	IRCMD_PRIME,
	IRCMD_RIGHT,
	IRCMD_LEFT,
	IRCMD_EXIT,
	IRCMD_GUIDE,
	IRCMD_RED,
	IRCMD_GREEN,
	IRCMD_YELLOW,
	IRCMD_BLUE,
	IRCMD_MENU,
	IRCMD_BACK,
	IRCMD_STOP,
	IRCMD_FASTFORWARD,
	IRCMD_REWIND,
	IRCMD_REC,
	IRCMD_PLAYPAUSE,
	IRCMD_RAKUTEN,
	IRCMD_LANG,
	IRCMD_YOUTUBE,
	TOTAL_IRCMD_COUNT
};

static unsigned ircmd_lastpress;
static enum ir_command currentcmd = IRCMD_UNKNOWN;

static enum ir_command bits_to_command(unsigned long long command) {
	switch (command) {
		case 0x0adb76db5577f0: return IRCMD_NETFLIX;
		case 0x0aed75bb5577f0: return IRCMD_HOME;
		case 0x0d575f5b5577f0: return IRCMD_WHAT;
		case 0x15555ff6aab7f0: return IRCMD_NUM_0;
		case 0x15557fd6aab7f0: return IRCMD_NUM_1;
		case 0x1555bfb6aab7f0: return IRCMD_NUM_2;
		case 0x1555ff56aab7f0: return IRCMD_NUM_3;
		case 0x1556bf76aab7f0: return IRCMD_NUM_4;
		case 0x1556fed6aab7f0: return IRCMD_NUM_5;
		case 0x15577eb6aab7f0: return IRCMD_NUM_6;
		case 0x1557fd56aab7f0: return IRCMD_NUM_7;
		case 0x155abef6aab7f0: return IRCMD_NUM_8;
		case 0x155afdd6aab7f0: return IRCMD_NUM_9;
		case 0x155bfb56aab7f0: return IRCMD_DISNEY;
		case 0x156abdf6aab7f0: return IRCMD_MUTE;
		case 0x156b7bb6aab7f0: return IRCMD_POWER;
		case 0x156d7b76aab7f0: return IRCMD_SOURCE;
		case 0x156ef6b6aab7f0: return IRCMD_INFO;
		case 0x156fed56aab7f0: return IRCMD_TXT;
		case 0x1575f5d6aab7f0: return IRCMD_UP;
		case 0x1576f5b6aab7f0: return IRCMD_VOL_UP;
		case 0x1577eb56aab7f0: return IRCMD_P_UP;
		case 0x157bead6aab7f0: return IRCMD_DOWN;
		case 0x157deab6aab7f0: return IRCMD_VOL_DOWN;
		case 0x157fd556aab7f0: return IRCMD_P_DOWN;
		case 0x15aaf7d6aab7f0: return IRCMD_OK;
		case 0x15d6ebb6aab7f0: return IRCMD_PRIME;
		case 0x16aab7f6aab7f0: return IRCMD_RIGHT;
		case 0x16ab6fb6aab7f0: return IRCMD_LEFT;
		case 0x16abdf56aab7f0: return IRCMD_EXIT;
		case 0x16added6aab7f0: return IRCMD_GUIDE;
		case 0x16b56ef6aab7f0: return IRCMD_RED;
		case 0x16b5ddd6aab7f0: return IRCMD_GREEN;
		case 0x16b6ddb6aab7f0: return IRCMD_YELLOW;
		case 0x16b7bb56aab7f0: return IRCMD_BLUE;
		case 0x16ef6b56aab7f0: return IRCMD_MENU;
		case 0x175ad776aab7f0: return IRCMD_BACK;
		case 0x176ad6f6aab7f0: return IRCMD_STOP;
		case 0x176f5b56aab7f0: return IRCMD_FASTFORWARD;
		case 0x1775ad76aab7f0: return IRCMD_REWIND;
		case 0x177eb556aab7f0: return IRCMD_REC;
		case 0x17db55b6aab7f0: return IRCMD_PLAYPAUSE;
		case 0x1d7abab6aab7f0: return IRCMD_RAKUTEN;
		case 0x1d7d7556aab7f0: return IRCMD_LANG;
		case 0x1dad5bb6aab7f0: return IRCMD_YOUTUBE;
		default: return IRCMD_UNKNOWN;
	}
}

static void on_key_down(enum ir_command cmd) {
	usartprint((snprintf(tmpbuf, 512, "key_down %d\r\n", cmd), tmpbuf));
}

static void on_key_up(enum ir_command cmd) {
	usartprint((snprintf(tmpbuf, 512, "key_up %d\r\n", cmd), tmpbuf));
}

static enum ir_command on_command_ll(unsigned long long command) {
	static unsigned long long prev = 0;
	if (command == 4) {
		command = prev;
	} else {
		prev = command;
	}
	enum ir_command cmd = bits_to_command(command);
	if (currentcmd != cmd) {
		if (currentcmd != IRCMD_UNKNOWN) {
			on_key_up(currentcmd);
		}
		currentcmd = cmd;
		on_key_down(cmd);
	}
	ircmd_lastpress = TIM_CNT(TIM1);
	return cmd;
}

static void handle_ir() {
	static pulse_t opulse = {};
	static bool burst = false;
	static unsigned short burst_cnt = 0;
	static unsigned long long command = 0;
	static unsigned bitidx = 64;
	static bool preamble_found = false;

	if (overflow) {
		usartprint((snprintf(tmpbuf, 512, "<-- OVERFLOW -->\r\n"), tmpbuf));
		overflow = false;
	}

	if (ringbuf.tail != ringbuf.head) {
		pulse_t pulse = ringbuf.buffer[ringbuf.tail];
		ringbuf.tail = (ringbuf.tail + 1) % 512;

		if (!burst && pulse.edge == 0) {
			burst = true;
		} else {
			unsigned short len = (pulse.ccr - opulse.ccr) & 0xffff;
			len = (len + 8) / 16;
			if (len > 20 && pulse.edge == 0) {
				preamble_found = false;
				on_command_ll(command);
			} else if (len == 16 && pulse.edge == 1) {
				bitidx = 0;
				preamble_found = true;
				command = 0;
			} else if (preamble_found) {
				for (unsigned i = 0; i < len; i++) {
					if (bitidx < 128) {
						if (pulse.edge) {
							command |= 1ULL << (bitidx / 2);
						}
						bitidx++;
					}
				}
			}
		}
		opulse = pulse;
		burst_cnt = TIM_CNT(TIM1);
	} else if (burst && ((TIM_CNT(TIM1) - burst_cnt) & 0xffff) > 2000) {
		burst = false;
		preamble_found = false;
		on_command_ll(command);
	}
}

int main(void)
{
	tmpbuf[512] = 0;
	ringbuf.head = 0;
	ringbuf.tail = 0;
	periph_setup();

	gpio_clear(GPIOA, GPIO12);
	for (unsigned i = 0; i < 800000; i++) {
		__asm__("nop");
	}

	usart_send_blocking(USART2, 'b');
	usart_send_blocking(USART2, 'o');
	usart_send_blocking(USART2, 'o');
	usart_send_blocking(USART2, 't');
	usart_send_blocking(USART2, '\r');
	usart_send_blocking(USART2, '\n');

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);

	while (1) {
		handle_ir();

		if (currentcmd != IRCMD_UNKNOWN) {
			unsigned passed = TIM_CNT(TIM1) - ircmd_lastpress;
			if (passed > 4000) {
				on_key_up(currentcmd);
				currentcmd = IRCMD_UNKNOWN;
			}
		};

		usbd_poll(usbd_dev);
	}
}

void tim1_cc_isr(void)
{
	unsigned short ccr;
	bool edge;
	if (TIM_SR(TIM1) & TIM_SR_CC1IF) {
		ccr = TIM_CCR1(TIM1);
		edge = 1;
		ringbuf.buffer[ringbuf.head] = (pulse_t){ccr, edge};
		ringbuf.head = (ringbuf.head + 1) % 512;
		if (ringbuf.head == ringbuf.tail) {
			overflow = true;
		}
		TIM_SR(TIM1) &= ~TIM_SR_CC1IF;
	}
	if (TIM_SR(TIM1) & TIM_SR_CC2IF) {
		ccr = TIM_CCR2(TIM1);
		edge = 0;
		ringbuf.buffer[ringbuf.head] = (pulse_t){ccr, edge};
		ringbuf.head = (ringbuf.head + 1) % 512;
		if (ringbuf.head == ringbuf.tail) {
			overflow = true;
		}
		TIM_SR(TIM1) &= ~TIM_SR_CC2IF;
	}
	if (TIM_SR(TIM1) & (TIM_SR_CC1OF | TIM_SR_CC2OF)) {
		overflow = true;
		TIM_SR(TIM1) &= ~(TIM_SR_CC1OF | TIM_SR_CC2OF);
	}
}

void sys_tick_handler(void)
{
	timestamp++;
	keyBoardHIDsub.MODIFIER = 0;
	switch (currentcmd) {
		case IRCMD_RIGHT:                                                 keyBoardHIDsub.KEYCODE1 = KEY_RIGHT;           break;
		case IRCMD_LEFT:                                                  keyBoardHIDsub.KEYCODE1 = KEY_LEFT;            break;
		case IRCMD_DOWN:                                                  keyBoardHIDsub.KEYCODE1 = KEY_DOWN;            break;
		case IRCMD_UP:                                                    keyBoardHIDsub.KEYCODE1 = KEY_UP;              break;
		case IRCMD_OK:                                                    keyBoardHIDsub.KEYCODE1 = KEY_ENTER;           break;
		case IRCMD_PLAYPAUSE:                                             keyBoardHIDsub.KEYCODE1 = KEY_SPACE;           break;
		case IRCMD_REWIND:      keyBoardHIDsub.MODIFIER = KEY_MOD_LALT;   keyBoardHIDsub.KEYCODE1 = KEY_LEFT;            break;
		case IRCMD_FASTFORWARD: keyBoardHIDsub.MODIFIER = KEY_MOD_LALT;   keyBoardHIDsub.KEYCODE1 = KEY_RIGHT;           break;
		case IRCMD_STOP:                                                  keyBoardHIDsub.KEYCODE1 = KEY_S;               break;
		case IRCMD_RED:         keyBoardHIDsub.MODIFIER = KEY_MOD_LSHIFT; keyBoardHIDsub.KEYCODE1 = KEY_TAB;             break;
		case IRCMD_GREEN:                                                 keyBoardHIDsub.KEYCODE1 = KEY_F2;              break;
		case IRCMD_YELLOW:                                                keyBoardHIDsub.KEYCODE1 = KEY_F3;              break;
		case IRCMD_BLUE:                                                  keyBoardHIDsub.KEYCODE1 = KEY_TAB;             break;
		case IRCMD_NUM_1:                                                 keyBoardHIDsub.KEYCODE1 = KEY_1;               break;
		case IRCMD_NUM_2:                                                 keyBoardHIDsub.KEYCODE1 = KEY_2;               break;
		case IRCMD_NUM_3:                                                 keyBoardHIDsub.KEYCODE1 = KEY_3;               break;
		case IRCMD_NUM_4:                                                 keyBoardHIDsub.KEYCODE1 = KEY_4;               break;
		case IRCMD_NUM_5:                                                 keyBoardHIDsub.KEYCODE1 = KEY_5;               break;
		case IRCMD_NUM_6:                                                 keyBoardHIDsub.KEYCODE1 = KEY_6;               break;
		case IRCMD_NUM_7:                                                 keyBoardHIDsub.KEYCODE1 = KEY_7;               break;
		case IRCMD_NUM_8:                                                 keyBoardHIDsub.KEYCODE1 = KEY_8;               break;
		case IRCMD_NUM_9:                                                 keyBoardHIDsub.KEYCODE1 = KEY_9;               break;
		case IRCMD_NUM_0:                                                 keyBoardHIDsub.KEYCODE1 = KEY_0;               break;
		case IRCMD_LANG:                                                  keyBoardHIDsub.KEYCODE1 = KEY_BACKSPACE;       break;
		case IRCMD_TXT:                                                   keyBoardHIDsub.KEYCODE1 = KEY_F12;             break;
		case IRCMD_EXIT:        keyBoardHIDsub.MODIFIER = KEY_MOD_LALT;   keyBoardHIDsub.KEYCODE1 = KEY_F4;              break;
		case IRCMD_BACK:                                                  keyBoardHIDsub.KEYCODE1 = KEY_HOME;            break;
		case IRCMD_P_UP:                                                  keyBoardHIDsub.KEYCODE1 = KEY_PAGEUP;          break;
		case IRCMD_P_DOWN:                                                keyBoardHIDsub.KEYCODE1 = KEY_PAGEDOWN;        break;
		case IRCMD_UNKNOWN: default: keyBoardHIDsub.KEYCODE1 = 0;
	}

	usbd_ep_write_packet(usbd_dev, 0x81, &keyBoardHIDsub, sizeof(keyBoardHIDsub));
}
