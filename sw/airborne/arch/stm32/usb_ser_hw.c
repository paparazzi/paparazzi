/*
 * Copyright (C) 2014 Michal Podhradsky,
 * based on example from libopencm3
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/stm32/usb_ser_hw.c
 * CDC USB device driver for STM32 architecture (STM32F1, STM32F4)
 */

/* This version derived from libopencm3 example */
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>

#include "mcu_periph/usb_serial.h"


/* Max packet size for USB transfer */
#define MAX_PACKET_SIZE          64
/* Max fifo size for storing data */
#define VCOM_FIFO_SIZE          128

typedef struct {
  int         head;
  int         tail;
  uint8_t     *buf;
} fifo_t;

static uint8_t txdata[VCOM_FIFO_SIZE];
static uint8_t rxdata[VCOM_FIFO_SIZE];

static fifo_t txfifo;
static fifo_t rxfifo;

void fifo_init(fifo_t *fifo, uint8_t *buf);
bool_t fifo_put(fifo_t *fifo, uint8_t c);
bool_t fifo_get(fifo_t *fifo, uint8_t *pc);
int  fifo_avail(fifo_t *fifo);
int  fifo_free(fifo_t *fifo);
inline char *get_dev_unique_id(char *serial_no);



usbd_device *my_usbd_dev;

static const struct usb_device_descriptor dev = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = USB_CLASS_CDC,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = MAX_PACKET_SIZE,
  .idVendor = 0x1d50, // OpenMoko, Inc.
  .idProduct = 0x603d, // Paparazzi LPC(STM32)USB Serial
  .bcdDevice = 0x0200,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
  }
};

static const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = MAX_PACKET_SIZE,
    .bInterval = 1,
  }, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = MAX_PACKET_SIZE,
    .bInterval = 1,
  }
};

static const struct {
  struct usb_cdc_header_descriptor header;
  struct usb_cdc_call_management_descriptor call_mgmt;
  struct usb_cdc_acm_descriptor acm;
  struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
  .header = {
    .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
    .bcdCDC = 0x0110,
  },
  .call_mgmt = {
    .bFunctionLength =
    sizeof(struct usb_cdc_call_management_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
    .bmCapabilities = 0,
    .bDataInterface = 1,
  },
  .acm = {
    .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_ACM,
    .bmCapabilities = 0,
  },
  .cdc_union = {
    .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_UNION,
    .bControlInterface = 0,
    .bSubordinateInterface0 = 1,
  }
};

static const struct usb_interface_descriptor comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,

    .endpoint = comm_endp,

    .extra = &cdcacm_functional_descriptors,
    .extralen = sizeof(cdcacm_functional_descriptors)
  }
};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
  }
};

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = comm_iface,
  }, {
    .num_altsetting = 1,
    .altsetting = data_iface,
  }
};

static const struct usb_config_descriptor config = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0,
  .bNumInterfaces = 2,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0x80,
  .bMaxPower = 0x32,

  .interface = ifaces,
};

static char serial_no[25];

/* Description of the device as it appears after enumeration */
static const char *usb_strings[] = {
  "Paparazzi UAV",
  "CDC Serial STM32",
  serial_no,
};

/**
 * Serial is 96bit so 12bytes so 12 hexa numbers, or 24 decimal + termination character
 */
inline char *get_dev_unique_id(char *s)
{
#if defined STM32F4
  volatile uint8_t *unique_id = (volatile uint8_t *)0x1FFF7A10;
#else
  volatile uint8_t *unique_id = (volatile uint8_t *)0x1FFFF7E8;
#endif
  int i;

  // Fetch serial number from chip's unique ID
  for (i = 0; i < 24; i += 2) {
    s[i] = ((*unique_id >> 4) & 0xF) + '0';
    s[i + 1] = (*unique_id++ & 0xF) + '0';
  }
  for (i = 0; i < 24; i++)
    if (s[i] > '9') {
      s[i] += 'A' - '9' - 1;
    }
  // add termination character
  s[24] = '\0';
  return s;
}

/*
 *  Buffer to be used for control requests.
 *  (from libopencm3 examples)
 */
uint8_t usbd_control_buffer[128];

/**
 * CDC device control request
 * (from libopencm3 examples)
 */
static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
                                  uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
  (void)complete;
  (void)buf;
  (void)usbd_dev;

  switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
      /*
       * This Linux cdc_acm driver requires this to be implemented
       * even though it's optional in the CDC spec, and we don't
       * advertise it in the ACM functional descriptor.
       */
      char local_buf[10];
      struct usb_cdc_notification *notif = (void *)local_buf;

      /* We echo signals back to host as notification. */
      notif->bmRequestType = 0xA1;
      notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
      notif->wValue = 0;
      notif->wIndex = 0;
      notif->wLength = 2;
      local_buf[8] = req->wValue & 3;
      local_buf[9] = 0;
      usbd_ep_write_packet(usbd_dev, 0x83, local_buf, 10);
      return 1;
    }
    case USB_CDC_REQ_SET_LINE_CODING:
      if (*len < sizeof(struct usb_cdc_line_coding)) {
        return 0;
      }

      return 1;
    default:
      return 0;
  }
}

/**
 * RX callback for CDC device
 * (from libopencm3 examples)
 */
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;
  uint8_t buf[MAX_PACKET_SIZE];
  static int len = 0;
  // read packet
  len = usbd_ep_read_packet(usbd_dev, 0x01, buf, MAX_PACKET_SIZE);

  // write to fifo
  for (int i = 0; i < len; i++) {
    fifo_put(&rxfifo, buf[i]);
  }
}

// store USB connection status
static bool_t usb_connected;

// use suspend callback to detect disconnect
static void suspend_cb(void)
{
  usb_connected = FALSE;
}

/**
 * Set configuration and control callbacks for CDC device
 * (from libopencm3 examples)
 */
static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
  (void)wValue;

  usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, MAX_PACKET_SIZE, cdcacm_data_rx_cb);
  usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, MAX_PACKET_SIZE, NULL);
  usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

  usbd_register_control_callback(usbd_dev,
                                 USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                                 USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                                 cdcacm_control_request);

  // use config and suspend callback to detect connect
  usb_connected = TRUE;
  usbd_register_suspend_callback(usbd_dev, suspend_cb);
}


void fifo_init(fifo_t *fifo, uint8_t *buf)
{
  fifo->head = 0;
  fifo->tail = 0;
  fifo->buf = buf;
}



bool_t fifo_put(fifo_t *fifo, uint8_t c)
{
  int next;

  // check if FIFO has room
  next = (fifo->head + 1) % VCOM_FIFO_SIZE;
  if (next == fifo->tail) {
    // full
    return FALSE;
  }

  fifo->buf[fifo->head] = c;
  fifo->head = next;

  return TRUE;
}


bool_t fifo_get(fifo_t *fifo, uint8_t *pc)
{
  int next;

  // check if FIFO has data
  if (fifo->head == fifo->tail) {
    return FALSE;
  }

  next = (fifo->tail + 1) % VCOM_FIFO_SIZE;

  *pc = fifo->buf[fifo->tail];
  fifo->tail = next;

  return TRUE;
}


int fifo_avail(fifo_t *fifo)
{
  return (VCOM_FIFO_SIZE + fifo->head - fifo->tail) % VCOM_FIFO_SIZE;
}


int fifo_free(fifo_t *fifo)
{
  return (VCOM_FIFO_SIZE - 1 - fifo_avail(fifo));
}


/**
 * Writes one character to VCOM port fifo.
 *
 * Since we don't really have an instant feeedback from USB driver if
 * the character was written, we always return c if we are connected.
 *
 * @param [in] c character to write
 * @returns character to be written, -1 if not usb is not connected
 */
int VCOM_putchar(int c)
{
  if (usb_connected) {
    // check if there are at least two more bytes left in queue
    if (VCOM_check_free_space(2)) {
      // if yes, add char
      fifo_put(&txfifo, c);
    } else {
      // less than 2 bytes available, add byte and send data now
      fifo_put(&txfifo, c);
      VCOM_send_message();
    }
    return c;
  }
  return -1;
}

/**
 * Reads one character from VCOM port
 * @returns character read, or -1 if character could not be read
 */
int VCOM_getchar(void)
{
  static int result = 0;
  uint8_t c;
  result = fifo_get(&rxfifo, &c) ? c : -1;
  return result;
}

/**
 * Checks if buffer free in VCOM buffer
 *  @returns TRUE if len bytes are free
 */
bool_t VCOM_check_free_space(uint8_t len)
{
  return (fifo_free(&txfifo) >= len ? TRUE : FALSE);
}

/**
 * Checks if data available in VCOM buffer.
 * @returns nonzero if char is available in the queue, zero otherwise
 */
int VCOM_check_available(void)
{
  return (fifo_avail(&rxfifo));
}

/**
 * Poll usb (required by libopencm3).
 * VCOM_event() should be called from main/module event function
 */
void VCOM_event(void)
{
  usbd_poll(my_usbd_dev);
}

/**
 * Send data from fifo right now.
 * Only if usb is connected.
 */
void VCOM_send_message(void)
{
  if (usb_connected) {
    uint8_t buf[MAX_PACKET_SIZE];
    uint8_t i;
    for (i = 0; i < MAX_PACKET_SIZE; i++) {
      if (!fifo_get(&txfifo, &buf[i])) {
        break;
      }
	}
    usbd_ep_write_packet(my_usbd_dev, 0x82, buf, i);
  }
}


/*
 * USE_USB_LINE_CODING is not used in case of example1, example2 and telemetry
 */
#ifdef USE_USB_LINE_CODING
void VCOM_allow_linecoding(uint8_t mode __attribute__((unused))) {}
void VCOM_set_linecoding(uint8_t mode __attribute__((unused))) {}
#endif

/*
 * For USB telemetry & generic device API
 */
// Periph with generic device API
struct usb_serial_periph usb_serial;

// Functions for the generic device API
static int usb_serial_check_free_space(struct usb_serial_periph *p __attribute__((unused)),
                                       uint8_t len)
{
  return (int)VCOM_check_free_space(len);
}

static void usb_serial_transmit(struct usb_serial_periph *p __attribute__((unused)), uint8_t byte)
{
  VCOM_putchar(byte);
}

static void usb_serial_send(struct usb_serial_periph *p __attribute__((unused)))
{
  VCOM_send_message();
}

void VCOM_init(void)
{
  // initialise fifos
  fifo_init(&txfifo, txdata);
  fifo_init(&rxfifo, rxdata);

  /* set up GPIO pins */
#if defined STM32F4
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
                  GPIO11 | GPIO12);
  gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);
#endif

  /* USB clock */
  rcc_periph_clock_enable(RCC_OTGFS);

  /* Get serial number */
  get_dev_unique_id(serial_no);

  /* usb driver init*/
  my_usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
                          usb_strings, 3,
                          usbd_control_buffer, sizeof(usbd_control_buffer));

  usbd_register_set_config_callback(my_usbd_dev, cdcacm_set_config);

  // disconnected by default
  usb_connected = FALSE;

  // Configure generic device
  usb_serial.device.periph = (void *)(&usb_serial);
  usb_serial.device.check_free_space = (check_free_space_t) usb_serial_check_free_space;
  usb_serial.device.transmit = (transmit_t) usb_serial_transmit;
  usb_serial.device.send_message = (send_message_t) usb_serial_send;
}
