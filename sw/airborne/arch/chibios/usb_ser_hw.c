/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be ueful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/



#include <stdio.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "hal_serial_usb.h"
#include "mcu_periph/usb_serial.h"

#if HAL_USE_SERIAL_USB

#ifndef SERIAL_USB
#define SERIAL_USB USBD1
#endif

#ifndef USBD1_DATA_REQUEST_EP
#define USBD1_DATA_REQUEST_EP           1
#endif
#ifndef USBD1_DATA_AVAILABLE_EP
#define USBD1_DATA_AVAILABLE_EP         1
#endif
#ifndef USBD1_INTERRUPT_REQUEST_EP
#define USBD1_INTERRUPT_REQUEST_EP      2
#endif

#if (CH_KERNEL_MAJOR == 2)
  static EventListener inputAvailEL = {NULL, NULL, 0, 0};
#else
  static event_listener_t inputAvailEL = {NULL, NULL, 0, 0, 0};
#endif
/*===========================================================================*/
/* USB related stuff.                                                        */
/*===========================================================================*/



/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x02,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         0x0483,        /* idVendor (ST).                   */
                         0x5740,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[67] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
                         0x02,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0xC0,          /* bmAttributes (self powered).     */
                         50),           /* bMaxPower (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x01,          /* bNumEndpoints.                   */
                         0x02,          /* bInterfaceClass (Communications
                                           Interface Class, CDC section
                                           4.2).                            */
                         0x02,          /* bInterfaceSubClass (Abstract
                                         Control Model, CDC section 4.3).   */
                         0x01,          /* bInterfaceProtocol (AT commands,
                                           CDC section 4.4).                */
                         0),            /* iInterface.                      */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* bLength.                         */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                           Functional Descriptor.           */
  USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                           Control Management Descriptor).  */
  USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
  USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                           Class Interface).                */
  USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                           Interface).                      */
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_INTERRUPT_REQUEST_EP|0x80,
                         0x03,          /* bmAttributes (Interrupt).        */
                         0x0008,        /* wMaxPacketSize.                  */
                         0xFF),         /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x0A,          /* bInterfaceClass (Data Class
                                           Interface, CDC section 4.5).     */
                         0x00,          /* bInterfaceSubClass (CDC section
                                           4.6).                            */
                         0x00,          /* bInterfaceProtocol (CDC section
                                           4.7).                            */
                         0x00),         /* iInterface.                      */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_AVAILABLE_EP,     /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00),         /* bInterval.                       */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_REQUEST_EP|0x80,  /* bEndpointAddress.*/
                         0x02,          /* bmAttributes (Bulk).             */
                         0x0040,        /* wMaxPacketSize.                  */
                         0x00)          /* bInterval.                       */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t vcom_string1[] = {
  USB_DESC_BYTE(38),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
  'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
  'c', 0, 's', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(56),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0,
  'R', 0, 'T', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
  'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0,
  'o', 0, 'r', 0, 't', 0
};

/*
 * Serial Number string.
 */
static const uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};


static SerialUSBDriver SDU;

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}


/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   OUT EP1 state.
 */
static USBOutEndpointState ep1outstate;

/**
 * @brief   EP1 initialization structure (IN only).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  sduDataReceived,
  0x0040,
  0x0040,
  &ep1instate,
  &ep1outstate,
  2,
  NULL
};

/**
 * @brief   IN EP2 state.
 */
static USBInEndpointState ep2instate;

/**
 * @brief   EP2 initialization structure (IN only).
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  0x0010,
  0x0000,
  &ep2instate,
  NULL,
  1,
  NULL
};



/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {

  switch (event) {
  case USB_EVENT_RESET:
    return;
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();

    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &ep1config);
    usbInitEndpointI(usbp, USBD1_INTERRUPT_REQUEST_EP, &ep2config);

    /* Resetting the state of the CDC subsystem.*/
    sduConfigureHookI(&SDU);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_UNCONFIGURED:
    return;
  case USB_EVENT_SUSPEND:
    chSysLockFromISR();
    
    /* Disconnection event on suspend.*/
    sduSuspendHookI(&SDU);
    
    chSysUnlockFromISR();
    return;
 case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

/*
 * Handles the USB driver global events.
 */
static void sof_handler(USBDriver *usbp)
{
  (void)usbp;

  chSysLockFromISR();
  sduSOFHookI(&SDU);
  chSysUnlockFromISR();
}

/*
 * Serial over USB driver configuration.
 */
static const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  sduRequestsHook,
  sof_handler
};

static SerialUSBConfig serusbcfg ;

USBDriver *usbGetDriver (void)
{
  return serusbcfg.usbp;
}



bool isUsbConnected(void)
{
  static bool hasEverReceive = FALSE;

#if (CH_KERNEL_MAJOR <= 3)
  if (inputAvailEL.el_listener == NULL)
#else
  if (inputAvailEL.listener == NULL)
#endif
    return FALSE; 

  if (hasEverReceive) 
    return TRUE;
  else
    return ((hasEverReceive = chEvtGetAndClearFlags(&inputAvailEL)) != 0);
}




void usbSerialReset(SerialUSBDriver *sdu) 
{
  usbStop(serusbcfg.usbp);
  usbDisconnectBus(serusbcfg.usbp);
  sduStop (sdu);
  
  chThdSleepMilliseconds(10);
  sduStart(sdu, &serusbcfg);
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(900);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);
}


/*
 * For USB telemetry & generic device API
 */
// Periph with generic device API
struct usb_serial_periph usb_serial;

// Functions for the generic device API
static int usb_serial_check_free_space(struct usb_serial_periph *p __attribute__((unused)),
                                       long *fd __attribute__((unused)),
                                       uint16_t len)
{
  return len;
}

static void usb_serial_transmit(struct usb_serial_periph *p __attribute__((unused)),
                                long fd __attribute__((unused)),
                                uint8_t byte)
{
  streamPut(&SDU, byte);
}

static void usb_serial_transmit_buffer(struct usb_serial_periph *p __attribute__((unused)),
                                       long fd __attribute__((unused)),
                                       uint8_t *data, uint16_t len)
{
  streamWrite(&SDU, data, len);
}

static void usb_serial_send(struct usb_serial_periph *p __attribute__((unused)), long fd __attribute__((unused)))
{
  // TODO
}

static int usb_serial_char_available(struct usb_serial_periph *p __attribute__((unused)))
{
  // some bytes in the buffer
  if(p->nb_bytes > 0) {
    return p->nb_bytes;
  }

  uint16_t write_idx = (p->rx_read_idx + p->nb_bytes) % USB_RX_BUFFER_SIZE;

  // no bytes in the buffer, try to get one
  msg_t res = ibqGetTimeout(&SDU.ibqueue, TIME_IMMEDIATE);
  if(res >= 0) {
    // got a byte. put it in the buffer.
    p->rx_buf[write_idx] = (uint8_t) res;
    p->nb_bytes++;
    return p->nb_bytes;
  } else {
    // timeout or queue suspended. No data available.
    return 0;
  }
}

static uint8_t usb_serial_getch(struct usb_serial_periph *p __attribute__((unused)))
{
  if(p->nb_bytes > 0) {
    uint8_t ret = p->rx_buf[p->rx_read_idx];
    p->rx_read_idx = (p->rx_read_idx + 1) % USB_RX_BUFFER_SIZE;
    p->nb_bytes--;
    return ret;
  } else {
    // blocking get
    return streamGet(&SDU);
  }
}




/*
 * usb init
 */
void VCOM_init() 
{
  serusbcfg.usbp = &SERIAL_USB;
  serusbcfg.bulk_in = USBD1_DATA_REQUEST_EP;
  serusbcfg.bulk_out = USBD1_DATA_AVAILABLE_EP;
  serusbcfg.int_in = USBD1_INTERRUPT_REQUEST_EP;

  sduObjectInit(&SDU);
  sduStart(&SDU, &serusbcfg);

  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1000);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  chEvtRegisterMask(chnGetEventSource(&SDU), &inputAvailEL, CHN_INPUT_AVAILABLE);



  usb_serial.nb_bytes = 0;
  usb_serial.rx_read_idx = 0;
  usb_serial.reg_addr = &SDU;
  // Configure generic device
  usb_serial.device.periph = (void *)(&usb_serial);
  usb_serial.device.check_free_space = (check_free_space_t) usb_serial_check_free_space;
  usb_serial.device.put_byte = (put_byte_t) usb_serial_transmit;
  usb_serial.device.put_buffer = (put_buffer_t) usb_serial_transmit_buffer;
  usb_serial.device.send_message = (send_message_t) usb_serial_send;
  usb_serial.device.char_available = (char_available_t) usb_serial_char_available;
  usb_serial.device.get_byte = (get_byte_t) usb_serial_getch;

}


int  VCOM_putchar(int c) {
  if(!isUsbConnected()) {
    return -1;
  }
  streamPut(&SDU, c);
  return c;
  
}

int  VCOM_getchar(void) {
  return usb_serial_getch(&usb_serial);
}

int VCOM_peekchar(int ofs __attribute__((unused))) {
  // Dummy implementation
  return 0;
}

bool VCOM_check_free_space(uint16_t len __attribute__((unused))) {
  return true;
}

int VCOM_check_available(void) {
  return usb_serial_char_available(&usb_serial);
}

void VCOM_set_linecoding(uint8_t mode __attribute__((unused))) {}

void VCOM_allow_linecoding(uint8_t mode __attribute__((unused))) {}

void VCOM_send_message(void) {}

void VCOM_event(void) {}


#endif // HAL_USE_SERIAL_USB
