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

/**
 * Endpoints
*/
#ifndef USBD1_DATA_REQUEST_EP
#define USBD1_DATA_REQUEST_EP           1
#endif
#ifndef USBD1_DATA_AVAILABLE_EP
#define USBD1_DATA_AVAILABLE_EP         1
#endif
#ifndef USBD1_INTERRUPT_REQUEST_EP
#define USBD1_INTERRUPT_REQUEST_EP      2
#endif

/**
 * Interfaces
*/
#define USB_NUM_INTERFACES              2
#define USB_CDC_CIF_NUM0                0
#define USB_CDC_DIF_NUM0                1


//------------------------------ 2 serial USB --------------------------
#if USBD_NUMBER >= 2
#ifndef USBD2_DATA_REQUEST_EP
#define USBD2_DATA_REQUEST_EP           3
#endif
#ifndef USBD2_DATA_AVAILABLE_EP
#define USBD2_DATA_AVAILABLE_EP         3
#endif
#ifndef USBD2_INTERRUPT_REQUEST_EP
#define USBD2_INTERRUPT_REQUEST_EP      4
#endif
#undef  USB_NUM_INTERFACES
#define USB_NUM_INTERFACES              4
#define USB_CDC_CIF_NUM1                2
#define USB_CDC_DIF_NUM1                3
#endif


#define USB_INTERRUPT_REQUEST_SIZE      0x10
#define USB_DATA_SIZE                   0x40



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
  USB_DESC_DEVICE       (0x0200,        /* bcdUSB (1.1).                    */
    0xEF,                                   /* bDeviceClass (misc).         */
    0x02,                                   /* bDeviceSubClass (common).    */
    0x01,                                   /* bDeviceProtocol (IAD).       */
                         USB_DATA_SIZE, /* bMaxPacketSize.                  */
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

#define CDC_IF_DESC_SET_SIZE                                                \
  (USB_DESC_INTERFACE_SIZE + 5 + 5 + 4 + 5 + USB_DESC_ENDPOINT_SIZE +       \
   USB_DESC_INTERFACE_SIZE + (USB_DESC_ENDPOINT_SIZE * 2))

#define IAD_CDC_IF_DESC_SET_SIZE                                            \
  (USB_DESC_INTERFACE_ASSOCIATION_SIZE + CDC_IF_DESC_SET_SIZE)

#define CONFIG_DESC_DATA_SIZE (USB_DESC_CONFIGURATION_SIZE + (IAD_CDC_IF_DESC_SET_SIZE*USBD_NUMBER))

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[CONFIG_DESC_DATA_SIZE] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(
      CONFIG_DESC_DATA_SIZE,                /* wTotalLength.                    */
      USB_NUM_INTERFACES,                   /* bNumInterfaces.                  */
      0x01,                                 /* bConfigurationValue.             */
      0,                                    /* iConfiguration.                  */
      0x80,                                 /* bmAttributes (self powered).     */
      50),                                  /* bMaxPower (100mA).               */
                         

  /* Interface Descriptor. USB serial 1.*/
  USB_DESC_INTERFACE(
      USB_CDC_CIF_NUM0,                     /* bInterfaceNumber.                */
      0x00,                                 /* bAlternateSetting.               */
      0x01,                                 /* bNumEndpoints.                   */
      CDC_COMMUNICATION_INTERFACE_CLASS,    /* bInterfaceClass                  */
      CDC_ABSTRACT_CONTROL_MODEL,           /* bInterfaceSubClass               */
      0x00,                                 /* bInterfaceProtocol (CDC section 4.4)*/
      4),                                   /* iInterface: Descriptor index     */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE     (5),                    /* bLength.                         */
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE     (CDC_HEADER),           /* bDescriptorSubtype               */
  USB_DESC_BCD      (0x0110),               /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE     (5),                    /* bFunctionLength.                 */
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE     (CDC_CALL_MANAGEMENT),  /* bDescriptorSubtype               */
  USB_DESC_BYTE     (0x00),                 /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE     (USB_CDC_DIF_NUM0),     /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE     (4),                    /* bFunctionLength.                 */
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE     (CDC_ABSTRACT_CONTROL_MANAGEMENT), /* bDescriptorSubtype    */
  USB_DESC_BYTE     (0x02),                 /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/    
  USB_DESC_BYTE     (5),                    /* bFunctionLength.                 */
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE     (CDC_UNION),            /* bDescriptorSubtype               */
  USB_DESC_BYTE     (USB_CDC_CIF_NUM0),     /* bMasterInterface                 */
  USB_DESC_BYTE     (USB_CDC_DIF_NUM0),     /* bSlaveInterface0                 */
  /* Endpoint, Interrupt IN.*/
  USB_DESC_ENDPOINT(
      USB_ENDPOINT_IN(USBD1_INTERRUPT_REQUEST_EP),
      USB_EP_MODE_TYPE_INTR,                /* bmAttributes (Interrupt).        */
      USB_INTERRUPT_REQUEST_SIZE,           /* wMaxPacketSize.                  */
      0xFF),                                /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE(
      USB_CDC_DIF_NUM0,                     /* bInterfaceNumber.                */
      0x00,                                 /* bAlternateSetting.               */
      0x02,                                 /* bNumEndpoints.                   */
      CDC_DATA_INTERFACE_CLASS,             /* bInterfaceClass                  */
      0x00,                                 /* bInterfaceSubClass (CDC sec. 4.6)*/
      0x00,                                 /* bInterfaceProtocol (CDC sec. 4.7)*/
      0x00),                                /* iInterface.                      */
  /* Endpoint, Bulk OUT.*/
  USB_DESC_ENDPOINT(
      USB_ENDPOINT_OUT(USBD1_DATA_AVAILABLE_EP),  /* bEndpointAddress.          */
      USB_EP_MODE_TYPE_BULK,                /* bmAttributes (Bulk).             */
      USB_DATA_SIZE,                        /* wMaxPacketSize.                  */
      0x00),                                /* bInterval.                       */
  /* Endpoint, Bulk IN*/
  USB_DESC_ENDPOINT(
      USB_ENDPOINT_IN(USBD1_DATA_REQUEST_EP),   /* bEndpointAddress.            */
      USB_EP_MODE_TYPE_BULK,                /* bmAttributes (Bulk).             */
      USB_DATA_SIZE,                        /* wMaxPacketSize.                  */
      0x00),                                /* bInterval.                       */



#if USBD_NUMBER >= 2
  /* Interface Descriptor. USB serial 2.*/
  USB_DESC_INTERFACE(
      USB_CDC_CIF_NUM1,                     /* bInterfaceNumber.                */
      0x00,                                 /* bAlternateSetting.               */
      0x01,                                 /* bNumEndpoints.                   */
      CDC_COMMUNICATION_INTERFACE_CLASS,    /* bInterfaceClass                  */
      CDC_ABSTRACT_CONTROL_MODEL,           /* bInterfaceSubClass               */
      0x00,                                 /* bInterfaceProtocol (CDC section 4.4)*/
      5),                                   /* iInterface: Descriptor index     */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE     (5),                    /* bLength.                         */
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE     (CDC_HEADER),           /* bDescriptorSubtype               */
  USB_DESC_BCD      (0x0110),               /* bcdCDC.                          */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE     (5),                    /* bFunctionLength.                 */
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE     (CDC_CALL_MANAGEMENT),  /* bDescriptorSubtype               */
  USB_DESC_BYTE     (0x00),                 /* bmCapabilities (D0+D1).          */
  USB_DESC_BYTE     (USB_CDC_DIF_NUM1),     /* bDataInterface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE     (4),                    /* bFunctionLength.                 */
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE     (CDC_ABSTRACT_CONTROL_MANAGEMENT), /* bDescriptorSubtype    */
  USB_DESC_BYTE     (0x02),                 /* bmCapabilities.                  */
  /* Union Functional Descriptor.*/    
  USB_DESC_BYTE     (5),                    /* bFunctionLength.                 */
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType (CS_INTERFACE).  */
  USB_DESC_BYTE     (CDC_UNION),            /* bDescriptorSubtype               */
  USB_DESC_BYTE     (USB_CDC_CIF_NUM1),     /* bMasterInterface                 */
  USB_DESC_BYTE     (USB_CDC_DIF_NUM1),     /* bSlaveInterface0                 */
  /* Endpoint, Interrupt IN.*/
  USB_DESC_ENDPOINT(
      USB_ENDPOINT_IN(USBD2_INTERRUPT_REQUEST_EP),
      USB_EP_MODE_TYPE_INTR,                /* bmAttributes (Interrupt).        */
      USB_INTERRUPT_REQUEST_SIZE,           /* wMaxPacketSize.                  */
      0xFF),                                /* bInterval.                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE(
      USB_CDC_DIF_NUM1,                     /* bInterfaceNumber.                */
      0x00,                                 /* bAlternateSetting.               */
      0x02,                                 /* bNumEndpoints.                   */
      CDC_DATA_INTERFACE_CLASS,             /* bInterfaceClass                  */
      0x00,                                 /* bInterfaceSubClass (CDC sec. 4.6)*/
      0x00,                                 /* bInterfaceProtocol (CDC sec. 4.7)*/
      0x00),                                /* iInterface.                      */
  /* Endpoint, Bulk OUT.*/
  USB_DESC_ENDPOINT(
      USB_ENDPOINT_OUT(USBD2_DATA_AVAILABLE_EP),  /* bEndpointAddress.          */
      USB_EP_MODE_TYPE_BULK,                /* bmAttributes (Bulk).             */
      USB_DATA_SIZE,                        /* wMaxPacketSize.                  */
      0x00),                                /* bInterval.                       */
  /* Endpoint, Bulk IN*/
  USB_DESC_ENDPOINT(
      USB_ENDPOINT_IN(USBD2_DATA_REQUEST_EP),   /* bEndpointAddress.            */
      USB_EP_MODE_TYPE_BULK,                /* bmAttributes (Bulk).             */
      USB_DATA_SIZE,                        /* wMaxPacketSize.                  */
      0x00)                                 /* bInterval.                       */
#endif
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
  USB_DESC_BYTE(28),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'P', 0, 'a', 0, 'p', 0, 'a', 0, 'r', 0, 'a', 0, 'z', 0, 'z', 0,
  'i', 0, ' ', 0, 'U', 0, 'A', 0, 'V', 0
};

/*
 * Device Description string.
 */
static const uint8_t vcom_string2[] = {
  USB_DESC_BYTE(34),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'C', 0, 'D', 0, 'C', 0, ' ', 0, 'S', 0, 'e', 0, 'r', 0, 'i', 0,
  'a', 0, 'l', 0, ' ', 0, 'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0
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

/**
 * interface ID ?
*/
static const uint8_t vcom_string4[] = {
  USB_DESC_BYTE(58),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'P', 0, 'a', 0, 'p', 0, 'a', 0, 'r', 0, 'a', 0, 'z', 0, 'z', 0,
  'i', 0, ' ', 0, 'U', 0, 'A', 0, 'V', 0, ' ', 0, 't', 0, 'e', 0,
  'l', 0, 'e', 0, 'm', 0, 'e', 0, 't', 0, 'r', 0, 'y', 0, ' ', 0,
  'P', 0, 'o', 0, 'r', 0, 't', 0
};

#if USBD_NUMBER >= 2
/**
 * interface ID ?
*/
static const uint8_t vcom_string5[] = {
  USB_DESC_BYTE(54),                    /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'P', 0, 'a', 0, 'p', 0, 'a', 0, 'r', 0, 'a', 0, 'z', 0, 'z', 0,
  'i', 0, ' ', 0, 'U', 0, 'A', 0, 'V', 0, ' ', 0, 'd', 0, 'e', 0,
  'b', 0, 'u', 0, 'g', 0, ' ', 0, 'P', 0, 'o', 0, 'r', 0, 't', 0
};
#endif

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3},
  {sizeof vcom_string4, vcom_string4},
#if USBD_NUMBER >= 2
  {sizeof vcom_string5, vcom_string5}
#endif
};


static SerialUSBDriver SDU1;
#if USBD_NUMBER >= 2
static SerialUSBDriver SDU2;
#endif

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
    if (dindex < 6)
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
  USB_DATA_SIZE,
  USB_DATA_SIZE,
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
  USB_INTERRUPT_REQUEST_SIZE,
  0x0000,
  &ep2instate,
  NULL,
  1,
  NULL
};

#if USBD_NUMBER >= 2
/**
 * @brief   IN EP3 state.
 */
static USBInEndpointState ep3instate;

/**
 * @brief   OUT EP3 state.
 */
static USBOutEndpointState ep3outstate;

/**
 * @brief   EP3 initialization structure (IN only).
 */
static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  sduDataReceived,
  USB_DATA_SIZE,
  USB_DATA_SIZE,
  &ep3instate,
  &ep3outstate,
  2,
  NULL
};

/**
 * @brief   IN EP4 state.
 */
static USBInEndpointState ep4instate;

/**
 * @brief   EP4 initialization structure (IN only).
 */
static const USBEndpointConfig ep4config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  USB_INTERRUPT_REQUEST_SIZE,
  0x0000,
  &ep4instate,
  NULL,
  1,
  NULL
};
#endif



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

    
    if (usbp->state == USB_ACTIVE) {
      /* Enables the endpoints specified into the configuration.
         Note, this callback is invoked from an ISR so I-Class functions
         must be used.*/
      usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &ep1config);
      usbInitEndpointI(usbp, USBD1_INTERRUPT_REQUEST_EP, &ep2config);
#if USBD_NUMBER >= 2
      usbInitEndpointI(usbp, USBD2_DATA_REQUEST_EP, &ep3config);
      usbInitEndpointI(usbp, USBD2_INTERRUPT_REQUEST_EP, &ep4config);
#endif
      /* Resetting the state of the CDC subsystem.*/
      sduConfigureHookI(&SDU1);
#if USBD_NUMBER >= 2
      sduConfigureHookI(&SDU2);
#endif
    }
    else if (usbp->state == USB_SELECTED) {
      usbDisableEndpointsI(usbp);
    }

    chSysUnlockFromISR();
    return;
  case USB_EVENT_UNCONFIGURED:
    return;
  case USB_EVENT_SUSPEND:
    chSysLockFromISR();
    
    /* Disconnection event on suspend.*/
    sduSuspendHookI(&SDU1);
#if USBD_NUMBER >= 2
    sduSuspendHookI(&SDU2);
#endif
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
  sduSOFHookI(&SDU1);
#if USBD_NUMBER >= 2
  sduSOFHookI(&SDU2);
#endif
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

static SerialUSBConfig serusbcfg1;
#if USBD_NUMBER >= 2
static SerialUSBConfig serusbcfg2;
#endif

USBDriver *usbGetDriver (void)
{
  return serusbcfg1.usbp;
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
  usbStop(serusbcfg1.usbp);
  usbDisconnectBus(serusbcfg1.usbp);
  sduStop (sdu);
  
  chThdSleepMilliseconds(10);
  sduStart(sdu, &serusbcfg1);
  usbDisconnectBus(serusbcfg1.usbp);
  chThdSleepMilliseconds(900);
  usbStart(serusbcfg1.usbp, &usbcfg);
  usbConnectBus(serusbcfg1.usbp);
}


/*
 * For USB telemetry & generic device API
 */
// Periph with generic device API
struct usb_serial_periph usb_serial;
#if USBD_NUMBER >= 2
struct usb_serial_periph usb_serial_debug;
#endif

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
  obqPutTimeout(&((SerialUSBDriver *)p->reg_addr)->obqueue, byte, TIME_IMMEDIATE);
}

static void usb_serial_transmit_buffer(struct usb_serial_periph *p __attribute__((unused)),
                                       long fd __attribute__((unused)),
                                       uint8_t *data, uint16_t len)
{
  obqWriteTimeout(&((SerialUSBDriver *)p->reg_addr)->obqueue, data, len, TIME_IMMEDIATE);
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
  msg_t res = ibqGetTimeout(&(((SerialUSBDriver*)(p->reg_addr))->ibqueue), TIME_IMMEDIATE);
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
    return ibqGetTimeout(&((SerialUSBDriver *)p->reg_addr)->ibqueue, TIME_IMMEDIATE);
  }
}




/*
 * usb init
 */
void VCOM_init() 
{
  serusbcfg1.usbp = &SERIAL_USB;
  serusbcfg1.bulk_in = USBD1_DATA_REQUEST_EP;
  serusbcfg1.bulk_out = USBD1_DATA_AVAILABLE_EP;
  serusbcfg1.int_in = USBD1_INTERRUPT_REQUEST_EP;
#if USBD_NUMBER >= 2
  serusbcfg2.usbp = &SERIAL_USB;
  serusbcfg2.bulk_in = USBD2_DATA_REQUEST_EP;
  serusbcfg2.bulk_out = USBD2_DATA_AVAILABLE_EP;
  serusbcfg2.int_in = USBD2_INTERRUPT_REQUEST_EP;
#endif

  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg1);
#if USBD_NUMBER >= 2
  sduObjectInit(&SDU2);
  sduStart(&SDU2, &serusbcfg2);
#endif

  usbDisconnectBus(serusbcfg1.usbp);
  chThdSleepMilliseconds(1000);
  usbStart(serusbcfg1.usbp, &usbcfg);
  usbConnectBus(serusbcfg1.usbp);

  chEvtRegisterMask(chnGetEventSource(&SDU1), &inputAvailEL, CHN_INPUT_AVAILABLE);



  usb_serial.nb_bytes = 0;
  usb_serial.rx_read_idx = 0;
  usb_serial.reg_addr = &SDU1;
  // Configure generic device
  usb_serial.device.periph = (void *)(&usb_serial);
  usb_serial.device.check_free_space = (check_free_space_t) usb_serial_check_free_space;
  usb_serial.device.put_byte = (put_byte_t) usb_serial_transmit;
  usb_serial.device.put_buffer = (put_buffer_t) usb_serial_transmit_buffer;
  usb_serial.device.send_message = (send_message_t) usb_serial_send;
  usb_serial.device.char_available = (char_available_t) usb_serial_char_available;
  usb_serial.device.get_byte = (get_byte_t) usb_serial_getch;


#if USBD_NUMBER >= 2
  usb_serial_debug.nb_bytes = 0;
  usb_serial_debug.rx_read_idx = 0;
  usb_serial_debug.reg_addr = &SDU2;
  // Configure generic device
  usb_serial_debug.device.periph = (void *)(&usb_serial_debug);
  usb_serial_debug.device.check_free_space = (check_free_space_t) usb_serial_check_free_space;
  usb_serial_debug.device.put_byte = (put_byte_t) usb_serial_transmit;
  usb_serial_debug.device.put_buffer = (put_buffer_t) usb_serial_transmit_buffer;
  usb_serial_debug.device.send_message = (send_message_t) usb_serial_send;
  usb_serial_debug.device.char_available = (char_available_t) usb_serial_char_available;
  usb_serial_debug.device.get_byte = (get_byte_t) usb_serial_getch;
#endif
}


int  VCOM_putchar(int c) {
  if(!isUsbConnected()) {
    return -1;
  }
  streamPut((SerialUSBDriver*)(usb_serial.reg_addr), c);
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
