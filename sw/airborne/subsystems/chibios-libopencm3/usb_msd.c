/*
 * Copyright (C) 2014 Gautier Hattenberger, Alexandre Bustico
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
 */

/*
 * @file subsystems/chibios-libopencm3/usb_msd.c
 *
 */

#include "usb_msd.h"

/* Request types */
#define MSD_REQ_RESET   0xFF
#define MSD_GET_MAX_LUN 0xFE

/* CBW/CSW block signatures */
#define MSD_CBW_SIGNATURE 0x43425355
#define MSD_CSW_SIGNATURE 0x53425355

/* Setup packet access macros */
#define MSD_SETUP_WORD(setup, index) (uint16_t)(((uint16_t)setup[index + 1] << 8) | (setup[index] & 0x00FF))
#define MSD_SETUP_VALUE(setup)       MSD_SETUP_WORD(setup, 2)
#define MSD_SETUP_INDEX(setup)       MSD_SETUP_WORD(setup, 4)
#define MSD_SETUP_LENGTH(setup)      MSD_SETUP_WORD(setup, 6)

/* Command statuses */
#define MSD_COMMAND_PASSED      0x00
#define MSD_COMMAND_FAILED      0x01
#define MSD_COMMAND_PHASE_ERROR 0x02

/* SCSI commands */
#define SCSI_CMD_TEST_UNIT_READY              0x00
#define SCSI_CMD_REQUEST_SENSE                0x03
#define SCSI_CMD_FORMAT_UNIT                  0x04
#define SCSI_CMD_INQUIRY                      0x12
#define SCSI_CMD_MODE_SENSE_6                 0x1A
#define SCSI_CMD_START_STOP_UNIT              0x1B
#define SCSI_CMD_SEND_DIAGNOSTIC              0x1D
#define SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL 0x1E
#define SCSI_CMD_READ_FORMAT_CAPACITIES       0x23
#define SCSI_CMD_READ_CAPACITY_10             0x25
#define SCSI_CMD_READ_10                      0x28
#define SCSI_CMD_WRITE_10                     0x2A
#define SCSI_CMD_VERIFY_10                    0x2F

/* SCSI sense keys */
#define SCSI_SENSE_KEY_GOOD                            0x00
#define SCSI_SENSE_KEY_RECOVERED_ERROR                 0x01
#define SCSI_SENSE_KEY_NOT_READY                       0x02
#define SCSI_SENSE_KEY_MEDIUM_ERROR                    0x03
#define SCSI_SENSE_KEY_HARDWARE_ERROR                  0x04
#define SCSI_SENSE_KEY_ILLEGAL_REQUEST                 0x05
#define SCSI_SENSE_KEY_UNIT_ATTENTION                  0x06
#define SCSI_SENSE_KEY_DATA_PROTECT                    0x07
#define SCSI_SENSE_KEY_BLANK_CHECK                     0x08
#define SCSI_SENSE_KEY_VENDOR_SPECIFIC                 0x09
#define SCSI_SENSE_KEY_COPY_ABORTED                    0x0A
#define SCSI_SENSE_KEY_ABORTED_COMMAND                 0x0B
#define SCSI_SENSE_KEY_VOLUME_OVERFLOW                 0x0D
#define SCSI_SENSE_KEY_MISCOMPARE                      0x0E

#define SCSI_ASENSE_NO_ADDITIONAL_INFORMATION          0x00
#define SCSI_ASENSE_WRITE_FAULT                        0x03
#define SCSI_ASENSE_LOGICAL_UNIT_NOT_READY             0x04
#define SCSI_ASENSE_READ_ERROR                         0x11
#define SCSI_ASENSE_INVALID_COMMAND                    0x20
#define SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE 0x21
#define SCSI_ASENSE_INVALID_FIELD_IN_CDB               0x24
#define SCSI_ASENSE_WRITE_PROTECTED                    0x27
#define SCSI_ASENSE_NOT_READY_TO_READY_CHANGE          0x28
#define SCSI_ASENSE_FORMAT_ERROR                       0x31
#define SCSI_ASENSE_MEDIUM_NOT_PRESENT                 0x3A

#define SCSI_ASENSEQ_NO_QUALIFIER                      0x00
#define SCSI_ASENSEQ_FORMAT_COMMAND_FAILED             0x01
#define SCSI_ASENSEQ_INITIALIZING_COMMAND_REQUIRED     0x02
#define SCSI_ASENSEQ_OPERATION_IN_PROGRESS             0x07

/**
 * @brief Response to a READ_CAPACITY_10 SCSI command
 */
PACK_STRUCT_BEGIN typedef struct {
  uint32_t last_block_addr;
  uint32_t block_size;
} PACK_STRUCT_STRUCT msd_scsi_read_capacity_10_response_t PACK_STRUCT_END;

/**
 * @brief Response to a READ_FORMAT_CAPACITIES SCSI command
 */
PACK_STRUCT_BEGIN typedef struct {
  uint8_t reserved[3];
  uint8_t capacity_list_length;
  uint32_t block_count;
  uint32_t desc_and_block_length;
} PACK_STRUCT_STRUCT msd_scsi_read_format_capacities_response_t PACK_STRUCT_END;

/**
 * @brief   Read-write buffers (TODO: need a way of specifying the size of this)
 */
static uint8_t rw_buf[2][512];

typedef uint32_t DWORD __attribute__((__may_alias__));;
typedef uint16_t WORD  __attribute__((__may_alias__));


/**
 * @brief Byte-swap a 32 bits unsigned integer
 */
static inline DWORD swap_uint32(DWORD  val );

static inline DWORD swap_uint32(DWORD val ) {
  val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
  return ((val << 16) & 0xFFFF0000) | ((val >> 16) & 0x0000FFFF);
}
/**
 * @brief Byte-swap a 16 bits unsigned integer
 */
#define swap_uint16(x) (((((WORD)(x)) >> 8) & 0xff) | ((((WORD)(x)) & 0xff) << 8))

static void msd_handle_end_point_notification(USBDriver *usbp, usbep_t ep);

/**
 * @brief IN end-point 1 state
 */
static USBInEndpointState ep1_in_state;

/**
 * @brief OUT end-point 1 state
 */
static USBOutEndpointState ep1_out_state;

/**
 * @brief End-point 1 initialization structure
 */
static const USBEndpointConfig ep_data_config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  msd_handle_end_point_notification,
  msd_handle_end_point_notification,
  64,
  64,
  &ep1_in_state,
  &ep1_out_state,
  1,
  NULL
};

/**
 * @brief   USB device configured handler.
 *
 * @param[in] msdp      pointer to the @p USBMassStorageDriver object
 *
 * @iclass
 */
void msdConfigureHookI(USBMassStorageDriver *msdp)
{
  usbInitEndpointI(msdp->config->usbp, msdp->config->bulk_ep, &ep_data_config);
  chBSemSignalI(&msdp->bsem);
  chEvtBroadcastI(&msdp->evt_connected);
}

/**
 * @brief   Default requests hook.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @return              The hook status.
 * @retval TRUE         Message handled internally.
 * @retval FALSE        Message not handled.
 */
bool msdRequestsHook(USBDriver *usbp) {

  /* check that the request is of type Class / Interface */
  if (((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS) &&
      ((usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK) == USB_RTYPE_RECIPIENT_INTERFACE)) {

    /* check that the request is for interface 0 */
    if (MSD_SETUP_INDEX(usbp->setup) != 0)
      return false;

    /* act depending on bRequest = setup[1] */
    switch (usbp->setup[1]) {
      case MSD_REQ_RESET:
        /* check that it is a HOST2DEV request */
        if (((usbp->setup[0] & USB_RTYPE_DIR_MASK) != USB_RTYPE_DIR_HOST2DEV) ||
            (MSD_SETUP_LENGTH(usbp->setup) != 0) ||
            (MSD_SETUP_VALUE(usbp->setup) != 0))
        {
          return false;
        }

        /* reset all endpoints */
        /* TODO!*/
        /* The device shall NAK the status stage of the device request until
         * the Bulk-Only Mass Storage Reset is complete.
         */
        return true;
      case MSD_GET_MAX_LUN:
        /* check that it is a DEV2HOST request */
        if (((usbp->setup[0] & USB_RTYPE_DIR_MASK) != USB_RTYPE_DIR_DEV2HOST) ||
            (MSD_SETUP_LENGTH(usbp->setup) != 1) ||
            (MSD_SETUP_VALUE(usbp->setup) != 0))
        {
          return false;
        }

        static uint8_t len_buf[1] = {0};
        /* stall to indicate that we don't support LUN */
        usbSetupTransfer(usbp, len_buf, 1, NULL);
        return true;
      default:
        return false;
        break;
    }
  }

  return false;
}

/**
 * @brief Wait until the end-point interrupt handler has been called
 */
static void msd_wait_for_isr(USBMassStorageDriver *msdp) {

  /* sleep until it completes */
  chSysLock();
  chBSemWaitS(&msdp->bsem);
  chSysUnlock();
}

/**
 * @brief Called when data can be read or written on the endpoint -- wakes the thread up
 */
static void msd_handle_end_point_notification(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  (void)ep;

  chSysLockFromIsr();
  chBSemSignalI(&((USBMassStorageDriver *)usbp->in_params[ep])->bsem);
  chSysUnlockFromIsr();
}

/**
 * @brief Starts sending data
 */
static void msd_start_transmit(USBMassStorageDriver *msdp, const uint8_t* buffer, size_t size) {

  usbPrepareTransmit(msdp->config->usbp, msdp->config->bulk_ep, buffer, size);
  chSysLock();
  usbStartTransmitI(msdp->config->usbp, msdp->config->bulk_ep);
  chSysUnlock();
}

/**
 * @brief Starts receiving data
 */
static void msd_start_receive(USBMassStorageDriver *msdp, uint8_t* buffer, size_t size) {

  usbPrepareReceive(msdp->config->usbp, msdp->config->bulk_ep, buffer, size);
  chSysLock();
  usbStartReceiveI(msdp->config->usbp, msdp->config->bulk_ep);
  chSysUnlock();
}

/**
 * @brief Changes the SCSI sense information
 */
static inline void msd_scsi_set_sense(USBMassStorageDriver *msdp, uint8_t key, uint8_t acode, uint8_t aqual) {
  msdp->sense.byte[2] = key;
  msdp->sense.byte[12] = acode;
  msdp->sense.byte[13] = aqual;
}

/**
 * @brief Processes an INQUIRY SCSI command
 */
bool msd_scsi_process_inquiry(USBMassStorageDriver *msdp) {

  msd_cbw_t *cbw = &(msdp->cbw);

  /* check the EVPD bit (Vital Product Data) */
  if (cbw->scsi_cmd_data[1] & 0x01) {

    /* check the Page Code byte to know the type of product data to reply */
    switch (cbw->scsi_cmd_data[2]) {

      /* unit serial number */
      case 0x80: {
                   uint8_t response[] = {'0'}; /* TODO */
                   msd_start_transmit(msdp, response, sizeof(response));
                   msdp->result = true;

                   /* wait for ISR */
                   return true;
                 }

                 /* unhandled */
      default:
                 msd_scsi_set_sense(msdp,
                     SCSI_SENSE_KEY_ILLEGAL_REQUEST,
                     SCSI_ASENSE_INVALID_FIELD_IN_CDB,
                     SCSI_ASENSEQ_NO_QUALIFIER);
                 return false;
    }
  }
  else
  {
    msd_start_transmit(msdp, (const uint8_t *)&msdp->inquiry, sizeof(msdp->inquiry));
    msdp->result = true;

    /* wait for ISR */
    return true;
  }
}

/**
 * @brief Processes a REQUEST_SENSE SCSI command
 */
bool msd_scsi_process_request_sense(USBMassStorageDriver *msdp) {

  msd_start_transmit(msdp, (const uint8_t *)&msdp->sense, sizeof(msdp->sense));
  msdp->result = true;

  /* wait for ISR immediately, otherwise the caller may reset the sense bytes before they are sent to the host! */
  msd_wait_for_isr(msdp);

  /* ... don't wait for ISR, we just did it */
  return false;
}

/**
 * @brief Processes a READ_CAPACITY_10 SCSI command
 */
bool msd_scsi_process_read_capacity_10(USBMassStorageDriver *msdp) {

  static msd_scsi_read_capacity_10_response_t response;

  response.block_size = swap_uint32(msdp->block_dev_info.blk_size);
  response.last_block_addr = swap_uint32(msdp->block_dev_info.blk_num-1);

  msd_start_transmit(msdp, (const uint8_t *)&response, sizeof(response));
  msdp->result = true;

  /* wait for ISR */
  return true;
}

/**
 * @brief Processes a SEND_DIAGNOSTIC SCSI command
 */
bool msd_scsi_process_send_diagnostic(USBMassStorageDriver *msdp) {

  msd_cbw_t *cbw = &(msdp->cbw);

  if (!(cbw->scsi_cmd_data[1] & (1 << 2))) {
    /* only self-test supported - update SENSE key and fail the command */
    msd_scsi_set_sense(msdp,
        SCSI_SENSE_KEY_ILLEGAL_REQUEST,
        SCSI_ASENSE_INVALID_FIELD_IN_CDB,
        SCSI_ASENSEQ_NO_QUALIFIER);
    msdp->result = false;
    return false;
  }

  /* TODO: actually perform the test */
  msdp->result = true;

  /* don't wait for ISR */
  return false;
}

/**
 * @brief Processes a READ_WRITE_10 SCSI command
 */
bool msd_scsi_process_start_read_write_10(USBMassStorageDriver *msdp) {

  msd_cbw_t *cbw = &(msdp->cbw);

  if ((cbw->scsi_cmd_data[0] == SCSI_CMD_WRITE_10) && blkIsWriteProtected(msdp->config->bbdp)) {
    /* device is write protected and a write has been issued */
    /* block address is invalid, update SENSE key and return command fail */
    msd_scsi_set_sense(msdp,
        SCSI_SENSE_KEY_DATA_PROTECT,
        SCSI_ASENSE_WRITE_PROTECTED,
        SCSI_ASENSEQ_NO_QUALIFIER);
    msdp->result = false;

    /* don't wait for ISR */
    return false;
  }

  uint32_t rw_block_address = swap_uint32(*(DWORD *)&cbw->scsi_cmd_data[2]);
  uint16_t total = swap_uint16(*(WORD *)&cbw->scsi_cmd_data[7]);
  uint16_t i = 0;

  if (rw_block_address >= msdp->block_dev_info.blk_num) {
    /* block address is invalid, update SENSE key and return command fail */
    msd_scsi_set_sense(msdp,
        SCSI_SENSE_KEY_ILLEGAL_REQUEST,
        SCSI_ASENSE_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE,
        SCSI_ASENSEQ_NO_QUALIFIER);
    msdp->result = false;

    /* don't wait for ISR */
    return false;
  }

  if (cbw->scsi_cmd_data[0] == SCSI_CMD_WRITE_10) {
    /* process a write command */

    /* get the first packet */
    msd_start_receive(msdp, rw_buf[i % 2], msdp->block_dev_info.blk_size);
    msd_wait_for_isr(msdp);

    /* loop over each block */
    for (i = 0; i < total; i++) {

      if (i < (total - 1)) {
        /* there is at least one block of data left to be read over USB */
        /* queue this read before issuing the blocking write */
        msd_start_receive(msdp, rw_buf[(i + 1) % 2], msdp->block_dev_info.blk_size);
      }

      /* now write the block to the block device */
      if (blkWrite(msdp->config->bbdp, rw_block_address++, rw_buf[i % 2], 1) == CH_FAILED) {
        /* write failed */
        msd_scsi_set_sense(msdp,
            SCSI_SENSE_KEY_MEDIUM_ERROR,
            SCSI_ASENSE_WRITE_FAULT,
            SCSI_ASENSEQ_NO_QUALIFIER);
        msdp->result = false;

        /* don't wait for ISR */
        return false;
      }

      if (i < (total - 1)) {
        /* now wait for the USB event to complete */
        msd_wait_for_isr(msdp);
      }
    }
  } else {
    /* process a read command */

    i = 0;

    /* read the first block from block device */
    if (blkRead(msdp->config->bbdp, rw_block_address++, rw_buf[i % 2], 1) == CH_FAILED) {
      /* read failed */
      msd_scsi_set_sense(msdp,
          SCSI_SENSE_KEY_MEDIUM_ERROR,
          SCSI_ASENSE_READ_ERROR,
          SCSI_ASENSEQ_NO_QUALIFIER);
      msdp->result = false;

      /* don't wait for ISR */
      return false;
    }

    /* loop over each block */
    for (i = 0; i < total; i++) {
      /* transmit the block */
      msd_start_transmit(msdp, rw_buf[i % 2], msdp->block_dev_info.blk_size);

      if (i < (total - 1)) {
        /* there is at least one more block to be read from device */
        /* so read that whilst the USB transfer takes place */
        if (blkRead(msdp->config->bbdp, rw_block_address++, rw_buf[(i + 1) % 2], 1) == CH_FAILED) {
          /* read failed */
          msd_scsi_set_sense(msdp,
              SCSI_SENSE_KEY_MEDIUM_ERROR,
              SCSI_ASENSE_READ_ERROR,
              SCSI_ASENSEQ_NO_QUALIFIER);
          msdp->result = false;

          /* wait for ISR (the previous transmission is still running) */
          return true;
        }
      }

      /* wait for the USB event to complete */
      msd_wait_for_isr(msdp);
    }
  }

  msdp->result = true;

  /* don't wait for ISR */
  return false;
}

/**
 * @brief Processes a START_STOP_UNIT SCSI command
 */
bool msd_scsi_process_start_stop_unit(USBMassStorageDriver *msdp) {

  if ((msdp->cbw.scsi_cmd_data[4] & 0x03) == 0x02) {
    /* device has been ejected */
    chEvtBroadcast(&msdp->evt_ejected);
    msdp->state = MSD_EJECTED;
  }

  msdp->result = true;

  /* don't wait for ISR */
  return false;
}

/**
 * @brief Processes a MODE_SENSE_6 SCSI command
 */
bool msd_scsi_process_mode_sense_6(USBMassStorageDriver *msdp) {

  static uint8_t response[4] = {
    0x03, /* number of bytes that follow                    */
    0x00, /* medium type is SBC                             */
    0x00, /* not write protected (TODO handle it correctly) */
    0x00  /* no block descriptor                            */
  };

  msd_start_transmit(msdp, response, sizeof(response));
  msdp->result = true;

  /* wait for ISR */
  return true;
}

/**
 * @brief Processes a READ_FORMAT_CAPACITIES SCSI command
 */
bool msd_scsi_process_read_format_capacities(USBMassStorageDriver *msdp) {

  msd_scsi_read_format_capacities_response_t response;
  response.capacity_list_length = 1;
  response.block_count = swap_uint32(msdp->block_dev_info.blk_num);
  response.desc_and_block_length = swap_uint32((0x02 << 24) | (msdp->block_dev_info.blk_size & 0x00FFFFFF));

  msd_start_transmit(msdp, (const uint8_t*)&response, sizeof(response));
  msdp->result = true;

  /* wait for ISR */
  return true;
}

/**
 * @brief Processes a TEST_UNIT_READY SCSI command
 */
bool msd_scsi_process_test_unit_ready(USBMassStorageDriver *msdp) {

  if (blkIsInserted(msdp->config->bbdp)) {
    /* device inserted and ready */
    msdp->result = true;
  } else {
    /* device not present or not ready */
    msd_scsi_set_sense(msdp,
        SCSI_SENSE_KEY_NOT_READY,
        SCSI_ASENSE_MEDIUM_NOT_PRESENT,
        SCSI_ASENSEQ_NO_QUALIFIER);
    msdp->result = false;
  }

  /* don't wait for ISR */
  return false;
}

/**
 * @brief Waits for a new command block
 */
bool msd_wait_for_command_block(USBMassStorageDriver *msdp) {

  msd_start_receive(msdp, (uint8_t *)&msdp->cbw, sizeof(msdp->cbw));
  msdp->state = MSD_READ_COMMAND_BLOCK;

  /* wait for ISR */
  return true;
}

/**
 * @brief Reads a newly received command block
 */
bool msd_read_command_block(USBMassStorageDriver *msdp) {

  msd_cbw_t *cbw = &(msdp->cbw);

  /* by default transition back to the idle state */
  msdp->state = MSD_IDLE;

  /* check the command */
  if ((cbw->signature != MSD_CBW_SIGNATURE) ||
      (cbw->lun > 0) ||
      ((cbw->data_len > 0) && (cbw->flags & 0x1F)) ||
      (cbw->scsi_cmd_len == 0) ||
      (cbw->scsi_cmd_len > 16)) {

    /* stall both IN and OUT endpoints */
    chSysLock();
    usbStallReceiveI(msdp->config->usbp, msdp->config->bulk_ep);
    usbStallTransmitI(msdp->config->usbp, msdp->config->bulk_ep);
    chSysUnlock();

    /* don't wait for ISR */
    return false;
  }

  bool sleep = false;

  /* check the command */
  switch (cbw->scsi_cmd_data[0]) {
    case SCSI_CMD_INQUIRY:
      sleep = msd_scsi_process_inquiry(msdp);
      break;
    case SCSI_CMD_REQUEST_SENSE:
      sleep = msd_scsi_process_request_sense(msdp);
      break;
    case SCSI_CMD_READ_CAPACITY_10:
      sleep = msd_scsi_process_read_capacity_10(msdp);
      break;
    case SCSI_CMD_READ_10:
    case SCSI_CMD_WRITE_10:
      if (msdp->config->rw_activity_callback)
        msdp->config->rw_activity_callback(TRUE);
      sleep = msd_scsi_process_start_read_write_10(msdp);
      if (msdp->config->rw_activity_callback)
        msdp->config->rw_activity_callback(FALSE);
      break;
    case SCSI_CMD_SEND_DIAGNOSTIC:
      sleep = msd_scsi_process_send_diagnostic(msdp);
      break;
    case SCSI_CMD_MODE_SENSE_6:
      sleep = msd_scsi_process_mode_sense_6(msdp);
      break;
    case SCSI_CMD_START_STOP_UNIT:
      sleep = msd_scsi_process_start_stop_unit(msdp);
      break;
    case SCSI_CMD_READ_FORMAT_CAPACITIES:
      sleep = msd_scsi_process_read_format_capacities(msdp);
      break;
    case SCSI_CMD_TEST_UNIT_READY:
      sleep = msd_scsi_process_test_unit_ready(msdp);
      break;
    case SCSI_CMD_FORMAT_UNIT:
      /* don't handle */
      msdp->result = true;
      break;
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
      /* don't handle */
      msdp->result = true;
      break;
    case SCSI_CMD_VERIFY_10:
      /* don't handle */
      msdp->result = true;
      break;
    default:
      msd_scsi_set_sense(msdp,
          SCSI_SENSE_KEY_ILLEGAL_REQUEST,
          SCSI_ASENSE_INVALID_COMMAND,
          SCSI_ASENSEQ_NO_QUALIFIER);

      /* stall IN endpoint */
      chSysLock();
      usbStallTransmitI(msdp->config->usbp, msdp->config->bulk_ep);
      chSysUnlock();

      return false;
  }

  if (msdp->result) {
    /* update sense with success status */
    msd_scsi_set_sense(msdp,
        SCSI_SENSE_KEY_GOOD,
        SCSI_ASENSE_NO_ADDITIONAL_INFORMATION,
        SCSI_ASENSEQ_NO_QUALIFIER);

    /* reset data length left */
    cbw->data_len = 0;
  }

  /* wait for ISR if needed */
  if (sleep)
    msd_wait_for_isr(msdp);

  msd_csw_t *csw = &(msdp->csw);

  if (!msdp->result && cbw->data_len) {
    /* still bytes left to send, this is too early to send CSW? */
    chSysLock();
    usbStallReceiveI(msdp->config->usbp, msdp->config->bulk_ep);
    usbStallTransmitI(msdp->config->usbp, msdp->config->bulk_ep);
    chSysUnlock();

    /*return false;*/
  }

  /* update the command status wrapper and send it to the host */
  csw->status = (msdp->result) ? MSD_COMMAND_PASSED : MSD_COMMAND_FAILED;
  csw->signature = MSD_CSW_SIGNATURE;
  csw->data_residue = cbw->data_len;
  csw->tag = cbw->tag;

  msd_start_transmit(msdp, (const uint8_t *)csw, sizeof(*csw));

  /* wait for ISR */
  return true;
}

/**
 * @brief Mass storage thread that processes commands
 */
static WORKING_AREA(mass_storage_thread_wa, 1536);
static msg_t mass_storage_thread(void *arg) {

  USBMassStorageDriver *msdp = (USBMassStorageDriver *)arg;

  chRegSetThreadName("USB-MSD");

  bool wait_for_isr = false;

  /* wait for the usb to be initialised */
  msd_wait_for_isr(msdp);

  while (!chThdShouldTerminate()) {
    wait_for_isr = false;

    /* wait on data depending on the current state */
    switch (msdp->state) {
      case MSD_IDLE:
        wait_for_isr = msd_wait_for_command_block(msdp);
        break;
      case MSD_READ_COMMAND_BLOCK:
        wait_for_isr = msd_read_command_block(msdp);
        break;
      case MSD_EJECTED:
        /* disconnect usb device */
        usbDisconnectBus(msdp->config->usbp);
        usbStop(msdp->config->usbp);
        chThdExit(0);
        return 0;
    }

    /* wait until the ISR wakes thread */
    if (wait_for_isr)
      msd_wait_for_isr(msdp);
  }

  return 0;
}

/**
 * @brief Initializse a USB mass storage driver
 */
void msdInit(USBMassStorageDriver *msdp) {

  chDbgCheck(msdp != NULL, "msdInit");

  msdp->config = NULL;
  msdp->thread = NULL;
  msdp->state = MSD_IDLE;

  /* initialize the driver events */
  chEvtInit(&msdp->evt_connected);
  chEvtInit(&msdp->evt_ejected);

  /* initialise the binary semaphore as taken */
  chBSemInit(&msdp->bsem, TRUE);

  /* initialise the sense data structure */
  size_t i;
  for (i = 0; i < sizeof(msdp->sense.byte); i++)
    msdp->sense.byte[i] = 0x00;
  msdp->sense.byte[0] = 0x70; /* response code */
  msdp->sense.byte[7] = 0x0A; /* additional sense length */

  /* initialize the inquiry data structure */
  msdp->inquiry.peripheral = 0x00;           /* direct access block device  */
  msdp->inquiry.removable = 0x80;            /* removable                   */
  msdp->inquiry.version = 0x04;              /* SPC-2                       */
  msdp->inquiry.response_data_format = 0x02; /* response data format        */
  msdp->inquiry.additional_length = 0x20;    /* response has 0x20 + 4 bytes */
  msdp->inquiry.sccstp = 0x00;
  msdp->inquiry.bqueetc = 0x00;
  msdp->inquiry.cmdque = 0x00;
}

/**
 * @brief Starts a USB mass storage driver
 */
void msdStart(USBMassStorageDriver *msdp, const USBMassStorageConfig *config) {

  chDbgCheck(msdp != NULL, "msdStart");
  chDbgCheck(config != NULL, "msdStart");
  chDbgCheck(msdp->thread == NULL, "msdStart");

  /* save the configuration */
  msdp->config = config;

  /* copy the config strings to the inquiry response structure */
  size_t i;
  for (i = 0; i < sizeof(msdp->config->short_vendor_id); ++i)
    msdp->inquiry.vendor_id[i] = config->short_vendor_id[i];
  for (i = 0; i < sizeof(msdp->config->short_product_id); ++i)
    msdp->inquiry.product_id[i] = config->short_product_id[i];
  for (i = 0; i < sizeof(msdp->config->short_product_version); ++i)
    msdp->inquiry.product_rev[i] = config->short_product_version[i];

  /* set the initial state */
  msdp->state = MSD_IDLE;

  /* make sure block device is working */
  while (blkGetDriverState(config->bbdp) != BLK_READY) {
    chThdSleepMilliseconds(50);
  }

  /* get block device information */
  blkGetInfo(config->bbdp, &msdp->block_dev_info);

  /* store the pointer to the mass storage driver into the user param
     of the USB driver, so that we can find it back in callbacks */
  config->usbp->in_params[config->bulk_ep] = (void *)msdp;
  config->usbp->out_params[config->bulk_ep] = (void *)msdp;

  /* run the thread */
  msdp->thread = chThdCreateStatic(mass_storage_thread_wa, sizeof(mass_storage_thread_wa), NORMALPRIO, mass_storage_thread, msdp);
}

/**
 * @brief Stops a USB mass storage driver
 */
void msdStop(USBMassStorageDriver *msdp) {

  chDbgCheck(msdp->thread != NULL, "msdStop");

  /* notify the thread that it's over */
  chThdTerminate(msdp->thread);

  /* wake the thread up and wait until it ends */
  chBSemSignal(&msdp->bsem);
  chThdWait(msdp->thread);
  msdp->thread = NULL;

  /* release the user params in the USB driver */
  msdp->config->usbp->in_params[msdp->config->bulk_ep] = NULL;
  msdp->config->usbp->out_params[msdp->config->bulk_ep] = NULL;
}
