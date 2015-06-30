/*
 * Copyright (C) 2015 Bart Slinger <bartslinger@gmail.com>
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

/** @file peripherals/sdcard_spi.c
 *  @brief This is an interface for reading and writing blocks of data to and from an SD card directly over SPI.
 * The pinout of the SD card is given here \cite sdcard_pinout. Connect the card according to the following table:
 *
 * MCU  | SD card | Description         |
 * :--- | :--     | :------------------ |
 * CS   | CS      | Chip Select         |
 * MOSI | DI      | Master Out Slave In |
 * MISO | DO      | Master In Slave Out |
 * SCK  | SCLK    | Clock Signal        |
 *
 * \image html airborne/sd-card-pinout.png SD Card pinout
 * The following resource was used as implementation reference: http://elm-chan.org/docs/mmc/mmc_e.html
 * The initialization procedure is implemented according to the following diagram. Only the branches for SD ver.2 are currently included.
 * \image html airborne/sdinit.png
 * @todo CRC checksums are not implemented. Fake values of 0xFF are used and they are ignored by the card.
 */

#include "sdcard_spi.h"

/**
 * This is the definition of the SD card. Currently, only one SD card can be used.
 * Most of the functions are prepared for feature support of multiple SD cards, by taking a reference to the SDCard as parameter.
 * Only the sdcard_spi_callback() function, which is called after a spi transaction is finished, does not provide a reference to the SDCard.
 * Thereby sdcard_spi_callback() cannot know which SDCard is ready.
 * However, the spi_transaction is given as a parameter, which is part of the SDCard. So thereby the SDCard might be identified.
 * @todo Add support for multiple SD cards.
 */
struct SDCard sdcard1;

/** @name Private Functions
 *  @{
 */
void sdcard_spi_spicallback(struct spi_transaction *t);
void sdcard_spi_send_cmd(struct SDCard *sdcard, uint8_t cmd, uint32_t arg);
void sdcard_spi_send_app_cmd(struct SDCard *sdcard, uint8_t cmd, uint32_t arg);
void sdcard_spi_request_bytes(struct SDCard *sdcard, uint8_t len);
/** @}*/

/**
 * @brief Configure initial values for SDCard.
 * Initialization of the card itself is done later in sdcard_spi_periodic().
 * @param sdcard Pointer to the SDCard.
 * @param spi_p Pointer to the SPI Peripheral.
 * @param slave_idx SPI Slave index.
 */
void sdcard_spi_init(struct SDCard *sdcard, struct spi_periph *spi_p, const uint8_t slave_idx)
{
  sdcard->spi_p = spi_p;
  sdcard->spi_t.slave_idx = slave_idx;
  sdcard->spi_t.select = SPISelectUnselect;
  sdcard->spi_t.status = SPITransDone;
  sdcard->spi_t.cpol = SPICpolIdleLow;
  sdcard->spi_t.cpha = SPICphaEdge1;
  sdcard->spi_t.dss = SPIDss8bit;
  sdcard->spi_t.bitorder = SPIMSBFirst;
  sdcard->spi_t.cdiv = SPIDiv64;
  sdcard->spi_t.input_buf = sdcard->input_buf;
  sdcard->spi_t.output_buf = sdcard->output_buf;
  sdcard->spi_t.input_length = 0;
  sdcard->spi_t.output_length = 0;

  sdcard->status = SDCard_BeforeDummyClock;
  sdcard->card_type = SDCardType_Unknown;
}

/**
 * @brief Periodic function of the SDCard.
 * @param sdcard Pointer to the SDCard.
 */
void sdcard_spi_periodic(struct SDCard *sdcard)
{

  /* Do nothing if spi transaction is in progress */
  if (sdcard->spi_t.status == SPITransPending || sdcard->spi_t.status == SPITransRunning) {
    return;
  }

  switch (sdcard->status) {

      /* Send dummy clock to set SD card in SPI mode */
    case SDCard_BeforeDummyClock:
      sdcard->spi_t.select = SPINoSelect;
      sdcard->spi_t.output_length = 10;
      sdcard->spi_t.input_length = 0;
      for (uint8_t i = 0; i < 10; i++) {
        sdcard->output_buf[i] = 0xFF;
      }
      sdcard->spi_t.after_cb = &sdcard_spi_spicallback;
      spi_submit(sdcard->spi_p, &sdcard->spi_t);
      sdcard->status = SDCard_SendingDummyClock;
      break;

      /* Sending ACMD41 */
    case SDCard_SendingACMD41v2:
      sdcard_spi_send_app_cmd(sdcard, 41, 0x40000000);
      sdcard->timeout_counter++;
      break;

      /* While busy, keep checking if it is still busy */
    case SDCard_Busy:
      sdcard_spi_request_bytes(sdcard, 1);
      break;

      /* While waiting for data token, keep polling */
    case SDCard_WaitingForDataToken:
      sdcard_spi_request_bytes(sdcard, 1);
      sdcard->timeout_counter++;
      break;

      /* While multiwrite busy, keep checking if it is still busy */
    case SDCard_MultiWriteBusy:
      sdcard_spi_request_bytes(sdcard, 1);
      break;

    default:
      break;
  }

}


/**
 * @brief Callback function for SPI transactions.
 * This function is called when the SPI transaction is finished and this function was defined as callback.
 * @param t Pointer to the spi_transaction that just finished.
 */
void sdcard_spi_spicallback(struct spi_transaction *t)
{
  (void) t; // ignore unused warning

  switch (sdcard1.status) {

      /* Ready with dummy clock, proceed with CMD0 */
    case SDCard_SendingDummyClock:
      sdcard1.spi_t.select = SPISelectUnselect;
      sdcard_spi_send_cmd(&sdcard1, 0, 0x00000000);
      sdcard1.status = SDCard_SendingCMD0;
      break;

      /* Ready sending CMD0, start polling result */
    case SDCard_SendingCMD0:
      sdcard1.response_counter = 0;
      sdcard_spi_request_bytes(&sdcard1, 1);
      sdcard1.status = SDCard_ReadingCMD0Resp;
      break;

      /* Continue polling bytes until there is a response or not */
    case SDCard_ReadingCMD0Resp:
      if (t->input_buf[0] == 0x01) {
        sdcard_spi_send_cmd(&sdcard1, 8, 0x000001AA);
        sdcard1.status = SDCard_SendingCMD8;
      } else if (sdcard1.response_counter >= 9) {
        sdcard1.status = SDCard_Error;
      } else {
        sdcard_spi_request_bytes(&sdcard1, 1);
      }
      break;

      /* Ready sending CMD8, start polling result */
    case SDCard_SendingCMD8:
      sdcard1.response_counter = 0;
      sdcard_spi_request_bytes(&sdcard1, 1);
      sdcard1.status = SDCard_ReadingCMD8Resp;
      break;

      /* Continue polling bytes until there is a response or not */
    case SDCard_ReadingCMD8Resp:
      if (t->input_buf[0] == 0x01) {
        sdcard_spi_request_bytes(&sdcard1, 4);
        sdcard1.status = SDCard_ReadingCMD8Parameter;
      } else if (sdcard1.response_counter >= 9) {
        sdcard1.status = SDCard_Error;
      } else {
        sdcard_spi_request_bytes(&sdcard1, 1);
      }
      break;

      /* Process parameter from CMD8 response (after 0x01 was received) */
    case SDCard_ReadingCMD8Parameter:
      if (sdcard1.input_buf[0] == 0x00 && sdcard1.input_buf[1] == 0x00 && sdcard1.input_buf[2] == 0x01
          && sdcard1.input_buf[3] == 0xAA) {
        sdcard1.status = SDCard_SendingACMD41v2;
        sdcard1.timeout_counter = 0;
      } else {
        sdcard1.status = SDCard_Error;
      }
      break;

      /* Ready sending the ACMDv2 command, start polling bytes for response */
    case SDCard_SendingACMD41v2:
      sdcard1.response_counter = 0;
      sdcard_spi_request_bytes(&sdcard1, 1);
      sdcard1.status = SDCard_ReadingACMD41v2Resp;
      break;

      /* Grabbing bytes in response to the ACMD41v2 command */
    case SDCard_ReadingACMD41v2Resp:
      if (t->input_buf[0] == 0x01) {
        if (sdcard1.timeout_counter < 500 - 1) { /* Wait 500 cycles until timeout */
          sdcard1.status = SDCard_SendingACMD41v2;
        } else {
          sdcard1.status = SDCard_Error;
        }
      } else if (t->input_buf[0] == 0x00) {
        sdcard_spi_send_cmd(&sdcard1, 58, 0x00000000);
        sdcard1.status = SDCard_SendingCMD58;
      } else if (sdcard1.response_counter >= 9) {
        sdcard1.status = SDCard_Error;
      } else {
        sdcard_spi_request_bytes(&sdcard1, 1);
      }
      break;

      /* Ready sending CMD58, request the first byte of R3 response */
    case SDCard_SendingCMD58:
      sdcard1.response_counter = 0;
      sdcard_spi_request_bytes(&sdcard1, 1);
      sdcard1.status = SDCard_ReadingCMD58Resp;
      break;

      /* Continue polling bytes until there is a response to CMD58 */
    case SDCard_ReadingCMD58Resp:
      if (sdcard1.response_counter >= 9) {
        sdcard1.status = SDCard_Error;
      } else if (sdcard1.input_buf[0] == 0x00) {
        sdcard_spi_request_bytes(&sdcard1, 4);
        sdcard1.status = SDCard_ReadingCMD58Parameter;
      } else {
        sdcard_spi_request_bytes(&sdcard1, 1);
      }
      break;

      /* Parameter of CMD58 ready, processing it */
    case SDCard_ReadingCMD58Parameter:
      if (sdcard1.input_buf[0] & 0x80) { // bit 31 set, CCS bit is valid
        if (sdcard1.input_buf[0] & 0x40) { // bit 30 set
          sdcard1.card_type = SDCardType_SdV2block;
          sdcard1.status = SDCard_Idle;
        } else { // bit 30 not set
          sdcard1.card_type = SDCardType_SdV1;
          sdcard_spi_send_cmd(&sdcard1, 16, SD_BLOCK_SIZE);
          sdcard1.status = SDCard_SendingCMD16;
        }
      } else { // bit 31 not set, CCS bit is unvalid
        sdcard1.status = SDCard_Error;
      }
      break;

      /* Ready sending CMD16, request first byte */
    case SDCard_SendingCMD16:
      sdcard1.response_counter = 0;
      sdcard_spi_request_bytes(&sdcard1, 1);
      sdcard1.status = SDCard_ReadingCMD16Resp;
      break;

      /* Continue polling bytes until there is a response to CMD16 */
    case SDCard_ReadingCMD16Resp:
      if (sdcard1.input_buf[0] == 0x00) {
        sdcard1.status = SDCard_Idle;
      } else if (sdcard1.response_counter >= 9) {
        sdcard1.status = SDCard_Error;
      } else {
        sdcard_spi_request_bytes(&sdcard1, 1);
      }
      break;

      /* Ready sending CMD24, request first response byte */
    case SDCard_SendingCMD24:
      sdcard1.response_counter = 0;
      sdcard_spi_request_bytes(&sdcard1, 1);
      sdcard1.status = SDCard_ReadingCMD24Resp;
      break;

      /* Request additional bytes until response or timeout */
    case SDCard_ReadingCMD24Resp:
      if (sdcard1.input_buf[0] == 0x00) {
        sdcard_spi_request_bytes(&sdcard1, 1);
        sdcard1.status = SDCard_BeforeSendingDataBlock;
      } else if (sdcard1.response_counter >= 9) {
        sdcard1.status = SDCard_Error;
      } else {
        sdcard_spi_request_bytes(&sdcard1, 1);
      }
      break;

      /* Send the data block */
    case SDCard_BeforeSendingDataBlock:
      sdcard1.spi_t.input_length = SD_BLOCK_SIZE + 4;
      sdcard1.spi_t.output_length = SD_BLOCK_SIZE + 4;
      sdcard1.spi_t.output_buf = &sdcard1.output_buf[5];
      sdcard1.spi_t.output_buf[0] = 0xFE; // data token
      sdcard1.spi_t.output_buf[SD_BLOCK_SIZE + 1] = 0xFF; // Fake CRC byte 1
      sdcard1.spi_t.output_buf[SD_BLOCK_SIZE + 2] = 0xFF; // Fake CRC byte 2
      sdcard1.spi_t.output_buf[SD_BLOCK_SIZE + 3] = 0xFF; // to request data response
      sdcard1.spi_t.after_cb = &sdcard_spi_spicallback;
      if (spi_submit(sdcard1.spi_p, &sdcard1.spi_t)) {
        sdcard1.status = SDCard_SendingDataBlock;
      } else {
        sdcard1.status = SDCard_Error;
      }
      break;

      /* Finished sending the data block */
    case SDCard_SendingDataBlock:
      if ((sdcard1.input_buf[SD_BLOCK_SIZE + 3] & 0x0F) == 0x05) {
        sdcard1.status = SDCard_Busy;
      } else {
        sdcard1.status = SDCard_Error;
      }
      sdcard1.spi_t.output_buf = sdcard1.output_buf;
      break;

    case SDCard_Busy:
      if (sdcard1.input_buf[0] != 0x00) {
        sdcard1.status = SDCard_Idle;
      }
      break;

      /* Ready sending CMD17, request first response byte */
    case SDCard_SendingCMD17:
      sdcard1.response_counter = 0;
      sdcard_spi_request_bytes(&sdcard1, 1);
      sdcard1.status = SDCard_ReadingCMD17Resp;
      break;

      /* Read response to CMD17 until response or timeout/error */
    case SDCard_ReadingCMD17Resp:
      if (sdcard1.input_buf[0] == 0x00) {
        sdcard_spi_request_bytes(&sdcard1, 1);
        sdcard1.timeout_counter = 0;
        sdcard1.status = SDCard_WaitingForDataToken;
      } else if (sdcard1.response_counter >= 9) {
        sdcard1.status = SDCard_Error;
      } else {
        sdcard_spi_request_bytes(&sdcard1, 1);
      }
      break;

      /* Processing byte to see if it is a data token */
    case SDCard_WaitingForDataToken:
      if (sdcard1.input_buf[0] == 0xFE) { // Data token received
        sdcard1.status = SDCard_Idle;
        sdcard1.spi_t.input_length = SD_BLOCK_SIZE + 2; // 2 Fake CRC bytes
        sdcard1.spi_t.output_length = SD_BLOCK_SIZE + 2;
        sdcard1.spi_t.cdiv = SPIDiv8;
        for (uint16_t i = 0; i < (SD_BLOCK_SIZE + 2); i++) {
          sdcard1.output_buf[i] = 0xFF;
        }
        sdcard1.spi_t.after_cb = &sdcard_spi_spicallback;
        spi_submit(sdcard1.spi_p, &sdcard1.spi_t);
        sdcard1.status = SDCard_ReadingDataBlock;
      } else if (sdcard1.timeout_counter > 498) {
        sdcard1.status = SDCard_Error;
      }
      break;

      /* Data block received in buffer, process data */
    case SDCard_ReadingDataBlock:
      sdcard1.status = SDCard_Idle;
      if (sdcard1.read_callback != NULL) {
        sdcard1.read_callback();
      }
      break;

      /* Ready sending CMD25, request first response byte */
    case SDCard_SendingCMD25:
      sdcard1.response_counter = 0;
      sdcard_spi_request_bytes(&sdcard1, 1);
      sdcard1.status = SDCard_ReadingCMD25Resp;
      break;

      /* Request additional bytes until response or timeout */
    case SDCard_ReadingCMD25Resp:
      if (sdcard1.input_buf[0] == 0x00) {
        sdcard_spi_request_bytes(&sdcard1, 1);
        sdcard1.status = SDCard_MultiWriteIdle;
      } else if (sdcard1.response_counter >= 9) {
        sdcard1.status = SDCard_Error;
      } else {
        sdcard_spi_request_bytes(&sdcard1, 1);
      }
      break;

      /* Check the response after a block was written */
    case SDCard_MultiWriteWriting:
      if ((sdcard1.input_buf[SD_BLOCK_SIZE + 3] & 0x0F) == 0x05 /* Data accepted */) {
        sdcard1.status = SDCard_MultiWriteBusy;
      } else {
        sdcard1.status = SDCard_Error;
      }
      break;

      /* Check if sd card is still busy after a packet during multiwrite */
    case SDCard_MultiWriteBusy:
      if (sdcard1.input_buf[0] != 0x00) {
        sdcard1.status = SDCard_MultiWriteIdle;
      }
      break;

      /* Finished the stop command transaction, waiting for internal processes of the SD card */
    case SDCard_MultiWriteStopping:
      sdcard1.status = SDCard_Busy;
      break;

      /* Should not reach this */
    default:
      break;
  }
}

/**
 * @brief Send a command to the SDCard.
 * The response is parsed by sdcard_spi_spicallback().
 * Possible commands and arguments are specified by the SD Association.
 * The following website specifies some common commands: http://elm-chan.org/docs/mmc/mmc_e.html
 * @param sdcard Pointer to the SDCard.
 * @param cmd Command index, can be a value from 0 to 63.
 * @param arg Argument to be passed together with the command.
 */
void sdcard_spi_send_cmd(struct SDCard *sdcard, uint8_t cmd, uint32_t arg)
{
  (void) cmd; (void) arg;
  sdcard->spi_t.input_length = 6;
  sdcard->spi_t.output_length = 6;
  sdcard->output_buf[0] = 0x40 | cmd;
  sdcard->output_buf[1] = arg >> 24;
  sdcard->output_buf[2] = arg >> 16;
  sdcard->output_buf[3] = arg >> 8;
  sdcard->output_buf[4] = arg;
  switch (cmd) {
    case 0:
      sdcard->output_buf[5] = 0x95; /* CRC byte for CMD0 */
      break;
    case 8:
      sdcard->output_buf[5] = 0x87; /* CRC byte for CMD8 */
      break;
    default:
      sdcard->output_buf[5] = 0x01; /* Fake CRC byte, will be ignored by the SDCard */
  }
  sdcard->spi_t.after_cb = &sdcard_spi_spicallback;

  spi_submit(sdcard->spi_p, &sdcard->spi_t);
}

/**
 * @brief Send a app-command to the SDCard.
 * The response is parsed by sdcard_spi_spicallback().
 * This is the same as a normal command, but preceeded by a CMD55.
 * Possible commands and arguments are specified by the SD Association.
 * The following website specifies some common commands: http://elm-chan.org/docs/mmc/mmc_e.html
 * @param sdcard Pointer to the SDCard.
 * @param cmd Command index, possible values in range from 0 to 63.
 * @param arg Argument to be passed together with the command.
 */
void sdcard_spi_send_app_cmd(struct SDCard *sdcard, uint8_t cmd, uint32_t arg)
{
  sdcard->spi_t.output_length = 21;
  sdcard->spi_t.input_length = 21;

  sdcard->output_buf[0] = 0x77; /* Begin CMD55 */
  sdcard->output_buf[1] = 0x00;
  sdcard->output_buf[2] = 0x00;
  sdcard->output_buf[3] = 0x00;
  sdcard->output_buf[4] = 0x00;
  sdcard->output_buf[5] = 0x01; /* End CMD55 */
  sdcard->output_buf[6] = 0xFF; /* Somewhere during the following bytes, response to CMD55 is received. */
  sdcard->output_buf[7] = 0xFF;
  sdcard->output_buf[8] = 0xFF;
  sdcard->output_buf[9] = 0xFF;
  sdcard->output_buf[10] = 0xFF;
  sdcard->output_buf[11] = 0xFF;
  sdcard->output_buf[12] = 0xFF;
  sdcard->output_buf[13] = 0xFF;
  sdcard->output_buf[14] = 0xFF;

  sdcard->output_buf[15] = 0x40 + cmd; /* Begin of the ACMD */
  sdcard->output_buf[16] = arg >> 24;
  sdcard->output_buf[17] = arg >> 16;
  sdcard->output_buf[18] = arg >> 8;
  sdcard->output_buf[19] = arg;
  sdcard->output_buf[20] = 0x01;  /* End of the ACMD */

  /* Set function to be called after transfer is finished */
  sdcard->spi_t.after_cb = &sdcard_spi_spicallback;

  /* Submit the transaction */
  spi_submit(sdcard->spi_p, &sdcard->spi_t);
}

/**
 * @brief Request one or more bytes from the SDCard.
 * This function request one or more bytes and parses the output in sdcard_spi_spicallback().
 * @param sdcard Pointer to the SDCard.
 * @param len Number of bytes to request.
 */
void sdcard_spi_request_bytes(struct SDCard *sdcard, uint8_t len)
{
  sdcard->spi_t.input_length = len;
  sdcard->spi_t.output_length = len;
  for (uint8_t i = 0; i < len; i++) {
    sdcard->output_buf[i] = 0xFF;
  }
  sdcard->spi_t.after_cb = &sdcard_spi_spicallback;
  spi_submit(sdcard->spi_p, &sdcard->spi_t);
  sdcard->response_counter++;
}

/**
 * @brief Write a single block (512 bytes) to the SDCard at a given address.
 * @param sdcard Pointer to the SDCard.
 * @param addr Block address to write to. The addresses are consecutive numbers, regardless the SDCardType.
 */
void sdcard_spi_write_block(struct SDCard *sdcard, uint32_t addr)
{
  /* Do not write data if not in idle state */
  if (sdcard->status != SDCard_Idle) {
    return;
  }

  /* To high transfer speed might cause problems. */
  sdcard->spi_t.cdiv = SPIDiv64;

  /* Translate block address to byte address */
  if (sdcard->card_type != SDCardType_SdV2block) {
    addr = addr * SD_BLOCK_SIZE;
  }

  /* Send command 24 (write block) to the SDCard */
  sdcard_spi_send_cmd(sdcard, 24, addr);
  sdcard->status = SDCard_SendingCMD24;
}

/**
 * @brief Read a single block (512 bytes) from the SDCard at a given address.
 * When the block reading is finished, sdcard_spi_spicallback() is triggered which in turn calls the function from the callback parameter.
 * @param sdcard Pointer to the SDCard.
 * @param addr Block address to read from. The addresses are consecutive numbers, regardless the SDCardType.
 * @param callback Function to call when block reading is finished.
 */
void sdcard_spi_read_block(struct SDCard *sdcard, uint32_t addr, SDCardCallback callback)
{
  /* Do not read data if not in idle state */
  if (sdcard->status != SDCard_Idle) {
    return;
  }

  /* To high transfer speed might cause problems. */
  sdcard->spi_t.cdiv = SPIDiv32;

  /* Translate block address to byte address */
  if (sdcard->card_type != SDCardType_SdV2block) {
    addr = addr * SD_BLOCK_SIZE;
  }

  /* Set function to be called after the read action has finished. */
  sdcard->read_callback = callback;

  /* Send command 17 (read block) to the SDCard */
  sdcard_spi_send_cmd(sdcard, 17, addr);
  sdcard->status = SDCard_SendingCMD17;
}

/**
 * @brief Start writing multiple blocks of 512 bytes to the SDCard.
 * This function notifies the SDCard to expect one or more data packets (which are sent by sdcard_spi_multiwrite_next()).
 * @param sdcard Pointer to the SDCard.
 * @param addr Block address to start writing at. Addresses are consecutive numbers, regardless the SDCardType.
 */
void sdcard_spi_multiwrite_start(struct SDCard *sdcard, uint32_t addr)
{
  /* Do not start multiwrite if not in idle state */
  if (sdcard->status != SDCard_Idle) {
    return;
  }

  /* Translate block address to byte address */
  if (sdcard->card_type != SDCardType_SdV2block) {
    addr = addr * SD_BLOCK_SIZE;
  }

  /* Send command 25 (start multiwrite) to the SDCard */
  sdcard_spi_send_cmd(sdcard, 25, addr);
  sdcard->status = SDCard_SendingCMD25;
}

/**
 * @brief Write a(nother) data block (512 bytes) to the SDCard.
 * Use only after sdcard_spi_multiwrite_start().
 * @param sdcard Pointer to the SDCard.
 */
void sdcard_spi_multiwrite_next(struct SDCard *sdcard)
{
  /* Can only write next block if card is in multiwrite mode and not currently busy */
  if (sdcard->status != SDCard_MultiWriteIdle) {
    return;
  }
  sdcard->spi_t.input_length = 516;
  sdcard->spi_t.output_length = 516;
  sdcard->spi_t.cdiv = SPIDiv32;        /* Too high write speed can cause problems */
  sdcard->spi_t.output_buf[0] = 0xFC;   /* Data token specific for multiwrite */
  sdcard->spi_t.output_buf[513] = 0xFF; /* Fake Fake CRC byte 1 */
  sdcard->spi_t.output_buf[514] = 0xFF; /* Fake CRC byte 2 */
  sdcard->spi_t.output_buf[515] = 0xFF; /* Polling for busy flag */

  /* Set the callback */
  sdcard->spi_t.after_cb = &sdcard_spi_spicallback;

  /* Submit the spi transaction */
  spi_submit(sdcard->spi_p, &sdcard->spi_t);

  sdcard->status = SDCard_MultiWriteWriting;
}

/**
 * @brief Stop with multiwrite procedure.
 * @param sdcard Pointer to the SDCard.
 */
void sdcard_spi_multiwrite_stop(struct SDCard *sdcard)
{
  /* Can only stop if card is in multiwrite mode and not currently busy */
  if (sdcard->status != SDCard_MultiWriteIdle) {
    return;
  }
  sdcard->spi_t.input_length = 2;
  sdcard->spi_t.output_length = 2;
  sdcard->spi_t.cdiv = SPIDiv32;                  /* Too high write speed can cause problems */
  sdcard->output_buf[0] = 0xFD;                   /* Stop token specific for multiwrite */
  sdcard->output_buf[1] = 0xFF;                   /* One dummy byte before polling if card is busy */

  /* Set the callback */
  sdcard->spi_t.after_cb = &sdcard_spi_spicallback;

  /* Submit the spi transaction */
  spi_submit(sdcard->spi_p, &sdcard->spi_t);
  sdcard->status = SDCard_MultiWriteStopping;
}

