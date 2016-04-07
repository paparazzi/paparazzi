/*
 * Copyright (C) Kevin van Hecke
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/px4_flash/px4_flash.c"
 * @author Kevin van Hecke
 * Enables to flashes the px4 FBW and AP through the original px4 bootloader.
 * Assumes the flash port on the Pixhawk is configured as the usb.
 */

#include "modules/px4_flash/px4_flash.h"
//#include "subsystems/datalink/downlink.h"
#include "modules/px4_flash/protocol.h"
#include "mcu_periph/sys_time_arch.h"
#include "subsystems/intermcu/intermcu_ap.h"

// Serial Port
#include "mcu_periph/uart.h"
#include "mcu_periph/usb_serial.h"

#include "led.h"

#include "libopencm3/cm3/scb.h"

#include "mcu_periph/sys_time.h"
tid_t px4iobl_tid; ///< id for time out of the px4 bootloader reset

// define coms link for px4io f1
#define PX4IO_PORT   (&((PX4IO_UART).device))
#define FLASH_PORT   (&((FLASH_UART).device))

// weird that these below are not in protocol.h, which is from the firmware px4 repo
// below is copied from qgroundcontrol:
#define PROTO_INSYNC            0x12 ///< 'in sync' byte sent before status
#define PROTO_EOC               0x20 ///< end of command
// Reply bytes
#define PROTO_OK                0x10 ///< INSYNC/OK      - 'ok' response
#define PROTO_FAILED            0x11 ///< INSYNC/FAILED  - 'fail' response
#define PROTO_INVALID           0x13 ///< INSYNC/INVALID - 'invalid' response for bad commands
// Command bytes
#define PROTO_GET_SYNC          0x21 ///< NOP for re-establishing sync
#define PROTO_GET_DEVICE        0x22 ///< get device ID bytes
#define PROTO_CHIP_ERASE        0x23 ///< erase program area and reset program address
#define PROTO_LOAD_ADDRESS      0x24 ///< set next programming address
#define PROTO_PROG_MULTI        0x27 ///< write bytes at program address and increment
#define PROTO_GET_CRC           0x29 ///< compute & return a CRC
#define PROTO_BOOT              0x30 ///< boot the application

bool setToBootloaderMode;
bool px4ioRebootTimeout;

void px4flash_init(void)
{
  setToBootloaderMode = false;
  px4ioRebootTimeout = false;
  px4iobl_tid = sys_time_register_timer(15.0, NULL); //20 (fbw pprz bl timeout)-5 (px4 fmu bl timeout)
}

void px4flash_event(void)
{
  if (sys_time_check_and_ack_timer(px4iobl_tid)) {
    px4ioRebootTimeout = TRUE;
    sys_time_cancel_timer(px4iobl_tid);
    //for unknown reasons, 1500000 baud does not work reliably after prolonged times.
    //I suspect a temperature related issue, combined with the fbw f1 crystal which is out of specs
    //After a initial period on 1500000, revert to 230400
    //We still start at 1500000 to remain compatible with original PX4 firmware. (which always runs at 1500000)
    uart_periph_set_baudrate(PX4IO_PORT->periph, B230400);
  }
  if (PX4IO_PORT->char_available(PX4IO_PORT->periph)) {
    if (!setToBootloaderMode) {
      //ignore anything coming from IO if not in bootloader mode (which should be nothing)
    } else {
      //relay everything from IO to the laptop
      while (PX4IO_PORT->char_available(PX4IO_PORT->periph)) {
        unsigned char b = PX4IO_PORT->get_byte(PX4IO_PORT->periph);
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, b);
      }
    }
  }

  //TODO: check if bootloader timeout was surpassed
  if (FLASH_PORT->char_available(FLASH_PORT->periph) && !setToBootloaderMode) {
    // TMP TEST
    //    while (FLASH_PORT->char_available(FLASH_PORT->periph)) {
    //      unsigned char bla = FLASH_PORT->get_byte(FLASH_PORT->periph);
    //      FLASH_PORT->put_byte(FLASH_PORT->periph, 0,bla);
    //    }
    //    return;

    //check whether this is flash related communication, and for who (ap/fbw)
    int state = 0;
    while (state < 4 && FLASH_PORT->char_available(FLASH_PORT->periph)) {
      unsigned char b = FLASH_PORT->get_byte(FLASH_PORT->periph);
      switch (state) {
        case (0) :
          if (b == 'p') { state++; } else { return; }
          break;
        case (1) :
          if (b == 'p') { state++; } else { return; }
          break;
        case (2) :
          if (b == 'r') { state++; } else { return; }
          break;
        case (3) :
          if (b == 'z') { state++; } else { return; }
          break;
        default :
          break;
      }
    }

    if (state != 4) {return;}
    //TODO: check if/how this interferes with flashing original PX4 firmware
    unsigned char target = FLASH_PORT->get_byte(FLASH_PORT->periph);
    if (target == '1') { //target ap
      //the target is the ap, so reboot to PX4 bootloader
      scb_reset_system();

    } else { // target fbw
      //the target is the fbw, so reboot the fbw and switch to relay mode

      //first check if the bootloader has not timeout:
      if (px4ioRebootTimeout) {
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'T');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'I');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'M');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'E');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'O');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'U');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'T'); // use 7 chars as answer
        return;
      }  { // FBW OK OK hollay hollay :)
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'F');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'B');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'W');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'O');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'K');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'O');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'K'); // use 7 chars as answer
      }


      //stop all intermcu communication:
      intermcu_set_enabled(FALSE);

      px4iobl_tid = sys_time_register_timer(5.0, NULL); //10 (fbw pprz bl timeout)-5 (px4 fmu bl timeout)
      /*
      * The forceprog define is very usefull, if for whatever reason the (normal, not bootloader) firmware on the IO chip became disfunct.
      * In that case:
      * 1. enable this define
      * 2. build and upload  the fmu f4 chip (ap target in pprz center)
      * 3. build the io code, and convert the firmware using the following command:
      * Optional 5&6:
      * 5a. Either, boot the Pixhawk (reconnect usb) holding the IO reset button until the FMU led stops blinking fast (i.e. exits its own bootloader)
      * 5b  Or, press the IO reset button on the pixhawk
      * 6. Watch the output of the command of step 4, it should recognize the IO bootloader and start flashing. If not try repeating step 5a.
      * 7. Don forget to disable the define and upload the ap again :)
      */
      //#define forceprog

#ifndef forceprog
      //send the reboot to bootloader command:
      static struct IOPacket  dma_packet;
      dma_packet.count_code = 0x40 + 0x01;
      dma_packet.crc = 0;
      dma_packet.page = PX4IO_PAGE_SETUP;
      dma_packet.offset = PX4IO_P_SETUP_REBOOT_BL;
      dma_packet.regs[0] = PX4IO_REBOOT_BL_MAGIC;
      dma_packet.crc = crc_packet(&dma_packet);
      struct IOPacket *pkt = &dma_packet;
      uint8_t *p = (uint8_t *)pkt;
      PX4IO_PORT->put_byte(PX4IO_PORT->periph, 0, p[0]);
      PX4IO_PORT->put_byte(PX4IO_PORT->periph, 0, p[1]);
      PX4IO_PORT->put_byte(PX4IO_PORT->periph, 0, p[2]);
      PX4IO_PORT->put_byte(PX4IO_PORT->periph, 0, p[3]);
      PX4IO_PORT->put_byte(PX4IO_PORT->periph, 0, p[4]);
      PX4IO_PORT->put_byte(PX4IO_PORT->periph, 0, p[5]);

      sys_time_usleep(5000); // this seems to be close to the minimum delay necessary to process this packet at the IO side
      //the pixhawk IO chip should respond with:
      // 0x00 ( PKT_CODE_SUCCESS )
      // 0xe5
      // 0x32
      // 0x0a
      //After that, the IO chips reboots into bootloader mode, in which it will stay for a short period
      //The baudrate in bootloader mode ic changed to 115200 (normal operating baud is 1500000, at least for original pixhawk fmu firmware)

      //state machine
      state = 0;
      while (state < 4 && PX4IO_PORT->char_available(PX4IO_PORT->periph)) {

        unsigned char b = PX4IO_PORT->get_byte(PX4IO_PORT->periph);
        switch (state) {
          case (0) :
            if (b == PKT_CODE_SUCCESS) { state++; } else { state = 0; }
            break;
          case (1) :
            if (b == 0xe5) { state++; } else { state = 0; }
            break;
          case (2) :
            if (b == 0x32) { state++; } else { state = 0; }
            break;
          case (3) :
            if (b == 0x0a) { state++; } else { state = 0; }
            break;
          default :
            break;
        }
      }
#else
      int state = 4;
#endif
      if (state == 4) {
        uart_periph_set_baudrate(PX4IO_PORT->periph, B115200);
        /* look for the bootloader for 150 ms */
        int ret = 0;
        for (int i = 0; i < 15 && !ret ; i++) {
          sys_time_usleep(10000);

          //send a get_sync command in order to keep the io in bootloader mode
          PX4IO_PORT->put_byte(PX4IO_PORT->periph, 0, PROTO_GET_SYNC);
          PX4IO_PORT->put_byte(PX4IO_PORT->periph, 0, PROTO_EOC);

          //get_sync should be replied with, so check if that happens and
          //all other bytes are discarded, hopefully those were not important
          //(they may be caused by sending multiple syncs)
          while (PX4IO_PORT->char_available(PX4IO_PORT->periph)) {
            unsigned char b = PX4IO_PORT->get_byte(PX4IO_PORT->periph);

            if (b == PROTO_INSYNC) {
              setToBootloaderMode = true;
              ret = 1;
              break;
            }
          }
        }
        if (setToBootloaderMode) {
          //if successfully entered bootloader mode, clear any remaining bytes (which may have a function, but I did not check)
          while (PX4IO_PORT->char_available(PX4IO_PORT->periph)) {PX4IO_PORT->get_byte(PX4IO_PORT->periph);}
        }
      } else {
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'E'); //TODO: find out what the PX4 protocol for error feedback is...
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'R');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'R');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'O');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, 'R');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, '!');
        FLASH_PORT->put_byte(FLASH_PORT->periph, 0, ' '); // use 7 chars as answer

      }
    }
  } else if (FLASH_PORT->char_available(FLASH_PORT->periph)) {
    //already in bootloader mode, just directly relay data
    while (FLASH_PORT->char_available(FLASH_PORT->periph)) {
      unsigned char b = FLASH_PORT->get_byte(FLASH_PORT->periph);
      PX4IO_PORT->put_byte(PX4IO_PORT->periph, 0, b);
    }
  }
}

