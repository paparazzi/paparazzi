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
 * @file "modules/px4io_flash/px4io_flash.c"
 * @author Kevin van Hecke
 * Flashes the px4io f1 through the px4 bootloader.
 * Assumes the telem2 port on the Pixhawk is connected to a ttyACM device (blackmagic probe)
 */

#include "modules/px4io_flash/px4io_flash.h"
//#include "subsystems/datalink/downlink.h"
#include "modules/px4io_flash/protocol.h"
#include "mcu_periph/sys_time_arch.h"
#include "subsystems/intermcu/intermcu_ap.h"

// Serial Port
#include "mcu_periph/uart.h"

// define coms link for px4io f1
#define PX4IO_PORT   (&((PX4IO_UART).device))
#define TELEM2_PORT   (&((TELEM2_UART).device))

// weird that these below are not in protocol.h, which is from the firmware px4 repo
// below is copied from qgroundcontrol:
#define PROTO_INSYNC            0x12   ///< 'in sync' byte sent before status
#define PROTO_EOC               0x20   ///< end of command
// Reply bytes
#define PROTO_OK                0x10   ///< INSYNC/OK      - 'ok' response
#define PROTO_FAILED            0x11   ///< INSYNC/FAILED  - 'fail' response
#define PROTO_INVALID           0x13 ///< INSYNC/INVALID - 'invalid' response for bad commands
// Command bytes
#define PROTO_GET_SYNC          0x21   ///< NOP for re-establishing sync
#define PROTO_GET_DEVICE        0x22   ///< get device ID bytes
#define PROTO_CHIP_ERASE        0x23   ///< erase program area and reset program address
#define PROTO_LOAD_ADDRESS      0x24 ///< set next programming address
#define PROTO_PROG_MULTI        0x27   ///< write bytes at program address and increment
#define PROTO_GET_CRC           0x29 ///< compute & return a CRC
#define PROTO_BOOT              0x30   ///< boot the application

bool_t setToBootloaderMode;

void px4ioflash_init(void)
{
  setToBootloaderMode = FALSE;
}

void px4ioflash_event(void)
{
  // setToBootloaderMode=true;
  if (PX4IO_PORT->char_available(PX4IO_PORT->periph)) {
    if (!setToBootloaderMode) {
      //ignore anything coming from IO if not in bootloader mode (which should be nothing)
    } else {
      //relay everything from IO to the laptop
      unsigned char b = PX4IO_PORT->get_byte(PX4IO_PORT->periph);
      TELEM2_PORT->put_byte(TELEM2_PORT->periph, b);
    }
  }

  //TODO: check if timeout was surpassed
  if (TELEM2_PORT->char_available(TELEM2_PORT->periph) && !setToBootloaderMode) {
    //data was received on the pc uart, so
    //stop all intermcu comminication:
    disable_inter_comm(true);
    //send the reboot to bootloader command:

    /*
      * The progdieshit define is very usefull, if for whatever reason the (normal, not bootloader) firmware on the IO chip became disfunct.
      * In that case:
      * 1. enable this define
      * 2. build and upload  the fmu f4 chip (ap target in pprz center)
      * 3. build the io code, and convert the firmware using the following command:
      *       /home/houjebek/paparazzi/sw/tools/pixhawk/px_mkfw.py --prototype "/home/houjebek/px4/Firmware/Images/px4io-v2.prototype" --image /home/houjebek/paparazzi/var/aircrafts/Iris/fbw/fbw.bin > /home/houjebek/paparazzi/var/aircrafts/Iris/fbw/fbw.px4
      * 4. Start the following command:
      *    /home/houjebek/paparazzi/sw/tools/pixhawk/px_uploader.py --port "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_FT906KBO-if00-port0" /home/houjebek/paparazzi/var/aircrafts/Iris/fbw/fbw.px4
      * 5a. Either, boot the Pixhawk (reconnect usb) holding the IO reset button until the FMU led stops blinking fast (i.e. exits its own bootloader)
      * 5b  Or, press the IO reset button on the pixhawk
      * 6. Watch the output of the command of step 4, it should recognize the IO bootloader and start flashing. If not try repeating step 5a.
      * 7. Don forget to disable the define and upload the ap again :)
      */
    //#define progdieshit

#ifndef progdieshit
    static struct IOPacket  dma_packet;
    dma_packet.count_code = 0x40 + 0x01;
    dma_packet.crc = 0;
    dma_packet.page = PX4IO_PAGE_SETUP;
    dma_packet.offset = PX4IO_P_SETUP_REBOOT_BL;
    dma_packet.regs[0] = PX4IO_REBOOT_BL_MAGIC;
    dma_packet.crc = crc_packet(&dma_packet);
    struct IOPacket *pkt = &dma_packet;
    uint8_t *p = (uint8_t *)pkt;
    PX4IO_PORT->put_byte(PX4IO_PORT->periph, p[0]);
    PX4IO_PORT->put_byte(PX4IO_PORT->periph, p[1]);
    PX4IO_PORT->put_byte(PX4IO_PORT->periph, p[2]);
    PX4IO_PORT->put_byte(PX4IO_PORT->periph, p[3]);
    PX4IO_PORT->put_byte(PX4IO_PORT->periph, p[4]);
    PX4IO_PORT->put_byte(PX4IO_PORT->periph, p[5]);

    // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'E');
    // for (int i=0;i<6;i++) {
    //  unsigned char tmp[3];
    //  itoa(p[i],tmp,16);
    //  TELEM2_PORT->put_byte(TELEM2_PORT->periph,tmp[0]);
    //  TELEM2_PORT->put_byte(TELEM2_PORT->periph,tmp[1]);
    //  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
    //  TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
    // }

    sys_time_usleep(5000); // this seems to be close to the minimum delay necessary to process this packet at the IO side
    //the pixhawk IO chip should respond with:
    // 0x00 ( PKT_CODE_SUCCESS )
    // 0xe5
    // 0x32
    // 0x0a
    //After that, the IO chips reboots into bootloader mode, in which it will stay for a short period
    //The baudrate in bootloader mode ic changed to 115200 (normal operating baud is 1500000, at least for original pixhawk fmu firmware)

    //state machine
    int state = 0;
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
          TELEM2_PORT->put_byte(TELEM2_PORT->periph, 'b');
          break;
      }
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'S');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,state+48);
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
    }
#else
    int state = 4;
#endif
    if (state == 4) {
#ifndef progdieshit
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'S');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'6');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
#endif
      uart_periph_set_baudrate(PX4IO_PORT->periph, B115200);
      /* look for the bootloader for 150 ms */
      int ret = 0;
      for (int i = 0; i < 15 && !ret ; i++) {
        sys_time_usleep(10000);


        //send a get_sync command in order to keep the io in bootloader mode
        PX4IO_PORT->put_byte(PX4IO_PORT->periph, PROTO_GET_SYNC);
        PX4IO_PORT->put_byte(PX4IO_PORT->periph, PROTO_EOC);


#ifndef progdieshit
        // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'S');
        //    TELEM2_PORT->put_byte(TELEM2_PORT->periph,'6');
        //    TELEM2_PORT->put_byte(TELEM2_PORT->periph,'a');
        //    TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
        //    TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
#endif

        //get_sync should be replied with, so check if that happens and
        //all other bytes are discarded, hopefully those were not important
        //(they may be caused by sending multiple syncs)
        while (PX4IO_PORT->char_available(PX4IO_PORT->periph)) {
          unsigned char b = PX4IO_PORT->get_byte(PX4IO_PORT->periph);

#ifndef progdieshit
          // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'S');
          //    TELEM2_PORT->put_byte(TELEM2_PORT->periph,'6');
          //    TELEM2_PORT->put_byte(TELEM2_PORT->periph,'b');
          //    TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
          //    TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
#endif

          if (b == PROTO_INSYNC) {
#ifndef progdieshit
            // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'S');
            // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'7');
            // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
            // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
#endif

            setToBootloaderMode = true;
            ret = 1;
            break;
          }
        }
      }
      if (setToBootloaderMode) {

#ifndef progdieshit
        // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'S');
        // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'8');
        // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
        // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
#endif

        //if successfully entered bootloader mode, clear any remaining bytes (which may have a function, but I did not check)
        while (PX4IO_PORT->char_available(PX4IO_PORT->periph)) {PX4IO_PORT->get_byte(PX4IO_PORT->periph);}
      }

#ifndef progdieshit
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'S');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'9');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
#endif

    } else {
      TELEM2_PORT->put_byte(TELEM2_PORT->periph, 'E');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'r');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'r');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'o');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'r');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\n');
      // TELEM2_PORT->put_byte(TELEM2_PORT->periph,'\r');
    }
  } else if (TELEM2_PORT->char_available(TELEM2_PORT->periph)) {
    //already in bootloader mode, just directly relay data
    unsigned char b = TELEM2_PORT->get_byte(TELEM2_PORT->periph);
    PX4IO_PORT->put_byte(PX4IO_PORT->periph, b);
    // TELEM2_PORT->put_byte(TELEM2_PORT->periph,b);
  }

}

