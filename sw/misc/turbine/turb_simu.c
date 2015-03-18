/*
 * Copyright (C) 2009 Martin Mueller
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

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>


/* *********************** change this to your needs *********************** */

/* serial speed  */
#define DEFAULT_BAUDRATE    57600
/* serial port (ttyS0, ttyACM0, ttyUSB0) */
#define SERIAL_DEVICE       "/dev/ttyUSB0"


struct speed_map
{
    speed_t speed;
    unsigned int speed_value;
};

static const struct speed_map speeds[] = {
    {B0, 0},
    {B50, 50},
    {B75, 75},
    {B110, 110},
    {B134, 134},
    {B150, 150},
    {B200, 200},
    {B300, 300},
    {B600, 600},
    {B1200, 1200},
    {B1800, 1800},
    {B2400, 2400},
    {B4800, 4800},
    {B9600, 9600},
    {B19200, 19200},
    {B38400, 38400},
    {B57600, 57600},
    {B115200, 115200},
    {B230400, 230400},
};

static const int NUM_SPEEDS = (sizeof(speeds) / sizeof(struct speed_map));


static speed_t int_to_baud(unsigned int value)
{
    int i;

    for (i = 0; i < NUM_SPEEDS; ++i)
    {
        if (value == speeds[i].speed_value)
        {
            return speeds[i].speed;
        }
    }
    return (speed_t) - 1;
}

int main (void)
{
    char out_file_name[80] = SERIAL_DEVICE;
    fd_set read_fd_set;
    int max_fd;
    struct termios orig_termios, cur_termios;
    struct timeval select_tv;
    unsigned char data[256];
    unsigned char data_temp;
    int serial_fd;
    int count, i;
    int br;
    int baud;

    if ((br = int_to_baud(DEFAULT_BAUDRATE)) < 0 )
    {
        printf("invalid baudrate %d\n", DEFAULT_BAUDRATE);
        return( -1 );
    }

    if ( (serial_fd = open( out_file_name, O_RDWR )) == 0 )
    {
        perror("could not open serial port");
        return( -1 );
    }

    if (tcgetattr(serial_fd, &orig_termios))
    {
        perror("getting modem serial device attr");
        return( -1 );
    }

    cur_termios = orig_termios;

    /* input modes */
    cur_termios.c_iflag &= ~(IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK|ISTRIP|INLCR|IGNCR
                           |ICRNL |IXON|IXANY|IXOFF|IMAXBEL);

    /* IGNCR does not pass 0x0D */
    cur_termios.c_iflag |= BRKINT;

    /* output_flags */
    cur_termios.c_oflag  &=~(OPOST|ONLCR|OCRNL|ONOCR|ONLRET);

    /* control modes */
    cur_termios.c_cflag &= ~(CSIZE|CSTOPB|CREAD|PARENB|PARODD|HUPCL|CLOCAL|CRTSCTS);
    cur_termios.c_cflag |= CREAD|CS8|CLOCAL;

    /* local modes */
    cur_termios.c_lflag &= ~(ISIG|ICANON|IEXTEN|ECHO|FLUSHO|PENDIN);
    cur_termios.c_lflag |= NOFLSH;

    if (cfsetispeed(&cur_termios, B0))
    {
        perror("setting input modem serial device speed");
        return( -1 );
    }

    if (cfsetospeed(&cur_termios, br))
    {
        perror("setting modem serial device speed");
        return( -1 );
    }

    if (tcsetattr(serial_fd, TCSADRAIN, &cur_termios))
    {
        perror("setting modem serial device attr");
        return( -1 );
    }


/* enable this to record data */
#if 0
    printf("receiving\n");
    while (1)
    {
        FD_ZERO( &read_fd_set );
        FD_SET( 0, &read_fd_set );
        FD_SET( serial_fd, &read_fd_set );
        max_fd = serial_fd;
        i = select( max_fd + 1,
                    &read_fd_set,
                    NULL,
                    NULL,
                    &select_tv );

        if ( FD_ISSET(serial_fd, &read_fd_set) )
        {
            read(serial_fd, &data_temp, 1);
printf("  0x%02X %c\n", data_temp, data_temp);
//            if (data_temp == 0x7E) printf("\n");
//            printf("0x%02X, ", data_temp);
        }
        else
        {
        }
    }
#endif


    /* this is the interesting part  */
    {
        /*

            XBee-message: ABCDxxxxxxxE
                A XBEE_START (0x7E)
                B LENGTH_MSB (D->D)
                C LENGTH_LSB
                D XBEE_PAYLOAD
                  0 XBEE_TX16 (0x01) / XBEE_RX16 (0x81)
                  1 FRAME_ID (0)     / SRC_ID_MSB
                  2 DEST_ID_MSB      / SRC_ID_LSB
                  3 DEST_ID_LSB      / XBEE_RSSI
                  4 TX16_OPTIONS (0) / RX16_OPTIONS
                  5 PPRZ_DATA
                    0 SENDER_ID
                    1 MSG_ID
                    2 MSG_PAYLOAD
                    . DATA (messages.xml)
                E XBEE_CHECKSUM (sum[D->D])

            <message name="WINDTURBINE_STATUS_" id="50">
            <field name="ac_id" type="uint8"/>
            <field name="tb_id" type="uint8"/>
            <field name="sync_itow"  type="uint32" unit="ms"/>
            <field name="cycle_time" type="uint32" unit="ms"/>
            </message>

        */

        /* valid example data */
        unsigned char data[] = { 0x7E,       // XBEE_START
                                 0x00, 0x11, // PAYLOAD LENGTH (0x0011)
                                 0x01,       // XBEE TX_16
                                 0x37,       // FRAME_ID (incremented)
                                 0xFF, 0xFF, // DESTINATION ID 0xFFFF (broadcast)
                                 0x00,       // TX16_OPTIONS (none)
                                             // Paparazzi message
                                 0x00,       // SENDER_ID (0)
                                 0x32,       // MSG_ID 0x32 = 50 (WINDTURBINE_STATUS)
                                 0x2A,         // ac_id 42 (target aircraft id)
                                 0x01,         // tb_id 01 (source turbine id)
                                 0x39, 0x30, 0x00, 0x00, // sync_itow
                                 0x32, 0x09, 0x01, 0x00, // cycle_time
                                 0xC7 };     // CHECKSUM

        /* this should be possible to set by defines in script */
        unsigned char ac_id = 23;
        unsigned char tb_id = 1;

        /* this is the data estimated */
        unsigned int sync_itow;
        unsigned int cycle_time;

        /* this is calculated in the routine*/
        unsigned char frame_id = 0;
        unsigned char checksum = 0;

        #define AT_COMMAND_SEQUENCE "+++"
        #define AT_SET_MY "ATMY0101\r"
        #define AT_AP_MODE "ATAP1\r"
        #define AT_EXIT "ATCN\r"

        /* switching to AT mode */
        write(serial_fd, AT_COMMAND_SEQUENCE, 3);

        /* busy wait 1.25s */
        usleep(1250000);

        /* setting my address to 0x0101 */
        write(serial_fd, AT_SET_MY, 9);

        /* start API */
        write(serial_fd, AT_AP_MODE, 6);

        /* switching back to normal mode */
        write(serial_fd, AT_EXIT, 5);

        while(1)
        {
            /* increase for each message*/
            frame_id++;

            /* should be calculated - dummy values here */
            sync_itow = 12345678;
            cycle_time = 87654321;

            /* set values in message */
            data[4]  = frame_id;
            data[10] = ac_id;
            data[11] = tb_id;
            data[12] =  sync_itow & 0xFF;
            data[13] = (sync_itow >> 8) & 0xFF;
            data[14] = (sync_itow >> 16) & 0xFF;
            data[15] = (sync_itow >> 24) & 0xFF;
            data[16] =  cycle_time & 0xFF;
            data[17] = (cycle_time >> 8) & 0xFF;
            data[18] = (cycle_time >> 16) & 0xFF;
            data[19] = (cycle_time >> 24) & 0xFF;

            /* calculate checksum */
            checksum = 0;
            for (i=3; i<20; i++) checksum += data[i];
            checksum = 0xFF - checksum;

            /* set checksum */
            data[20] = checksum;

            /* send data */
            write(serial_fd, data, 21);
            sleep(1);
        }
    }

    close(serial_fd);

    return(0);
}
