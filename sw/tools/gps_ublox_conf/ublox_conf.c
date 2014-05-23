/*
 * Paparazzi ublox gps rx configurator $Id$
 *
 * Copyright (C) 2005 Martin Mueller
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
#include <getopt.h>
#include <stdlib.h>


/* *********************** change this to your needs *********************** */

/* serial port that is used at the ublox gps receiver (1 or 2) */
#define UBLOX_PORT          1
/* serial speed that for the ublox gps receiver at startup (usb: don't care) */
#define DEFAULT_BAUDRATE    9600
/* this is the name of the input configuration text from ublox */
#define IN_FILE_NAME        "Tiny_LEA-4P-v6.txt"
/* this is where your gps receiver lives on the PC (ttyS0, ttyACM0, ttyUSB0) */
#define OUT_FILE_NAME       "/dev/ttyUSB0"
/* should the config be written to flash/battery backed ram? (1=yes) */
#define SAVE_PERMANENT      1

/* **************** no user servicable part below this line **************** */


#define SYNC_CHAR_1     0xB5
#define SYNC_CHAR_2     0x62
#define BLANK_CHAR      ' '
#define MINUS_CHAR      '-'

#define NAV             0x01
#define RXM             0x02
#define INF             0x04
#define ACK             0x05
#define ACK_ACK         0x01
#define CFG             0x06
#define CFG_PRT         0x00
#define CFG_MSG         0x01
#define CFG_INF         0x02
#define CFG_DAT         0x06
#define CFG_TP          0x07
#define CFG_RATE        0x08
#define CFG_CFG         0x09
#define CFG_TM          0x10
#define CFG_RXM         0x11
#define CFG_FXN         0x0E
#define CFG_ANT         0x13
#define CFG_SBAS        0x16
#define CFG_NMEA        0x17
#define UPD             0x09
#define MON             0x0A
#define MON_VER         0x04
#define AID             0x0B
#define TIM             0x0D

#define ACK_ACK_LENGTH  0x0A
#define CFG_CFG_LENGTH  0x14

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


int wait_for_ack( unsigned char* data, int serial_fd )
{
    /* the acknowledge buffer to check if the gps understood it */
    unsigned char ack_ack[ACK_ACK_LENGTH] = {
        SYNC_CHAR_1, SYNC_CHAR_2, ACK, ACK_ACK, 0x02, 0x00 };
    struct timeval select_tv;
    fd_set read_fd_set;
    int max_fd;
    int i;
    int ack_index;
    unsigned char data_temp;

    ack_index = 0;

    /* calc ACK checksum */
    ack_ack[6] = *(data+2);
    ack_ack[7] = *(data+3);
    ack_ack[8] = 0;
    ack_ack[9] = 0;

    for (i=2; i < ACK_ACK_LENGTH-2; i++)
    {
        ack_ack[8] = ack_ack[8] + ack_ack[i];
        ack_ack[9] = ack_ack[9] + ack_ack[8];
    }

    /* 900 msec timeout for reply */
    select_tv.tv_sec = 0;
    select_tv.tv_usec = 900000;

    while (1)
    {
        /* wait for ACK-ACK */
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
            if (data_temp != ack_ack[ack_index++])
            {
                ack_index = 0;
            }
            if (ack_index == ACK_ACK_LENGTH)
            {
                printf("OK\n");
                return(0);
            }
        }
        else
        {
            printf("*** no ACK from gps rx ***\n");
            return(-1);
        }
    }
}


int main (int argc, char **argv)
{
    char *in_file_name  = NULL;
    char *out_file_name = OUT_FILE_NAME;

    struct termios orig_termios, cur_termios;

    /* the general buffer for config read from file */
    unsigned char data[65536] = { SYNC_CHAR_1, SYNC_CHAR_2 };
    /* this packet saves the just sent CFG commands */
    unsigned char cfg_cfg[CFG_CFG_LENGTH] = {
        SYNC_CHAR_1, SYNC_CHAR_2, CFG, CFG_CFG, 0x0C, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0xFF, 0xFF, 0xFF, 0xFF,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00 };
    unsigned char data_temp;

    FILE *in_file;
    int serial_fd;
    int count, i;
    int br;
    int baud;
    int save_perm = SAVE_PERMANENT;
    int baudrate = DEFAULT_BAUDRATE;
    int ublox_port = UBLOX_PORT;

    while ((i=getopt(argc, argv, "p:b:s:d:")) != -1)
    {
        switch(i)
        {
            case'p':
                ublox_port = strtoul(optarg, 0, 0);
                break;
            case'b':
                baudrate = strtoul(optarg, 0, 0);
                break;
            case's':
                save_perm = strtoul(optarg, 0, 0);
                break;
            case'd':
                out_file_name = optarg;
                break;
        }
    }

    if (optind == argc - 1)
    {
        in_file_name = argv[ optind ];
        ++optind;
    }

    if (in_file_name == NULL)
    {
//        fprintf(stderr, usage_str);
        fprintf(stderr,
                "ublox_conf\nConfigures u-blox GPS receivers\n");
        fprintf(stderr,
                "  <file>    : configuration file name (example: %s)\n",
                IN_FILE_NAME);
        fprintf(stderr,
                "  -d <dev>  : GPS device name (default: %s)\n",
                OUT_FILE_NAME);
        fprintf(stderr,
                "  -b <baud> : initial GPS receiver baud rate (default: %d)\n",
                DEFAULT_BAUDRATE);
        fprintf(stderr,
                "  -s <0|1>  : save config to Flash/battery backed RAM (default: %d)\n",
                SAVE_PERMANENT);
        fprintf(stderr,
                "  -p <port> : internal u-blox receiver port (default: %d)\n",
                UBLOX_PORT);
        exit(2);
    }

    if ((br = int_to_baud(baudrate)) < 0 )
    {
        printf("invalid baudrate %d\n", baudrate);
        return( -1 );
    }

    if ( (in_file = fopen( in_file_name, "r" )) == NULL )
    {
        perror("could not open in file");
        return( -1 );
    }

    if ( (serial_fd = open( out_file_name, O_RDWR )) == 0 )
    {
        perror("could not open out file");
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

    while (!feof(in_file))
    {
        /* search for first ' ' character */
        while( ((data_temp = fgetc(in_file)) != BLANK_CHAR) &&
               (!feof(in_file)) )
        {
            printf("%c", data_temp);
        }

        /* check for minus sign */
        if ( (!feof(in_file)) &&
             (fgetc(in_file) != MINUS_CHAR) )
        {
            printf("format error (no minus)\n");
            return(-1);
        }

        /* search for second ' ' character */
        while( (fgetc(in_file) != BLANK_CHAR) &&
               (!feof(in_file)) ){}

        if (feof(in_file)) break;

        printf(" ");

        /* read message */
        count = 2;
        do
        {
            data_temp = fgetc(in_file);

            if ( (data_temp <= '9') && (data_temp >= '0') )
            {
                data[count] = (data_temp - '0') << 4;
            }
            else
            if ( (data_temp <= 'f') && (data_temp >= 'a') )
            {
                data[count] = (data_temp - 'a' + 10) << 4;
            }
            else
            if ( (data_temp <= 'F') && (data_temp >= 'A') )
            {
                data[count] = (data_temp - 'A' + 10) << 4;
            }
            else
            {
                printf("format error (letters a)\n");
                return(-1);
            }

            data_temp = fgetc(in_file);

            if ( (data_temp <= '9') && (data_temp >= '0') )
            {
                data[count] += (data_temp - '0');
            }
            else
            if ( (data_temp <= 'f') && (data_temp >= 'a') )
            {
                data[count] += (data_temp - 'a' + 10);
            }
            else
            if ( (data_temp <= 'F') && (data_temp >= 'A') )
            {
                data[count] += (data_temp - 'A' + 10);
            }
            else
            {
                printf("format error (letters b)\n");
                return(-1);
            }
            count++;
        }
        while( (fgetc(in_file) == BLANK_CHAR) &&
               (!feof(in_file)) );

        if ( (data[4] + (data[5] << 8) + 4) != count-2)
        {
            printf("format error (length)");
            return(-1);
        }

        /* calc checksum */
        data[count]   = 0;
        data[count+1] = 0;

        for (i=2; i < count; i++)
        {
            data[count]   = data[count]   + data[i];
            data[count+1] = data[count+1] + data[count];
        }

        /* send data */
        write(serial_fd, data, count+2);

        /* did we configure something: check for acknowledge */
        if (data[2] == CFG)
        {
            /* did we change baudrate: change PC as well */
            if (data[3] == CFG_PRT)
            {
                baud = (data[6+8]) +
                       (data[6+9] << 8) +
                       (data[6+10] << 16) +
                       (data[6+11] << 24);

                if (data[6] == ublox_port)
                {
                    sleep(1);
                    printf("port in use, ");

                    if ((br = int_to_baud(baud)) < 0 )
                    {
                        printf("invalid baudrate %d\n", baud);
                        break;
                    }
                    if (cfsetospeed(&cur_termios, br))
                    {
                        perror("setting modem serial device speed");
                        return( -1 );
                    }
                    else
                    {
                        if (tcsetattr(serial_fd, TCSADRAIN, &cur_termios))
                        {
                            perror("setting modem serial device attr");
                            return( -1 );
                        }
                        else
                        {
                            printf("baudrate set to %d ", baud);
                        }
                    }
                }
            }

            wait_for_ack( data, serial_fd );
        }
        /* monitor command? */
        else if (data[2] == MON)
        {
            /* monitor version command? */
            if (data[3] == MON_VER)
            {
                /* print it */
                for (i=6; i < count; i++)
                {
                    printf("%c", data[i]);
                }
                printf("\n");
            }
        }
        else
        {
            printf("-\n");
        }

    }

    /* save it forever? */
    if (save_perm != 0)
    {
        printf("CFG-CFG writing config to permanent memory ");
        count = CFG_CFG_LENGTH-2;
        for (i=2; i < count; i++)
        {
            cfg_cfg[count]   = cfg_cfg[count]   + cfg_cfg[i];
            cfg_cfg[count+1] = cfg_cfg[count+1] + cfg_cfg[count];
        }

        /* send data */
        write(serial_fd, cfg_cfg, count+2);

        wait_for_ack( cfg_cfg, serial_fd );
    }

    fclose(in_file);
    close(serial_fd);

    return(0);
}

