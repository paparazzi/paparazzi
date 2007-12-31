/*
 * i2c_usb.c - test application for the i2c-tiby-usb interface
 *             http://www.harbaum.org/till/i2c_tiny_usb
 *
 *             adapted to Melexis MLX 90614 adapter
 *
 *
 * $Id$
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb.h>

/* mlx90614 chip address (default address) */
#define MLX90614_ADDR    0x00
#define MLX90614_ADDR_1  0x02
#define MLX90614_ADDR_2  0x05

#define LOOPS 100

#define USB_CTRL_IN    (USB_TYPE_CLASS | USB_ENDPOINT_IN)
#define USB_CTRL_OUT   (USB_TYPE_CLASS)

/* the vendor and product id was donated by ftdi ... many thanks!*/
#define I2C_TINY_USB_VID  0x0403
#define I2C_TINY_USB_PID  0xc631

#ifdef WIN
#include <windows.h>
#include <winbase.h>
#define usleep(t) Sleep((t) / 1000)
#endif

#define I2C_M_RD		0x01

/* commands via USB, must e.g. match command ids firmware */
#define CMD_ECHO       0
#define CMD_GET_FUNC   1
#define CMD_SET_DELAY  2
#define CMD_GET_STATUS 3
#define CMD_I2C_IO     4
#define CMD_I2C_BEGIN  1  // flag to I2C_IO
#define CMD_I2C_END    2  // flag to I2C_IO

#define STATUS_IDLE          0
#define STATUS_ADDRESS_ACK   1
#define STATUS_ADDRESS_NAK   2

usb_dev_handle      *handle = NULL;

unsigned crc8(unsigned char* dat, int len)
{
  unsigned bits = 0;
  int i;
  
  /* x^8 + x^2 + x^1 + x^0 */
  while (len-- > 0) {
    for (i = 7; i >= 0; i--) {
	  bits = (bits << 1) | (((*dat) & (1 << i)) >> i);
      if (bits > 0xFF)
        bits ^= 0x107;
    }
    dat++;
  }

  for (i = 7; i >= 0; i--) {
    bits <<= 1;
    if (bits > 0xFF)
      bits ^= 0x107;
  }
  
  return bits;
}

/* write a set of bytes to the i2c_tiny_usb device */
int i2c_tiny_usb_write(int request, int value, int index) {
  if(usb_control_msg(handle, USB_CTRL_OUT, request, 
		      value, index, NULL, 0, 1000) < 0) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    return -1;
  }
  return 1;
}

/* read a set of bytes from the i2c_tiny_usb device */
int i2c_tiny_usb_read(unsigned char cmd, void *data, int len) {
  int                 nBytes;

  /* send control request and accept return value */
  nBytes = usb_control_msg(handle, 
	   USB_CTRL_IN, 
	   cmd, 0, 0, data, len, 1000);

  if(nBytes < 0) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    return nBytes;
  }

  return 0;
}

/* get i2c usb interface firmware version */
void i2c_tiny_usb_get_func(void) {
  unsigned long func;
  
  if(i2c_tiny_usb_read(CMD_GET_FUNC, &func, sizeof(func)) == 0)
    printf("Functionality = %lx\n", func);
}

/* set a value in the I2C_USB interface */
void i2c_tiny_usb_set(unsigned char cmd, int value) {
  if(usb_control_msg(handle, 
	     USB_TYPE_VENDOR, cmd, value, 0, 
	     NULL, 0, 1000) < 0) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
  }
}

/* get the current transaction status from the i2c_tiny_usb interface */
int i2c_tiny_usb_get_status(void) {
  int i;
  unsigned char status;
  
  if((i=i2c_tiny_usb_read(CMD_GET_STATUS, &status, sizeof(status))) < 0) {
    fprintf(stderr, "Error reading status\n");
    return i;
  }

  return status;
}

/* write command and read an 8 or 16 bit value from the given chip */
int i2c_read_with_cmd(unsigned char addr, char cmd, int length) {
  unsigned char result[2];

  if((length < 0) || (length > sizeof(result))) {
    fprintf(stderr, "request exceeds %d bytes\n", sizeof(result));
    return -1;
  } 

  /* write one byte register address to chip */
  if(usb_control_msg(handle, USB_CTRL_OUT, 
		     CMD_I2C_IO + CMD_I2C_BEGIN
		     + ((!length)?CMD_I2C_END:0),
		     0, addr, &cmd, 1, 
		     1000) < 1) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    return -1;
  } 

  if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
    fprintf(stderr, "write command status failed\n");
    return -1;
  }

  // just a test? return ok
  if(!length) return 0;

  if(usb_control_msg(handle, 
		     USB_CTRL_IN, 
		     CMD_I2C_IO + CMD_I2C_END,
		     I2C_M_RD, addr, (char*)result, length, 
		     1000) < 1) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    return -1;
  } 

  if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
    fprintf(stderr, "read data status failed\n");
    return -1;
  }

  // return 16 bit result
  if(length == 2)
    return 256*result[0] + result[1];

  // return 8 bit result
  return result[0];  
}

/* write command and read an 16 bit value from mlx */
int i2c_mlx_read_word_with_cmd(unsigned char addr, char cmd) {
  unsigned char result[3];
  unsigned char pec[6];

  /* write one byte register address to chip */
  if(usb_control_msg(handle, USB_CTRL_OUT, 
		     CMD_I2C_IO + CMD_I2C_BEGIN,
		     0, addr, &cmd, 1, 
		     1000) < 1) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    return -1;
  } 

  if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
    fprintf(stderr, "write command status failed\n");
    return -1;
  }

  if(usb_control_msg(handle, 
		     USB_CTRL_IN, 
		     CMD_I2C_IO + CMD_I2C_END,
		     I2C_M_RD, addr, (char*)result, 4, 
		     1000) < 1) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    return -1;
  } 

  if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
    fprintf(stderr, "read data status failed\n");
    return -1;
  }

  pec[0]=addr<<1;
  pec[1]=cmd;
  pec[2]=(addr<<1)|1;
  pec[3]=result[0];
  pec[4]=result[1];

  if (result[2] != crc8(pec, 5))
    return -2;

  /* LSB first */
  return 256*result[1] + result[0];
}

/* write a single byte to the i2c client */
int i2c_write_byte(unsigned char addr, char data) {

  /* write one byte register address to chip */
  if(usb_control_msg(handle, USB_CTRL_OUT, 
		     CMD_I2C_IO + CMD_I2C_BEGIN + CMD_I2C_END,
		     0, addr, &data, 1, 
		     1000) < 1) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    return -1;
  } 

  if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
    fprintf(stderr, "write command status failed\n");
    return -1;
  }

  return 0;  
}

/* write a command byte and a single byte to the i2c client */
int i2c_write_cmd_and_byte(unsigned char addr, char cmd, char data) {
  char msg[2];

  msg[0] = cmd;
  msg[1] = data;

  /* write one byte register address to chip */
  if(usb_control_msg(handle, USB_CTRL_OUT, 
		     CMD_I2C_IO + CMD_I2C_BEGIN + CMD_I2C_END,
		     0, addr, msg, 2, 
		     1000) < 1) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    return -1;
  } 

  if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
    fprintf(stderr, "write command status failed\n");
    return -1;
  }

  return 0;  
}

/* write a command byte and a 16 bit value to the mlx */
int i2c_mlx_write_cmd_and_word(unsigned char addr, char cmd, int data) {
  char msg[4];
  unsigned char pec[5];
    
  pec[0]=addr<<1;
  pec[1]=cmd;
  pec[2]=data & 0xff;
  pec[3]=data >> 8;

  msg[0] = cmd;
  msg[1] = data & 0xff;
  msg[2] = data >> 8;
  msg[3] = crc8(pec, 4);

  /* write one byte register address to chip */
  if(usb_control_msg(handle, USB_CTRL_OUT, 
		     CMD_I2C_IO + CMD_I2C_BEGIN + CMD_I2C_END,
		     0, addr, msg, 4, 
		     1000) < 1) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    return -1;
  } 

  if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
    fprintf(stderr, "write command status failed\n");
    return -1;
  }

  return 0;  
}

/* write a command byte and a 16 bit value to the mlx i2c client */
int i2c_write_cmd_and_word(unsigned char addr, char cmd, int data) {
  char msg[4];

  msg[0] = cmd;
  msg[1] = data >> 8;
  msg[2] = data & 0xff;

  /* write one byte register address to chip */
  if(usb_control_msg(handle, USB_CTRL_OUT, 
		     CMD_I2C_IO + CMD_I2C_BEGIN + CMD_I2C_END,
		     0, addr, msg, 3, 
		     1000) < 1) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    return -1;
  } 

  if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
    fprintf(stderr, "write command status failed\n");
    return -1;
  }

  return 0;  
}

int main(int argc, char *argv[]) {
  struct usb_bus      *bus;
  struct usb_device   *dev;
  int i;
#ifndef WIN
  int ret;
#endif
  
  printf("--      i2c-tiny-usb test application       --\n");
  printf("--         (c) 2006 by Till Harbaum         --\n");
  printf("-- http://www.harbaum.org/till/i2c_tiny_usb --\n");

  usb_init();
  
  usb_find_busses();
  usb_find_devices();
  
  for(bus = usb_get_busses(); bus; bus = bus->next) {
    for(dev = bus->devices; dev; dev = dev->next) {
      if((dev->descriptor.idVendor == I2C_TINY_USB_VID) && 
	 (dev->descriptor.idProduct == I2C_TINY_USB_PID)) {
	
	printf("Found i2c_tiny_usb device on bus %s device %s.\n", 
	       bus->dirname, dev->filename);
	
	/* open device */
	if(!(handle = usb_open(dev))) 
	  fprintf(stderr, "Error: Cannot open the device: %s\n", 
		  usb_strerror());

	break;
      }
    }
  }
  
  if(!handle) {
    fprintf(stderr, "Error: Could not find i2c_tiny_usb device\n");

#ifdef WIN
    printf("Press return to quit\n");
    getchar();
#endif

    exit(-1);
  }

#ifndef WIN
  /* Get exclusive access to interface 0. Does not work under windows. */
  ret = usb_claim_interface(handle, 0);
  if (ret != 0) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());

    exit(1);
  }
#endif
  
  /* do some testing */
  i2c_tiny_usb_get_func();

  /* try to set i2c clock to 100kHz (10us), will actually result in ~50kHz */
  /* since the software generated i2c clock isn't too exact. in fact setting */
  /* it to 10us doesn't do anything at all since this already is the default */
  i2c_tiny_usb_set(CMD_SET_DELAY, 10);

  i=i;

#if 0
  /* -------- begin of mlx90614 client processing --------- */
  printf("Probing for MLX90614 ... ");

  /* try to access mlx90614 at address MLX90614_ADDR */
  if(usb_control_msg(handle, USB_CTRL_IN, 
		     CMD_I2C_IO + CMD_I2C_BEGIN + CMD_I2C_END,
		     0, MLX90614_ADDR, NULL, 0, 
		     1000) < 0) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    goto quit;
  } 
  
  if(i2c_tiny_usb_get_status() == STATUS_ADDRESS_ACK) {
    int tp;
    
    printf("success at address 0x%02x\n", MLX90614_ADDR);
    usleep(10000);

    tp = i2c_mlx_read_word_with_cmd(MLX90614_ADDR, 0x3C);
    printf("ID = 0x%04X", tp);
    tp = i2c_mlx_read_word_with_cmd(MLX90614_ADDR, 0x3D);
    printf("%04X", tp);
    tp = i2c_mlx_read_word_with_cmd(MLX90614_ADDR, 0x3E);
    printf("%04X", tp);
    tp = i2c_mlx_read_word_with_cmd(MLX90614_ADDR, 0x3F);
    printf("%04X\n", tp);

    tp = i2c_mlx_read_word_with_cmd(MLX90614_ADDR, 0x2E);
    printf("i2c addr = 0x%04X\n", tp);

    /* write new i2c address, always set bit0 ! */
//    i2c_mlx_write_cmd_and_word(MLX90614_ADDR, 0x2E, 1);
//    usleep(1000000);
    
    tp = i2c_mlx_read_word_with_cmd(MLX90614_ADDR, 0x2E);
    printf("i2c addr = 0x%04X\n", tp);

    tp = i2c_mlx_read_word_with_cmd(MLX90614_ADDR, 0x06);
    printf("Ta    = %2.2f°C (0x%04X)\n", (tp*0.02)-273.15, tp);
    
    tp = i2c_mlx_read_word_with_cmd(MLX90614_ADDR, 0x07);
    printf("Tobj1 = %2.2f°C (0x%04X)\n", (tp*0.02)-273.15, tp);

    usleep(100000);
  } else
    printf("failed\n");
  /* -------- end of mlx90614 client processing --------- */
#endif

  /* -------- begin of mlx90614 multi client processing --------- */

  /* try to access mlx90614 at address MLX90614_ADDR_1 */
  if(usb_control_msg(handle, USB_CTRL_IN, 
		     CMD_I2C_IO + CMD_I2C_BEGIN + CMD_I2C_END,
		     0, MLX90614_ADDR_1, NULL, 0, 
		     1000) < 0) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    goto quit;
  } 
  
  if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
    printf("no device at address 0x%02x\n", MLX90614_ADDR_1);
    goto quit;
  }  
  /* try to access mlx90614 at address MLX90614_ADDR_2 */
  if(usb_control_msg(handle, USB_CTRL_IN, 
		     CMD_I2C_IO + CMD_I2C_BEGIN + CMD_I2C_END,
		     0, MLX90614_ADDR_1, NULL, 0, 
		     1000) < 0) {
    fprintf(stderr, "USB error: %s\n", usb_strerror());
    goto quit;
  } 
  
  if(i2c_tiny_usb_get_status() != STATUS_ADDRESS_ACK) {
    printf("no device at address 0x%02x\n", MLX90614_ADDR_2);
    goto quit;
  }  
  
  {  
    int tp1, tp2;
    
    usleep(100000);

    while(1) {
      tp1 = i2c_mlx_read_word_with_cmd(MLX90614_ADDR_1, 0x07);
      if (tp1 == -1) goto quit;
      tp2 = i2c_mlx_read_word_with_cmd(MLX90614_ADDR_2, 0x07);
      if (tp2 == -1) goto quit;
      if ((tp2 == -2) || (tp2 == -2))
        printf("##########\n");
      else
        printf("%2.2f°C\n", ((tp1*0.02)-273.15) - ((tp2*0.02)-273.15));
//      usleep(20000);
    }
  }
  /* -------- end of mlx90614 client processing --------- */

 quit:
#ifndef WIN
  ret = usb_release_interface(handle, 0);
  if (ret)
    fprintf(stderr, "USB error: %s\n", usb_strerror());
#endif

  usb_close(handle);

#ifdef WIN
  printf("Press return to quit\n");
  getchar();
#endif

  return 0;
}
