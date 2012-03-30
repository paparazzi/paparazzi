#ifndef ADXL345_H
#define ADXL345_H

/* default I2C address */
#define ADXL345_ADDR            0xA6
#define ADXL345_ADDR_ALT        0x3A

/* Registers */
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_INT_ENABLE  0x2E
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATA_X0     0x32
#define ADXL345_REG_DATA_X1     0x33
#define ADXL345_REG_DATA_Y0     0x34
#define ADXL345_REG_DATA_Y1     0x35
#define ADXL345_REG_DATA_Z0     0x36
#define ADXL345_REG_DATA_Z1     0x37

/* Selectable data rates in ADXL345_REG_BW_RATE
 * bandwith is always half of data rate
 */
#define ADXL345_RATE_3200 0x0F
#define ADXL345_RATE_1600 0x0E
#define ADXL345_RATE_800  0x0D
#define ADXL345_RATE_400  0x0C
#define ADXL345_RATE_200  0x0B
#define ADXL345_RATE_100  0x0A
#define ADXL345_RATE_50   0x09

/* data format bits, range */
#define ADXL345_INT_INVERT  0x20
#define ADXL345_FULL_RES    0x08
#define ADXL345_JUSTIFY_MSB 0x04
#define ADXL345_RANGE_16G   0x03
#define ADXL345_RANGE_8G    0x02
#define ADXL345_RANGE_4G    0x01
#define ADXL345_RANGE_2G    0x00

#endif /* ADXL345_H */
