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


#endif /* ADXL345_H */
