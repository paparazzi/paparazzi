#ifndef MPU60X0
#define MPU60X0

/* default I2C address */
#define MPU60X0_ADDR            0xD0
#define MPU60X0_ADDR_ALT        0xD2

/* Registers */

#define MPU60X0_AUX_VDDIO		0x01
// Must be set to 0 on MPU6000


#define MPU60X0_REG_WHO_AM_I    	0X75
#define MPU60X0_REG_SMPLRT_DIV  	0X19
#define MPU60X0_REG_CONFIG      	0X1A
#define MPU60X0_REG_GYRO_CONFIG 	0X1B
#define MPU60X0_REG_ACC_CONFIG  	0X1C
#define MPU60X0_REG_FIFO_EN     	0X23

#define MPU60X0_REG_I2C_MSTR    	0X24
#define MPU60X0_REG_I2C_MSTR_STATUS    	0X24
// Slave 0
#define MPU60X0_REG_I2C_SLV0_1  0X25	// i2c addr
#define MPU60X0_REG_I2C_SLV0_2  0X26	// slave reg
#define MPU60X0_REG_I2C_SLV0_3  0X27	// set-bits
// Slave 1
#define MPU60X0_REG_I2C_SLV1_1  0X28	// i2c addr
#define MPU60X0_REG_I2C_SLV1_2  0X29	// slave reg
#define MPU60X0_REG_I2C_SLV1_3  0X2A	// set-bits
// Slave 2
#define MPU60X0_REG_I2C_SLV2_1  0X2B	// i2c addr
#define MPU60X0_REG_I2C_SLV2_2  0X2C	// slave reg
#define MPU60X0_REG_I2C_SLV2_3  0X2D	// set-bits
// Slave 3
#define MPU60X0_REG_I2C_SLV3_1  0X2E	// i2c addr
#define MPU60X0_REG_I2C_SLV3_2  0X2F	// slave reg
#define MPU60X0_REG_I2C_SLV3_3  0X30	// set-bits
// Slave 4 - special
#define MPU60X0_REG_I2C_SLV4_1  0X31	// i2c addr
#define MPU60X0_REG_I2C_SLV4_2  0X32	// slave reg
#define MPU60X0_REG_I2C_SLV4_3  0X33	// DO
#define MPU60X0_REG_I2C_SLV4_4  0X34	// set-bits
#define MPU60X0_REG_I2C_SLV4_5  0X35	// DI

//TODO
#define MPU60X0_REG_TEMP_OUT_L  0X1C
#define MPU60X0_REG_GYRO_XOUT_H 0X1D
#define MPU60X0_REG_GYRO_XOUT_L 0X1E
#define MPU60X0_REG_GYRO_YOUT_H 0X1F
#define MPU60X0_REG_GYRO_YOUT_L 0X20
#define MPU60X0_REG_GYRO_ZOUT_H 0X21
#define MPU60X0_REG_GYRO_ZOUT_L 0X22
#define MPU60X0_REG_PWR_MGM     0X3E



/////////////////////////////////////////////////
// MPU60X0 Definitions

#define MPU60X0_WHOAMI_REPLY	0x68



#endif /* MPU60X0 */
