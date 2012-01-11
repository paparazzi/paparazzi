#ifndef MPU60X0
#define MPU60X0

/* default I2C address */
#define MPU60X0_ADDR            0xD0
#define MPU60X0_ADDR_ALT        0xD2

#define MPU60X0_SPI_READ	0x80

// Power and Interface
#define MPU60X0_REG_AUX_VDDIO		0x01	// Must be set to 0 on MPU6000
#define MPU60X0_REG_USER_CTRL		0x6A
#define MPU60X0_REG_PWR_MGMT_1		0x6B
#define MPU60X0_REG_PWR_MGMT_2		0x6C

// FIFO
#define MPU60X0_REG_FIFO_EN     	0X23
#define MPU60X0_REG_FIFO_COUNT_H	0x72
#define MPU60X0_REG_FIFO_COUNT_L	0x73
#define MPU60X0_REG_FIFO_R_W		0x74

// Measurement Settings
#define MPU60X0_REG_SMPLRT_DIV  	0X19
#define MPU60X0_REG_CONFIG      	0X1A
#define MPU60X0_REG_GYRO_CONFIG 	0X1B
#define MPU60X0_REG_ACCEL_CONFIG  	0X1C

// I2C Slave settings
#define MPU60X0_REG_I2C_MST_CTRL    	0X24
#define MPU60X0_REG_I2C_MST_STATUS    	0X36
#define MPU60X0_REG_I2C_MST_DELAY    	0X67
// Slave 0
#define MPU60X0_REG_I2C_SLV0_ADDR	0X25	// i2c addr
#define MPU60X0_REG_I2C_SLV0_REG	0X26	// slave reg
#define MPU60X0_REG_I2C_SLV0_CTRL	0X27	// set-bits
#define MPU60X0_REG_I2C_SLV0_DO		0X63	// DO
// Slave 1
#define MPU60X0_REG_I2C_SLV1_ADDR	0X28	// i2c addr
#define MPU60X0_REG_I2C_SLV1_REG	0X29	// slave reg
#define MPU60X0_REG_I2C_SLV1_CTRL	0X2A	// set-bits
#define MPU60X0_REG_I2C_SLV1_DO		0X64	// DO
// Slave 2
#define MPU60X0_REG_I2C_SLV2_ADDR	0X2B	// i2c addr
#define MPU60X0_REG_I2C_SLV2_REG	0X2C	// slave reg
#define MPU60X0_REG_I2C_SLV2_CTRL	0X2D	// set-bits
#define MPU60X0_REG_I2C_SLV2_DO		0X65	// DO
// Slave 3
#define MPU60X0_REG_I2C_SLV3_ADDR	0X2E	// i2c addr
#define MPU60X0_REG_I2C_SLV3_REG	0X2F	// slave reg
#define MPU60X0_REG_I2C_SLV3_CTRL	0X30	// set-bits
#define MPU60X0_REG_I2C_SLV3_DO		0X66	// DO
// Slave 4 - special
#define MPU60X0_REG_I2C_SLV4_ADDR	0X31	// i2c addr
#define MPU60X0_REG_I2C_SLV4_REG	0X32	// slave reg
#define MPU60X0_REG_I2C_SLV4_DO		0X33	// DO
#define MPU60X0_REG_I2C_SLV4_CTRL	0X34	// set-bits
#define MPU60X0_REG_I2C_SLV4_DI		0X35	// DI

// Interrupt
#define MPU60X0_REG_INT_PIN_CFG     	0X37
#define MPU60X0_REG_INT_ENABLE  	0X38
#define MPU60X0_REG_INT_STATUS  	0X3A

// Accelero
#define MPU60X0_REG_ACCEL_XOUT_H 	0X3B
#define MPU60X0_REG_ACCEL_XOUT_L 	0X3C
#define MPU60X0_REG_ACCEL_YOUT_H 	0X3D
#define MPU60X0_REG_ACCEL_YOUT_L 	0X3E
#define MPU60X0_REG_ACCEL_ZOUT_H 	0X3F
#define MPU60X0_REG_ACCEL_ZOUT_L 	0X40

// Temperature
#define MPU60X0_REG_TEMP_OUT_H   	0X41
#define MPU60X0_REG_TEMP_OUT_L   	0X42

// Gyro
#define MPU60X0_REG_GYRO_XOUT_H  	0X43
#define MPU60X0_REG_GYRO_XOUT_L  	0X44
#define MPU60X0_REG_GYRO_YOUT_H  	0X45
#define MPU60X0_REG_GYRO_YOUT_L  	0X46
#define MPU60X0_REG_GYRO_ZOUT_H  	0X47
#define MPU60X0_REG_GYRO_ZOUT_L  	0X48

// External Sensor Data
#define MPU60X0_EXT_SENS_DATA    	0X49
#define MPU60X0_EXT_SENS_DATA_SIZE 	24


/////////////////////////////////////////////////
// MPU60X0 Definitions

#define MPU60X0_REG_WHO_AM_I    	0X75
#define MPU60X0_WHOAMI_REPLY		0x68



#endif /* MPU60X0 */
