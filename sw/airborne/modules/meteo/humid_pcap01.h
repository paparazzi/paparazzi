#ifndef PCAP01_H
#define PCAP01_H

#include "std.h"

//#define PCAP01_STANDARD
#define PCAP01_200HZ

typedef struct {
	uint32_t temp;
	uint32_t hum_t;
	uint32_t hum;
	uint32_t R_ratio;
	uint32_t C_ratio;
	uint32_t NV_temp;
	uint32_t V_rh;
	uint32_t status;
}PCAP01VALUE;

#define PCAP01_ADDR 0xA0

#define PCAP01_IDLE			0
#define PCAP01_GET_HUMID	1
#define PCAP01_GET_TEMP		2

//OpCodes für PCap Programmierung
#define PCAP01_PU_RESET 0x88
#define PCAP01_IN_RESET 0x8a
#define PCAP01_START 0x8c
#define PCAP01_TERM 0x84

#define PCAP01_WRITE_REG 0xC0
#define PCAP01_READ_REG 0x40
#define PCAP01_READ_STAT 0x48
#define PCAP01_WRITE_SRAM 0x90
#define PCAP01_WRITE_OTP 0xA0

// Configuration Registers
#define PCAP01_REG0 0x00
#define PCAP01_REG1 0x01
#define PCAP01_REG2 0x02
#define PCAP01_REG3 0x03
#define PCAP01_REG4 0x04
#define PCAP01_REG5 0x05
#define PCAP01_REG6 0x06
#define PCAP01_REG7 0x07
#define PCAP01_REG8 0x08
#define PCAP01_REG9 0x09
#define PCAP01_REG10 0x0A
#define PCAP01_REG11 0x0B
#define PCAP01_REG12 0x0C
#define PCAP01_REG13 0x0D
#define PCAP01_REG14 0x0E
#define PCAP01_REG15 0x0F
#define PCAP01_REG16 0x10
#define PCAP01_REG17 0x11
#define PCAP01_REG18 0x12
#define PCAP01_REG19 0x13
#define PCAP01_REG20 0x14

#ifdef PCAP01_200HZ
// Register configuration values
#define PCAP01_REG0_VALUE 0x420F0F
#define PCAP01_REG1_VALUE 0x201004
#define PCAP01_REG2_VALUE 0x1F460A
#define PCAP01_REG3_VALUE 0x090004
#define PCAP01_REG4_VALUE 0x08040D
#define PCAP01_REG5_VALUE 0xC0001E
#define PCAP01_REG6_VALUE 0x000C40
#define PCAP01_REG7_VALUE 0x1F0000
#define PCAP01_REG8_VALUE 0x800053
#define PCAP01_REG9_VALUE 0x00A88F
#define PCAP01_REG10_VALUE 0x18004B
#define PCAP01_REG11_VALUE 0x000000
#define PCAP01_REG12_VALUE 0x000000
#define PCAP01_REG13_VALUE 0x000000
#define PCAP01_REG14_VALUE 0x000000
#define PCAP01_REG15_VALUE 0x000000
#define PCAP01_REG16_VALUE 0x000000
#define PCAP01_REG17_VALUE 0x000006
#define PCAP01_REG18_VALUE 0x0000A6
#define PCAP01_REG19_VALUE 0x000001
#define PCAP01_REG20_VALUE 0x000001
#endif
#ifdef PCAP01_STANDARD
// Register configuration values
#define PCAP01_REG0_VALUE 0x4200FF
#define PCAP01_REG1_VALUE 0x201022
#define PCAP01_REG2_VALUE 0x0F460B
#define PCAP01_REG3_VALUE 0x070010
#define PCAP01_REG4_VALUE 0x080000
#define PCAP01_REG5_VALUE 0x000000
#define PCAP01_REG6_VALUE 0x000040
#define PCAP01_REG7_VALUE 0x1F0000
#define PCAP01_REG8_VALUE 0xA00010
#define PCAP01_REG9_VALUE 0xFF000F
#define PCAP01_REG10_VALUE 0x180047
#define PCAP01_REG11_VALUE 0x000000
#define PCAP01_REG12_VALUE 0x000000
#define PCAP01_REG13_VALUE 0x000000
#define PCAP01_REG14_VALUE 0x000000
#define PCAP01_REG15_VALUE 0x000000
#define PCAP01_REG16_VALUE 0x000000
#define PCAP01_REG17_VALUE 0x000000
#define PCAP01_REG18_VALUE 0x000000
#define PCAP01_REG19_VALUE 0x200000
#define PCAP01_REG20_VALUE 0x000001
#endif

void writePCAP01_SRAM(uint8_t data, uint16_t s_add);
uint8_t readPCAP01_SRAM(uint16_t s_add);
void PCAP01_Control(uint8_t control);
void pcap01readRegister(uint8_t reg);
void pcap01writeRegister(uint8_t reg,uint32_t value);
void writePCAP01_firmware(void);
void pcap01_init(void);
void pcap01_periodic(void);
void pcap01_event(void);

#endif
