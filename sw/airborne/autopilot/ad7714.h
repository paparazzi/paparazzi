#ifndef AD_7714_H
#define AD_7714_H

#include <inttypes.h>

/* AD7714 COM register */
#define AD_COM_REG    (0<<4)
#define AD_MODE_REG   (1<<4)
#define AD_F_HIGH_REG (2<<4)
#define AD_F_LOW_REG  (3<<4)
#define AD_TEST_REG   (4<<4)
#define AD_DATA_REG   (5<<4)
#define AD_ZS_CAL_REG (6<<4)
#define AD_FS_CAL_REG (7<<4)

#define AD_WR         (1<<3)

#define AD_FD0        (4<<0)
#define AD_FD1        (5<<0)
#define AD_FD2        (6<<0)

/*  AD7714 MODE register */
#define AD_NOR_MOD        (0<<5)
#define AD_SELFCAL_MOD    (1<<5)
#define AD_ZS_SYSCAL_MOD  (2<<5)
#define AD_FS_SYSCAL_MOD  (3<<5)
#define AD_SYSOFFCAL_MOD  (4<<5)
#define AD_BG_CAL_MOD     (5<<5)
#define AD_ZS_SELFCAL_MOD (6<<5)
#define AD_FS_SELFCAL_MOD (7<<5)

#define AD_GAIN_1      (0<<2)
#define AD_GAIN_2      (1<<2)
#define AD_GAIN_4      (2<<2)
#define AD_GAIN_8      (3<<2)
#define AD_GAIN_16     (4<<2)
#define AD_GAIN_32     (5<<2)
#define AD_GAIN_64     (6<<2)
#define AD_GAIN_128    (7<<2)

#define AD_BO     (1<<1)
#define AD_FSYNC  (1<<0)

#define Ad7714_Disable_It() { cbi(EIMSK, INT6); } 

#define Ad7714_Enable_It() { sbi(EIMSK, INT6); } 

#define Ad7714_Ready() (bit_is_clear(EIFR, INTF6))

extern uint16_t ad7714_sample;
extern uint8_t  ad7714_sample_read;
void ad7714_init( void );
void ad7714_on_spi_it( void );
void ad7714_on_it( void );

#endif /* AD_7714_H */
