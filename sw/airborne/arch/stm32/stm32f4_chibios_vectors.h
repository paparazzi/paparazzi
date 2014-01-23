#pragma once

/*
#                 ______
#                |  ____|
#                | |__     _ __    ___    _ __ ___
#                |  __|   | '__|  / _ \  | '_ ` _ \
#                | |      | |    | (_) | | | | | | |
#                |_|      |_|     \___/  |_| |_| |_|
#                  ____    _ __                    _____   __  __   ____
#                 / __ \  | '_ \                  / ____| |  \/  | |___ \
#                | |  | | | |_) |   ___   _ __   | |      | \  / |   __) |
#                | |  | | | .__/   / _ \ | '_ \  | |      | |\/| |  |__ <
#                | |__| | | |     |  __/ | | | | | |____  | |  | |  ___) |
#                 \____/  |_|      \___| |_| |_|  \_____| |_|  |_| |____/
*/

#define NVIC_NVIC_WWDG_IRQ 0
#define NVIC_PVD_IRQ 1
#define NVIC_TAMP_STAMP_IRQ 2
#define NVIC_RTC_WKUP_IRQ 3
#define NVIC_FLASH_IRQ 4
#define NVIC_RCC_IRQ 5
#define NVIC_EXTI0_IRQ 6
#define NVIC_EXTI1_IRQ 7
#define NVIC_EXTI2_IRQ 8
#define NVIC_EXTI3_IRQ 9
#define NVIC_EXTI4_IRQ 10
#define NVIC_DMA1_STREAM0_IRQ 11
#define NVIC_DMA1_STREAM1_IRQ 12
#define NVIC_DMA1_STREAM2_IRQ 13
#define NVIC_DMA1_STREAM3_IRQ 14
#define NVIC_DMA1_STREAM4_IRQ 15
#define NVIC_DMA1_STREAM5_IRQ 16
#define NVIC_DMA1_STREAM6_IRQ 17
#define NVIC_ADC_IRQ 18
#define NVIC_CAN1_TX_IRQ 19
#define NVIC_CAN1_RX0_IRQ 20
#define NVIC_CAN1_RX1_IRQ 21
#define NVIC_CAN1_SCE_IRQ 22
#define NVIC_EXTI9_5_IRQ 23
#define NVIC_TIM1_BRK_TIM9_IRQ 24
#define NVIC_TIM1_UP_TIM10_IRQ 25
#define NVIC_TIM1_TRG_COM_TIM11_IRQ 26
#define NVIC_TIM1_CC_IRQ 27
#define NVIC_TIM2_IRQ 28
#define NVIC_TIM3_IRQ 29
#define NVIC_TIM4_IRQ 30
#define NVIC_I2C1_EV_IRQ 31
#define NVIC_I2C1_ER_IRQ 32
#define NVIC_I2C2_EV_IRQ 33
#define NVIC_I2C2_ER_IRQ 34
#define NVIC_SPI1_IRQ 35
#define NVIC_SPI2_IRQ 36
#define NVIC_USART1_IRQ 37
#define NVIC_USART2_IRQ 38
#define NVIC_USART3_IRQ 39
#define NVIC_EXTI15_10_IRQ 40
#define NVIC_RTC_ALARM_IRQ 41
#define NVIC_USB_FS_WKUP_IRQ 42
#define NVIC_TIM8_BRK_TIM12_IRQ 43
#define NVIC_TIM8_UP_TIM13_IRQ 44
#define NVIC_TIM8_TRG_COM_TIM14_IRQ 45
#define NVIC_TIM8_CC_IRQ 46
#define NVIC_DMA1_STREAM7_IRQ 47
#define NVIC_FSMC_IRQ 48
#define NVIC_SDIO_IRQ 49
#define NVIC_TIM5_IRQ 50
#define NVIC_SPI3_IRQ 51
#define NVIC_UART4_IRQ 52
#define NVIC_UART5_IRQ 53
#define NVIC_TIM6_DAC_IRQ 54
#define NVIC_TIM7_IRQ 55
#define NVIC_DMA2_STREAM0_IRQ 56
#define NVIC_DMA2_STREAM1_IRQ 57
#define NVIC_DMA2_STREAM2_IRQ 58
#define NVIC_DMA2_STREAM3_IRQ 59
#define NVIC_DMA2_STREAM4_IRQ 60
#define NVIC_ETH_IRQ 61
#define NVIC_ETH_WKUP_IRQ 62
#define NVIC_CAN2_TX_IRQ 63
#define NVIC_CAN2_RX0_IRQ 64
#define NVIC_CAN2_RX1_IRQ 65
#define NVIC_CAN2_SCE_IRQ 66
#define NVIC_OTG_FS_IRQ 67
#define NVIC_DMA2_STREAM5_IRQ 68
#define NVIC_DMA2_STREAM6_IRQ 69
#define NVIC_DMA2_STREAM7_IRQ 70
#define NVIC_USART6_IRQ 71
#define NVIC_I2C3_EV_IRQ 72
#define NVIC_I2C3_ER_IRQ 73
#define NVIC_OTG_HS_EP1_OUT_IRQ 74
#define NVIC_OTG_HS_EP1_IN_IRQ 75
#define NVIC_OTG_HS_WKUP_IRQ 76
#define NVIC_OTG_HS_IRQ 77
#define NVIC_DCMI_IRQ 78
#define NVIC_CRYP_IRQ 79
#define NVIC_HASH_RNG_IRQ 80
#define NVIC_IRQ_COUNT 81

#define CM3_WEAK extern void __attribute__ ((weak))

CM3_WEAK nvic_wwdg_isr(void);
CM3_WEAK pvd_isr(void);
CM3_WEAK tamp_stamp_isr(void);
CM3_WEAK rtc_wkup_isr(void);
CM3_WEAK flash_isr(void);
CM3_WEAK rcc_isr(void);
CM3_WEAK exti0_isr(void);
CM3_WEAK exti1_isr(void);
CM3_WEAK exti2_isr(void);
CM3_WEAK exti3_isr(void);
CM3_WEAK exti4_isr(void);
CM3_WEAK dma1_stream0_isr(void);
CM3_WEAK dma1_stream1_isr(void);
CM3_WEAK dma1_stream2_isr(void);
CM3_WEAK dma1_stream3_isr(void);
CM3_WEAK dma1_stream4_isr(void);
CM3_WEAK dma1_stream5_isr(void);
CM3_WEAK dma1_stream6_isr(void);
CM3_WEAK adc_isr(void);
CM3_WEAK can1_tx_isr(void);
CM3_WEAK can1_rx0_isr(void);
CM3_WEAK can1_rx1_isr(void);
CM3_WEAK can1_sce_isr(void);
CM3_WEAK exti9_5_isr(void);
CM3_WEAK tim1_brk_tim9_isr(void);
CM3_WEAK tim1_up_tim10_isr(void);
CM3_WEAK tim1_trg_com_tim11_isr(void);
CM3_WEAK tim1_cc_isr(void);
CM3_WEAK tim2_isr(void);
CM3_WEAK tim3_isr(void);
CM3_WEAK tim4_isr(void);
CM3_WEAK i2c1_ev_isr(void);
CM3_WEAK i2c1_er_isr(void);
CM3_WEAK i2c2_ev_isr(void);
CM3_WEAK i2c2_er_isr(void);
CM3_WEAK spi1_isr(void);
CM3_WEAK spi2_isr(void);
CM3_WEAK usart1_isr(void);
CM3_WEAK usart2_isr(void);
CM3_WEAK usart3_isr(void);
CM3_WEAK exti15_10_isr(void);
CM3_WEAK rtc_alarm_isr(void);
CM3_WEAK usb_fs_wkup_isr(void);
CM3_WEAK tim8_brk_tim12_isr(void);
CM3_WEAK tim8_up_tim13_isr(void);
CM3_WEAK tim8_trg_com_tim14_isr(void);
CM3_WEAK tim8_cc_isr(void);
CM3_WEAK dma1_stream7_isr(void);
CM3_WEAK fsmc_isr(void);
CM3_WEAK sdio_isr(void);
CM3_WEAK tim5_isr(void);
CM3_WEAK spi3_isr(void);
CM3_WEAK uart4_isr(void);
CM3_WEAK uart5_isr(void);
CM3_WEAK tim6_dac_isr(void);
CM3_WEAK tim7_isr(void);
CM3_WEAK dma2_stream0_isr(void);
CM3_WEAK dma2_stream1_isr(void);
CM3_WEAK dma2_stream2_isr(void);
CM3_WEAK dma2_stream3_isr(void);
CM3_WEAK dma2_stream4_isr(void);
CM3_WEAK eth_isr(void);
CM3_WEAK eth_wkup_isr(void);
CM3_WEAK can2_tx_isr(void);
CM3_WEAK can2_rx0_isr(void);
CM3_WEAK can2_rx1_isr(void);
CM3_WEAK can2_sce_isr(void);
CM3_WEAK otg_fs_isr(void);
CM3_WEAK dma2_stream5_isr(void);
CM3_WEAK dma2_stream6_isr(void);
CM3_WEAK dma2_stream7_isr(void);
CM3_WEAK usart6_isr(void);
CM3_WEAK i2c3_ev_isr(void);
CM3_WEAK i2c3_er_isr(void);
CM3_WEAK otg_hs_ep1_out_isr(void);
CM3_WEAK otg_hs_ep1_in_isr(void);
CM3_WEAK otg_hs_wkup_isr(void);
CM3_WEAK otg_hs_isr(void);
CM3_WEAK dcmi_isr(void);
CM3_WEAK cryp_isr(void);
CM3_WEAK hash_rng_isr(void);



#define NVIC_NVIC_WWDG_IRQ_VEC_CHIBIOS Vector40
#define NVIC_NVIC_WWDG_IRQ_VEC_OPENCM3 nvic_wwdg_isr
#define NVIC_PVD_IRQ_VEC_CHIBIOS Vector44
#define NVIC_PVD_IRQ_VEC_OPENCM3 pvd_isr
#define NVIC_TAMP_STAMP_IRQ_VEC_CHIBIOS Vector48
#define NVIC_TAMP_STAMP_IRQ_VEC_OPENCM3 tamp_stamp_isr
#define NVIC_RTC_WKUP_IRQ_VEC_CHIBIOS Vector4C
#define NVIC_RTC_WKUP_IRQ_VEC_OPENCM3 rtc_wkup_isr
#define NVIC_FLASH_IRQ_VEC_CHIBIOS Vector50
#define NVIC_FLASH_IRQ_VEC_OPENCM3 flash_isr
#define NVIC_RCC_IRQ_VEC_CHIBIOS Vector54
#define NVIC_RCC_IRQ_VEC_OPENCM3 rcc_isr
#define NVIC_EXTI0_IRQ_VEC_CHIBIOS Vector58
#define NVIC_EXTI0_IRQ_VEC_OPENCM3 exti0_isr
#define NVIC_EXTI1_IRQ_VEC_CHIBIOS Vector5C
#define NVIC_EXTI1_IRQ_VEC_OPENCM3 exti1_isr
#define NVIC_EXTI2_IRQ_VEC_CHIBIOS Vector60
#define NVIC_EXTI2_IRQ_VEC_OPENCM3 exti2_isr
#define NVIC_EXTI3_IRQ_VEC_CHIBIOS Vector64
#define NVIC_EXTI3_IRQ_VEC_OPENCM3 exti3_isr
#define NVIC_EXTI4_IRQ_VEC_CHIBIOS Vector68
#define NVIC_EXTI4_IRQ_VEC_OPENCM3 exti4_isr
#define NVIC_DMA1_STREAM0_IRQ_VEC_CHIBIOS Vector6C
#define NVIC_DMA1_STREAM0_IRQ_VEC_OPENCM3 dma1_stream0_isr
#define NVIC_DMA1_STREAM1_IRQ_VEC_CHIBIOS Vector70
#define NVIC_DMA1_STREAM1_IRQ_VEC_OPENCM3 dma1_stream1_isr
#define NVIC_DMA1_STREAM2_IRQ_VEC_CHIBIOS Vector74
#define NVIC_DMA1_STREAM2_IRQ_VEC_OPENCM3 dma1_stream2_isr
#define NVIC_DMA1_STREAM3_IRQ_VEC_CHIBIOS Vector78
#define NVIC_DMA1_STREAM3_IRQ_VEC_OPENCM3 dma1_stream3_isr
#define NVIC_DMA1_STREAM4_IRQ_VEC_CHIBIOS Vector7C
#define NVIC_DMA1_STREAM4_IRQ_VEC_OPENCM3 dma1_stream4_isr
#define NVIC_DMA1_STREAM5_IRQ_VEC_CHIBIOS Vector80
#define NVIC_DMA1_STREAM5_IRQ_VEC_OPENCM3 dma1_stream5_isr
#define NVIC_DMA1_STREAM6_IRQ_VEC_CHIBIOS Vector84
#define NVIC_DMA1_STREAM6_IRQ_VEC_OPENCM3 dma1_stream6_isr
#define NVIC_ADC_IRQ_VEC_CHIBIOS Vector88
#define NVIC_ADC_IRQ_VEC_OPENCM3 adc_isr
#define NVIC_CAN1_TX_IRQ_VEC_CHIBIOS Vector8C
#define NVIC_CAN1_TX_IRQ_VEC_OPENCM3 can1_tx_isr
#define NVIC_CAN1_RX0_IRQ_VEC_CHIBIOS Vector90
#define NVIC_CAN1_RX0_IRQ_VEC_OPENCM3 can1_rx0_isr
#define NVIC_CAN1_RX1_IRQ_VEC_CHIBIOS Vector94
#define NVIC_CAN1_RX1_IRQ_VEC_OPENCM3 can1_rx1_isr
#define NVIC_CAN1_SCE_IRQ_VEC_CHIBIOS Vector98
#define NVIC_CAN1_SCE_IRQ_VEC_OPENCM3 can1_sce_isr
#define NVIC_EXTI9_5_IRQ_VEC_CHIBIOS Vector9C
#define NVIC_EXTI9_5_IRQ_VEC_OPENCM3 exti9_5_isr
#define NVIC_TIM1_BRK_TIM9_IRQ_VEC_CHIBIOS VectorA0
#define NVIC_TIM1_BRK_TIM9_IRQ_VEC_OPENCM3 tim1_brk_tim9_isr
#define NVIC_TIM1_UP_TIM10_IRQ_VEC_CHIBIOS VectorA4
#define NVIC_TIM1_UP_TIM10_IRQ_VEC_OPENCM3 tim1_up_tim10_isr
#define NVIC_TIM1_TRG_COM_TIM11_IRQ_VEC_CHIBIOS VectorA8
#define NVIC_TIM1_TRG_COM_TIM11_IRQ_VEC_OPENCM3 tim1_trg_com_tim11_isr
#define NVIC_TIM1_CC_IRQ_VEC_CHIBIOS VectorAC
#define NVIC_TIM1_CC_IRQ_VEC_OPENCM3 tim1_cc_isr
#define NVIC_TIM2_IRQ_VEC_CHIBIOS VectorB0
#define NVIC_TIM2_IRQ_VEC_OPENCM3 tim2_isr
#define NVIC_TIM3_IRQ_VEC_CHIBIOS VectorB4
#define NVIC_TIM3_IRQ_VEC_OPENCM3 tim3_isr
#define NVIC_TIM4_IRQ_VEC_CHIBIOS VectorB8
#define NVIC_TIM4_IRQ_VEC_OPENCM3 tim4_isr
#define NVIC_I2C1_EV_IRQ_VEC_CHIBIOS VectorBC
#define NVIC_I2C1_EV_IRQ_VEC_OPENCM3 i2c1_ev_isr
#define NVIC_I2C1_ER_IRQ_VEC_CHIBIOS VectorC0
#define NVIC_I2C1_ER_IRQ_VEC_OPENCM3 i2c1_er_isr
#define NVIC_I2C2_EV_IRQ_VEC_CHIBIOS VectorC4
#define NVIC_I2C2_EV_IRQ_VEC_OPENCM3 i2c2_ev_isr
#define NVIC_I2C2_ER_IRQ_VEC_CHIBIOS VectorC8
#define NVIC_I2C2_ER_IRQ_VEC_OPENCM3 i2c2_er_isr
#define NVIC_SPI1_IRQ_VEC_CHIBIOS VectorCC
#define NVIC_SPI1_IRQ_VEC_OPENCM3 spi1_isr
#define NVIC_SPI2_IRQ_VEC_CHIBIOS VectorD0
#define NVIC_SPI2_IRQ_VEC_OPENCM3 spi2_isr
#define NVIC_USART1_IRQ_VEC_CHIBIOS VectorD4
#define NVIC_USART1_IRQ_VEC_OPENCM3 usart1_isr
#define NVIC_USART2_IRQ_VEC_CHIBIOS VectorD8
#define NVIC_USART2_IRQ_VEC_OPENCM3 usart2_isr
#define NVIC_USART3_IRQ_VEC_CHIBIOS VectorDC
#define NVIC_USART3_IRQ_VEC_OPENCM3 usart3_isr
#define NVIC_EXTI15_10_IRQ_VEC_CHIBIOS VectorE0
#define NVIC_EXTI15_10_IRQ_VEC_OPENCM3 exti15_10_isr
#define NVIC_RTC_ALARM_IRQ_VEC_CHIBIOS VectorE4
#define NVIC_RTC_ALARM_IRQ_VEC_OPENCM3 rtc_alarm_isr
#define NVIC_USB_FS_WKUP_IRQ_VEC_CHIBIOS VectorE8
#define NVIC_USB_FS_WKUP_IRQ_VEC_OPENCM3 usb_fs_wkup_isr
#define NVIC_TIM8_BRK_TIM12_IRQ_VEC_CHIBIOS VectorEC
#define NVIC_TIM8_BRK_TIM12_IRQ_VEC_OPENCM3 tim8_brk_tim12_isr
#define NVIC_TIM8_UP_TIM13_IRQ_VEC_CHIBIOS VectorF0
#define NVIC_TIM8_UP_TIM13_IRQ_VEC_OPENCM3 tim8_up_tim13_isr
#define NVIC_TIM8_TRG_COM_TIM14_IRQ_VEC_CHIBIOS VectorF4
#define NVIC_TIM8_TRG_COM_TIM14_IRQ_VEC_OPENCM3 tim8_trg_com_tim14_isr
#define NVIC_TIM8_CC_IRQ_VEC_CHIBIOS VectorF8
#define NVIC_TIM8_CC_IRQ_VEC_OPENCM3 tim8_cc_isr
#define NVIC_DMA1_STREAM7_IRQ_VEC_CHIBIOS VectorFC
#define NVIC_DMA1_STREAM7_IRQ_VEC_OPENCM3 dma1_stream7_isr
#define NVIC_FSMC_IRQ_VEC_CHIBIOS Vector100
#define NVIC_FSMC_IRQ_VEC_OPENCM3 fsmc_isr
#define NVIC_SDIO_IRQ_VEC_CHIBIOS Vector104
#define NVIC_SDIO_IRQ_VEC_OPENCM3 sdio_isr
#define NVIC_TIM5_IRQ_VEC_CHIBIOS Vector108
#define NVIC_TIM5_IRQ_VEC_OPENCM3 tim5_isr
#define NVIC_SPI3_IRQ_VEC_CHIBIOS Vector10C
#define NVIC_SPI3_IRQ_VEC_OPENCM3 spi3_isr
#define NVIC_UART4_IRQ_VEC_CHIBIOS Vector110
#define NVIC_UART4_IRQ_VEC_OPENCM3 uart4_isr
#define NVIC_UART5_IRQ_VEC_CHIBIOS Vector114
#define NVIC_UART5_IRQ_VEC_OPENCM3 uart5_isr
#define NVIC_TIM6_DAC_IRQ_VEC_CHIBIOS Vector118
#define NVIC_TIM6_DAC_IRQ_VEC_OPENCM3 tim6_dac_isr
#define NVIC_TIM7_IRQ_VEC_CHIBIOS Vector11C
#define NVIC_TIM7_IRQ_VEC_OPENCM3 tim7_isr
#define NVIC_DMA2_STREAM0_IRQ_VEC_CHIBIOS Vector120
#define NVIC_DMA2_STREAM0_IRQ_VEC_OPENCM3 dma2_stream0_isr
#define NVIC_DMA2_STREAM1_IRQ_VEC_CHIBIOS Vector124
#define NVIC_DMA2_STREAM1_IRQ_VEC_OPENCM3 dma2_stream1_isr
#define NVIC_DMA2_STREAM2_IRQ_VEC_CHIBIOS Vector128
#define NVIC_DMA2_STREAM2_IRQ_VEC_OPENCM3 dma2_stream2_isr
#define NVIC_DMA2_STREAM3_IRQ_VEC_CHIBIOS Vector12C
#define NVIC_DMA2_STREAM3_IRQ_VEC_OPENCM3 dma2_stream3_isr
#define NVIC_DMA2_STREAM4_IRQ_VEC_CHIBIOS Vector130
#define NVIC_DMA2_STREAM4_IRQ_VEC_OPENCM3 dma2_stream4_isr
#define NVIC_ETH_IRQ_VEC_CHIBIOS Vector134
#define NVIC_ETH_IRQ_VEC_OPENCM3 eth_isr
#define NVIC_ETH_WKUP_IRQ_VEC_CHIBIOS Vector138
#define NVIC_ETH_WKUP_IRQ_VEC_OPENCM3 eth_wkup_isr
#define NVIC_CAN2_TX_IRQ_VEC_CHIBIOS Vector13C
#define NVIC_CAN2_TX_IRQ_VEC_OPENCM3 can2_tx_isr
#define NVIC_CAN2_RX0_IRQ_VEC_CHIBIOS Vector140
#define NVIC_CAN2_RX0_IRQ_VEC_OPENCM3 can2_rx0_isr
#define NVIC_CAN2_RX1_IRQ_VEC_CHIBIOS Vector144
#define NVIC_CAN2_RX1_IRQ_VEC_OPENCM3 can2_rx1_isr
#define NVIC_CAN2_SCE_IRQ_VEC_CHIBIOS Vector148
#define NVIC_CAN2_SCE_IRQ_VEC_OPENCM3 can2_sce_isr
#define NVIC_OTG_FS_IRQ_VEC_CHIBIOS Vector14C
#define NVIC_OTG_FS_IRQ_VEC_OPENCM3 otg_fs_isr
#define NVIC_DMA2_STREAM5_IRQ_VEC_CHIBIOS Vector150
#define NVIC_DMA2_STREAM5_IRQ_VEC_OPENCM3 dma2_stream5_isr
#define NVIC_DMA2_STREAM6_IRQ_VEC_CHIBIOS Vector154
#define NVIC_DMA2_STREAM6_IRQ_VEC_OPENCM3 dma2_stream6_isr
#define NVIC_DMA2_STREAM7_IRQ_VEC_CHIBIOS Vector158
#define NVIC_DMA2_STREAM7_IRQ_VEC_OPENCM3 dma2_stream7_isr
#define NVIC_USART6_IRQ_VEC_CHIBIOS Vector15C
#define NVIC_USART6_IRQ_VEC_OPENCM3 usart6_isr
#define NVIC_I2C3_EV_IRQ_VEC_CHIBIOS Vector160
#define NVIC_I2C3_EV_IRQ_VEC_OPENCM3 i2c3_ev_isr
#define NVIC_I2C3_ER_IRQ_VEC_CHIBIOS Vector164
#define NVIC_I2C3_ER_IRQ_VEC_OPENCM3 i2c3_er_isr
#define NVIC_OTG_HS_EP1_OUT_IRQ_VEC_CHIBIOS Vector168
#define NVIC_OTG_HS_EP1_OUT_IRQ_VEC_OPENCM3 otg_hs_ep1_out_isr
#define NVIC_OTG_HS_EP1_IN_IRQ_VEC_CHIBIOS Vector16C
#define NVIC_OTG_HS_EP1_IN_IRQ_VEC_OPENCM3 otg_hs_ep1_in_isr
#define NVIC_OTG_HS_WKUP_IRQ_VEC_CHIBIOS Vector170
#define NVIC_OTG_HS_WKUP_IRQ_VEC_OPENCM3 otg_hs_wkup_isr
#define NVIC_OTG_HS_IRQ_VEC_CHIBIOS Vector174
#define NVIC_OTG_HS_IRQ_VEC_OPENCM3 otg_hs_isr
#define NVIC_DCMI_IRQ_VEC_CHIBIOS Vector178
#define NVIC_DCMI_IRQ_VEC_OPENCM3 dcmi_isr
#define NVIC_CRYP_IRQ_VEC_CHIBIOS Vector17C
#define NVIC_CRYP_IRQ_VEC_OPENCM3 cryp_isr
#define NVIC_HASH_RNG_IRQ_VEC_CHIBIOS Vector180
#define NVIC_HASH_RNG_IRQ_VEC_OPENCM3 hash_rng_isr
