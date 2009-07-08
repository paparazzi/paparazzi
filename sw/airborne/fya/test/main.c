#include "stm32f10x_lib.h"
#include "stm32f10x_map.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "bits.h"

#define STACK_TOP 0x20000800
#define NVIC_CCR ((volatile unsigned long *)(0xE000ED14))


void nmi_handler(void);
void hardfault_handler(void);
int main(void);

// Define the vector table
unsigned int * myvectors[4] 
__attribute__ ((section("vectors")))= {
  (unsigned int *)	0x20000800,	// stack pointer
  (unsigned int *) 	main,		// code entry point
  (unsigned int *)	nmi_handler,		// NMI handler (not really)
  (unsigned int *)	hardfault_handler		// hard fault handler (let's hope not)
};


GPIO_InitTypeDef GPIO_InitStructure;



/*************************************************************************
 * Function Name: main
 * Parameters: none
 * Return: Int32U
 *
 * Description: The main subroutine
 *
 *************************************************************************/
int main(void)
{
	
  *NVIC_CCR = *NVIC_CCR | 0x200; /* Set STKALIGN in NVIC */
  // Init clock system
  Clk_Init();

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
					

  // Configure PC.12 as output push-pull (LED)
  GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);


  while(1)
    {
      GPIOC->BRR |= 0x00001000;
      myDelay(500000);
      GPIOC->BSRR |= 0x00001000;
      myDelay(500000);
    }
}
void nmi_handler(void)
{
  return ;
}

void hardfault_handler(void)
{
  return ;
}
//Functions definitions
void myDelay(unsigned long delay )
{ 
  while(delay) delay--;
}

/*************************************************************************
 * Function Name: Clk_Init
 * Parameters: Int32U Frequency
 * Return: Int32U
 *
 * Description: Init clock system
 *
 *************************************************************************/

void Clk_Init (void)
{
  // 1. Cloking the controller from internal HSI RC (8 MHz)
  RCC_HSICmd(ENABLE);
  // wait until the HSI is ready
  while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
  // 2. Enable ext. high frequency OSC
  RCC_HSEConfig(RCC_HSE_ON);
  // wait until the HSE is ready
  while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);
  // 3. Init PLL
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9); // 72MHz
  //  RCC_PLLConfig(RCC_PLLSource_HSE_Div2,RCC_PLLMul_9); // 72MHz
  RCC_PLLCmd(ENABLE);
  // wait until the PLL is ready
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
  // 4. Set system clock divders
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  RCC_PCLK2Config(RCC_HCLK_Div1);
  RCC_PCLK1Config(RCC_HCLK_Div2);
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  // Flash 1 wait state 
  *(vu32 *)0x40022000 = 0x12;
  // 5. Clock system from PLL
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
}
