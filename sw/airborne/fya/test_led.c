
#include <stm32/flash.h>
#include <stm32/misc.h>

#include CONFIG
#include "init_hw.h"
#include "led.h"

void Delay(__IO uint32_t nCount);

int main(void) {

  hw_init();
  //  led_init(); // handled by PERIPHERALS_AUTO_INIT
  while (1) {
    LED_ON(1);
    Delay(500000);
    LED_OFF(1);
    Delay(500000);

  };
  return 0;
}

void Delay(__IO uint32_t nCount) {
  for(; nCount != 0; nCount--);
}


