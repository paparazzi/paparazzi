#ifndef SC18IS600_ARCH_H
#define SC18IS600_ARCH_H

#define Sc18Is600Unselect() GPIOB->BSRR = GPIO_Pin_12
#define Sc18Is600Select()   GPIOB->BRR = GPIO_Pin_12



#endif /* SC18IS600_ARCH_H */
