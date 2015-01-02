#warning "LED debug needs porting to libopencm3 (or removal)"

static inline void LED1_ON(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_6 , Bit_SET);
}

static inline void LED1_OFF(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_6 , !Bit_SET);
}

static inline void LED2_ON(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_7 , Bit_SET);
}

static inline void LED2_OFF(void)
{
  GPIO_WriteBit(GPIOB, GPIO_Pin_7 , !Bit_SET);
}

static inline void LED_INIT(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  LED1_OFF();
  LED2_OFF();
}


static inline void LED_ERROR(uint8_t base, uint8_t nr)
{
  LED2_ON();
  for (int i = 0; i < (base + nr); i++) {
    LED1_ON();
    LED1_OFF();
  }
  LED2_OFF();
}

static inline void LED_SHOW_ACTIVE_BITS(I2C_TypeDef *regs)
{
  uint16_t CR1 = regs->CR1;
  uint16_t SR1 = regs->SR1;
  uint16_t SR2 = regs->SR2;
  // Note: reading SR1 and then SR2 will clear ADDR bits

  LED1_ON();

  // 1 Start
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_SB, SR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 2 Addr
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_ADDR, SR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 3 BTF
  if (BIT_X_IS_SET_IN_REG(I2C_SR1_BIT_BTF, SR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 4 ERROR
  if ((SR1 & I2C_SR1_BITS_ERR) != 0x0000) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // Anything?
  if ((SR1 + SR2) != 0x0000) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  LED1_OFF();


  LED1_ON();

  // 1 Start
  if (BIT_X_IS_SET_IN_REG(I2C_CR1_BIT_START, CR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 2 Stop
  if (BIT_X_IS_SET_IN_REG(I2C_CR1_BIT_STOP, CR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 3 Busy
  if (BIT_X_IS_SET_IN_REG(I2C_SR2_BIT_BUSY, SR2)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 4 Tra
  if (BIT_X_IS_SET_IN_REG(I2C_SR2_BIT_TRA, SR2)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 5 Master
  if (BIT_X_IS_SET_IN_REG(I2C_SR2_BIT_MSL, SR2)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();
  LED1_OFF();

  //#define I2C_DEBUG_LED_CONTROL
#ifdef I2C_DEBUG_LED_CONTROL


  LED1_ON();

  // 1 Anything CR?
  if ((CR1) != 0x0000) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 2 PE
  if (BIT_X_IS_SET_IN_REG(I2C_CR1_BIT_PE, CR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  // 3 SWRESET
  if (BIT_X_IS_SET_IN_REG(I2C_CR1_BIT_SWRST, CR1)) {
    LED2_ON();
  } else {
    LED2_OFF();
  }
  LED2_OFF();

  LED1_OFF();
#endif

}
