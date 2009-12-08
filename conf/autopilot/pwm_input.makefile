# Generic PWM input on lpc21xx using TIMER0 ISR
ap.CFLAGS += -DUSE_PWM_INPUT -DUSE_PWM_INPUT1 -DUSE_PWM_INPUT2
ap.srcs += $(SRC_ARCH)/pwm_input.c

