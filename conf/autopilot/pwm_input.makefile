# Generic PWM input on lpc21xx using TIMER0 ISR
ap.CFLAGS += -DUSE_PWM_INPUT
ap.srcs += $(SRC_ARCH)/pwm_input.c

