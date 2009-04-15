#include "rdyb_gpio.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

// file descriptor for gpio node in sysfs
static int gpio_fd;

// cached value for gpio register
static unsigned char gpio_val = 0;

static const char gpio_path[] = "/sys/class/hwmon/hwmon0/device/gpio1_value";
static const unsigned char gpio_offset = 16;
static const unsigned char gpio_count = 8;

void gpio_init(void)
{
  gpio_fd = open(gpio_path, O_RDWR);
}

static void gpio_reg_commit(void)
{
	int byte_count;
	char buf[8];

	byte_count = sprintf(buf, "%i\n", gpio_val);
	if (byte_count > 0) {
		write(gpio_fd, buf, byte_count);
	}
}

// Sets a gpio pin on or off
void gpio_set(uint8_t pin, uint8_t on)
{
	// ignore invalid pin numbers
	if (pin < gpio_count) {
		if (on) {
			gpio_val |= (gpio_offset << pin);
		} else {
			gpio_val &= ~(gpio_offset << pin);
		}

		gpio_reg_commit();
	}
}

void gpio_toggle( uint8_t pin )
{
	// ignore invalid pin numbers
	if (pin < gpio_count) {
		gpio_val ^= (gpio_offset << pin);

		gpio_reg_commit();
	}
}
