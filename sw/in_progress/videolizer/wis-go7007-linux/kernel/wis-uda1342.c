/*
 * Copyright (C) 2005-2006 Micronas USA Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <media/audiochip.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
#include <media/v4l2-common.h>
#endif

#include "wis-i2c.h"

static int write_reg(struct i2c_client *client, int reg, int value)
{
	/* UDA1342 wants MSB first, but SMBus sends LSB first */
	i2c_smbus_write_word_data(client, reg, swab16(value));
	return 0;
}

static int wis_uda1342_command(struct i2c_client *client,
				unsigned int cmd, void *arg)
{
	switch (cmd) {
	case AUDC_SET_INPUT:
	{
		int *inp = arg;

		switch (*inp) {
		case AUDIO_TUNER:
			write_reg(client, 0x00, 0x1441); /* select input 2 */
			break;
		case AUDIO_EXTERN:
			write_reg(client, 0x00, 0x1241); /* select input 1 */
			break;
		default:
			printk(KERN_ERR "wis-uda1342: input %d not supported\n",
					*inp);
			break;
		}
		break;
	}
	default:
		break;
	}
	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12)
static int wis_uda1342_i2c_id = 0;
#endif
static struct i2c_driver wis_uda1342_driver;

static struct i2c_client wis_uda1342_client_templ = {
	.name		= "UDA1342 (WIS)",
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
	.flags		= I2C_CLIENT_ALLOW_USE,
#endif
	.driver		= &wis_uda1342_driver,
};

static int wis_uda1342_detect(struct i2c_adapter *adapter, int addr, int kind)
{
	struct i2c_client *client;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return 0;

	client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if (client == NULL)
		return -ENOMEM;
	memcpy(client, &wis_uda1342_client_templ,
			sizeof(wis_uda1342_client_templ));
	client->adapter = adapter;
	client->addr = addr;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12)
	client->id = wis_uda1342_i2c_id++;
#endif

	printk(KERN_DEBUG
		"wis-uda1342: initializing UDA1342 at address %d on %s\n",
		addr, adapter->name);

	write_reg(client, 0x00, 0x8000); /* reset registers */
	write_reg(client, 0x00, 0x1241); /* select input 1 */

	i2c_attach_client(client);
	return 0;
}

static int wis_uda1342_detach(struct i2c_client *client)
{
	int r;

	r = i2c_detach_client(client);
	if (r < 0)
		return r;

	kfree(client);
	return 0;
}

static struct i2c_driver wis_uda1342_driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
	.owner		= THIS_MODULE,
	.name		= "WIS UDA1342 I2C driver",
#else
	.driver = {
		.name	= "WIS UDA1342 I2C driver",
	},
#endif
	.id		= I2C_DRIVERID_WIS_UDA1342,
	.detach_client	= wis_uda1342_detach,
	.command	= wis_uda1342_command,
};

static int __init wis_uda1342_init(void)
{
	int r;

	r = i2c_add_driver(&wis_uda1342_driver);
	if (r < 0)
		return r;
	return wis_i2c_add_driver(wis_uda1342_driver.id, wis_uda1342_detect);
}

static void __exit wis_uda1342_cleanup(void)
{
	wis_i2c_del_driver(wis_uda1342_detect);
	i2c_del_driver(&wis_uda1342_driver);
}

module_init(wis_uda1342_init);
module_exit(wis_uda1342_cleanup);

MODULE_LICENSE("GPL v2");
