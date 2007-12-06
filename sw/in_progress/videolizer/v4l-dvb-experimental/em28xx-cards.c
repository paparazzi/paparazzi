/*
   em28xx-cards.c - driver for Empia EM2800/EM2820/2840/2880 USB video capture devices

   Copyright (C) 2005, 2006 Markus Rechberger <mrechberger@gmail.com>
		 2005       Mauro Carvalho Chehab <mchehab@infradead.org>
		 2005       Sascha Sommer <saschasommer@freenet.de>
		 2005       Ludovico Cavedon <cavedon@sssup.it>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/usb.h>
#include "compat.h"
#include <media/tuner.h>
#include <media/audiochip.h>
#include <media/tveeprom.h>
#include <media/v4l2-common.h>
#include "msp3400-driver.h"
#include <media/saa7115.h>
#include <media/tvp5150.h>

#include "em28xx.h"


static const u8 em28xx_terratec_cinergy_200_usb_i2c_devs[]     = {0x4a,0x60,0x62,0x64,0x86,0xc0,0xc2,0};
static const u8 em28xx_vgear_pockettv_i2c_devs[]               = {0x4a,0x60,0xc6,0};

struct em28xx_board em28xx_boards[] = {
	/* FIXME: please verify the supported video standards */
	[EM2800_BOARD_GENERIC] = {
		.name         = "Generic EM2800 video grabber",
	},
	[EM2820_BOARD_GENERIC] = {
		.name         = "Generic EM2820 video grabber",
	},
	[EM2821_BOARD_GENERIC] = {
		.name         = "Generic EM2821 video grabber",
	},
	[EM2750_BOARD_GENERIC] = {
		.name         = "Generic EM2750 video grabber",
	},
	[EM2860_BOARD_GENERIC] = {
		.name         = "Generic EM2860 video grabber",
	},
	[EM2861_BOARD_GENERIC] = {
		.name         = "Generic EM2861 video grabber",
	},
	[EM2870_BOARD_GENERIC] = {
		.name         = "Generic EM2870 video grabber",
	},
	[EM2881_BOARD_GENERIC] = {
		.name         = "Generic EM2881 video grabber",
	},
	[EM2883_BOARD_GENERIC] = {
		.name         = "Generic EM2883 video grabber",
	},
	[EM2820_BOARD_KWORLD_PVRTV2800RF] = {
		.name         = "Kworld PVR TV 2800 RF",
		.vchannels    = 2,
		.norm         = V4L2_STD_PAL_BG,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes    = EM28XX_VIDEO,
		.input           = {{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2820_BOARD_TERRATEC_CINERGY_250] = {
		.name         = "Terratec Cinergy 250 USB",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_LG_PAL_NEW_TAPC,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes    = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 1,
			.amix     = EM28XX_MIX_VIDEO,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
			.amix     = EM28XX_MIX_LINE_IN,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
			.amix     = EM28XX_MIX_LINE_IN,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2820_BOARD_DLINK_USB_TV] = {
		.name         = "D-Link DUB-T210 TV Tuner",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_LG_PAL_NEW_TAPC,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes    = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2821_BOARD_PROLINK_PLAYTV_USB2] = {
		.name         = "SIIG AVTuner-PVR/Prolink PlayTV USB 2.0",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_LG_PAL_NEW_TAPC, /* unknown? */
		.tda9887_conf = TDA9887_PRESENT, /* unknown? */
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes    = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2820_BOARD_HERCULES_SMART_TV_USB2] = {
		.name         = "Hercules Smart TV USB 2.0",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_LG_PAL_NEW_TAPC,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes    = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2820_BOARD_PINNACLE_USB_2] = {
		.name         = "Pinnacle PCTV USB 2",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_LG_PAL_NEW_TAPC,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes    = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
		}},
	},
	[EM2820_BOARD_PINNACLE_USB_2_FM1216ME] = {
		.name         = "Pinnacle PCTV USB 2 (Philips FM1216ME)",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_PHILIPS_FM1216ME_MK3,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes    = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms        = {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM-DK",
				.id = V4L2_STD_SECAM_DK,
			}},
	},
	[EM2820_BOARD_HAUPPAUGE_WINTV_USB_2_R2] = {
		.name         = "Hauppauge WinTV USB 2 (R2)",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_ABSENT,
		.tda9887_conf = TDA9887_PRESENT|TDA9887_PORT1_ACTIVE|TDA9887_PORT2_ACTIVE,
		.has_tuner    = 1,
		.decoder      = EM28XX_TVP5150,
		.has_msp34xx  = 1,
		.dev_modes    = EM28XX_VIDEO,
		/*FIXME: S-Video not tested */
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 6,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
		}},
	},
	[EM2820_BOARD_HAUPPAUGE_WINTV_USB_2] = {
		.name         = "Hauppauge WinTV USB 2",
		.vchannels    = 3,
		.norm         = V4L2_STD_NTSC,
		.tuner_type   = TUNER_ABSENT,
		.tda9887_conf = TDA9887_PRESENT|TDA9887_PORT1_ACTIVE|TDA9887_PORT2_ACTIVE,
		.has_tuner    = 1,
		.decoder      = EM28XX_TVP5150,
		.has_msp34xx  = 1,
		.dev_modes    = EM28XX_VIDEO,
		/*FIXME: S-Video not tested */
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 6,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		}},
		.tvnorms	= {{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
		}},
	},
	[EM2821_BOARD_SUPERCOMP_USB_2] = {
		.name         = "Supercomp USB 2.0 TV",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_PHILIPS_FM1236_MK3,
		.tda9887_conf = TDA9887_PRESENT|TDA9887_PORT1_ACTIVE|TDA9887_PORT2_ACTIVE,
		.decoder      = EM28XX_SAA7113,
		.dev_modes    = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2880_BOARD_HAUPPAUGE_WINTV_HVR_900] = {
		.name         = "Hauppauge WinTV HVR 900",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_ABSENT, /* XCEIVE_XC3028, */
		.tuner_addr   = 0xc2,
		.has_tuner    = 1,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2883_BOARD_PINNACLE_PCTV_HD_PRO] = {
		.name         = "Pinnacle PCTV HD Pro",
		.has_tuner    = 1,
		.vchannels    = 3,
		.norm         = VIDEO_MODE_NTSC,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.has_tuner    = 1,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_AUDIO | /* TODO: EM28XX_VBI |*/ EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2883_BOARD_KWORLD_HYBRID_A316] = {
		.name         = "Kworld PlusTV HD Hybrid 330",
		.vchannels    = 3,
		.norm         = VIDEO_MODE_NTSC,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.has_tuner    = 1,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_AUDIO | /* TODO: EM28XX_VBI |*/ EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2883_BOARD_HAUPPAUGE_WINTV_HVR_950] = {
		.name         = "Hauppauge WinTV HVR 950",
		.vchannels    = 3,
		.norm         = VIDEO_MODE_NTSC,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.has_tuner    = 1,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_AUDIO | /* TODO: EM28XX_VBI |*/ EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2880_BOARD_HAUPPAUGE_WINTV_HVR_900_R2] = {
		.name         = "Hauppauge WinTV HVR Rev. 1.2",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_ABSENT, /*XCEIVE_XC3028, */
		.tuner_addr   = 0xc2,
		.has_tuner    = 1,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_AUDIO | EM28XX_VBI/* | EM28XX_DVB not ready yet */,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2860_BOARD_TERRATEC_HYBRID_XS] = {
		.name         = "Terratec Cinergy A Hybrid XS",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2880_BOARD_TERRATEC_HYBRID_XS] = {
		.name         = "Terratec Hybrid XS",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2861_BOARD_KWORLD_PVRTV_300U] = {
		.name	      = "KWorld PVRTV 300U",
		.vchannels    = 3,
		.norm         = V4L2_STD_NTSC,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO, /* EM28XX_VBI doesn't work properly */
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2880_BOARD_KWORLD_DVB_310U] = {
		.name	      = "KWorld DVB-T 310U",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2880_BOARD_KWORLD_DVB_305U] = {
		.name	      = "KWorld DVB-T 305U",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2880_BOARD_TERRATEC_HYBRID_XS_FR] = {
		.name         = "Terratec Hybrid XS Secam",
		.vchannels    = 3,
		.norm         = V4L2_STD_SECAM_L,
		.has_tuner    = 1,
		.has_msp34xx  = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2882_BOARD_TERRATEC_HYBRID_XS] = {
		.name         = "Terratec Hybrid XS (em2882)",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB | EM28XX_AUDIO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2881_BOARD_DNT_DA2_HYBRID] = {
		.name         = "DNT DA2 Hybrid",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2870_BOARD_TERRATEC_XS] = {
		.name         = "Terratec Cinergy T XS",
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.dev_modes    = EM28XX_DVB,
	},
	[EM2870_BOARD_TERRATEC_XS_MT2060] = {
		.name         = "Terratec Cinergy T XS (MT2060)",
		.tuner_type   = TUNER_ABSENT, // MT2060,
		.dev_modes    = EM28XX_DVB,
	},
	[EM2870_BOARD_PINNACLE_PCTV_DVB] = {
		.name         = "Pinnacle PCTV DVB-T",
		.tuner_type   = TUNER_ABSENT, // MT2060,
		.dev_modes    = EM28XX_DVB,
	},
	[EM2870_BOARD_KWORLD_350U] = {
		.name         = "Kworld 350 U DVB-T",
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.dev_modes    = EM28XX_DVB,
	},
	[EM2870_BOARD_KWORLD_355U] = {
		.name         = "Kworld 355 U DVB-T",
		.tuner_type   = TUNER_QT1010,
		.dev_modes    = EM28XX_DVB,
	},
	[EM2870_BOARD_COMPRO_VIDEOMATE] = {
		.name         = "Compro, VideoMate U3",
		.has_tuner    = 1,
		.tuner_type   = TUNER_ABSENT, // MT2060
		.dev_modes    = EM28XX_DVB,
	},
	[EM2881_BOARD_PINNACLE_HYBRID_PRO] = {
		.name         = "Pinnacle Hybrid Pro",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2882_BOARD_PINNACLE_HYBRID_PRO] = {
		.name         = "Pinnacle Hybrid Pro (2)",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_AUDIO /*| EM28XX_DVB */,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	/* maybe there's a reason behind it why Terratec sells the Hybrid XS as Prodigy XS with a
	 * different PID, let's keep it separated for now maybe we'll need it lateron */
	[EM2880_BOARD_TERRATEC_PRODIGY_XS] = {
		.name         = "Terratec Prodigy XS",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2880_BOARD_MSI_DIGIVOX_AD] = {
		.name         = "MSI DigiVox A/D",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2882_BOARD_KWORLD_VS_DVBT] = {
		.name         = "Kworld VS-DVB-T 323UR",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2880_BOARD_MSI_DIGIVOX_AD_II] = {
		.name         = "MSI DigiVox A/D II",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 1,
		.tuner_type   = TUNER_XCEIVE_XC3028,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO | EM28XX_VBI | EM28XX_DVB,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			}, {
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
			}, {
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2820_BOARD_MSI_VOX_USB_2] = {
		.name		= "MSI VOX USB 2.0",
		.vchannels	= 3,
		.norm		= V4L2_STD_PAL_BG,
		.tuner_type	= TUNER_LG_PAL_NEW_TAPC,
		.tda9887_conf	= TDA9887_PRESENT|TDA9887_PORT1_ACTIVE|TDA9887_PORT2_ACTIVE,
		.has_tuner	= 1,
		.decoder        = EM28XX_SAA7114,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE4,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
		}},
	},
	[EM2800_BOARD_TERRATEC_CINERGY_200] = {
		.name         = "Terratec Cinergy 200 USB",
		.em_type      = EM2800,
		.i2c_devs     = em28xx_terratec_cinergy_200_usb_i2c_devs,
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_LG_PAL_NEW_TAPC,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2820_BOARD_LEADTEK_WINFAST_USBII_DELUXE] = {
		.name         = "Leadtek Winfast USB II Deluxe",
		.em_type      = EM2820,
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_PHILIPS_FM1216ME_MK3,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7114,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = 2,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = 0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = 9,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2800_BOARD_LEADTEK_WINFAST_USBII] = {
		.name         = "Leadtek Winfast USB II",
		.em_type      = EM2800,
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_LG_PAL_NEW_TAPC,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2800_BOARD_KWORLD_USB2800] = {
		.name         = "Kworld USB2800",
		.em_type      = EM2800,
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_PHILIPS_ATSC,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			}
		},
	},
	[EM2820_BOARD_PINNACLE_DVC_90] = {
		.name         = "Pinnacle Dazzle DVC 90",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 0,
		.decoder      = EM28XX_SAA7113,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
		}},
	},
	[EM2820_BOARD_PINNACLE_DVC_100] = {
		.name         = "Pinnacle Dazzle DVC 100",
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.has_tuner    = 0,
		.decoder      = EM28XX_SAA7113,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "SECAM L",
				.id = V4L2_STD_SECAM_L,
			},{
				.name = "SECAM LC",
				.id = V4L2_STD_SECAM_LC,
			},{
				.name = "SECAM K1",
				.id = V4L2_STD_SECAM_K1,
		}},
	},
	[EM2820_BOARD_VIDEOLOGY_20K14XUSB] = {
		.name          = "Videology 20K14XUSB USB2.0",
		.vchannels     = 1,
		.norm          = V4L2_STD_PAL_BG,
		.has_tuner     = 0,
		.ctrl          = em28xx_vy_cctrl,
		.gctrl         = em28xx_vy_gctrl,
		.qctrl         = em28xx_vy_qctrl,
		.dev_modes      = EM28XX_VIDEO,
		.input         = {{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = 0,
			.amux     = 0,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
		}},
	},
	[EM2821_BOARD_USBGEAR_VD204] = {
		.name          = "Usbgear VD204v9",
		.vchannels     = 2,
		.norm          = V4L2_STD_PAL_BG,
		.decoder       = EM28XX_SAA7113,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type  = EM28XX_VMUX_COMPOSITE1,
			.vmux  = SAA7115_COMPOSITE0,
			.amux  = 1,
		},{
			.type  = EM28XX_VMUX_SVIDEO,
			.vmux  = SAA7115_SVIDEO3,
			.amux  = 1,
		}},
		.tvnorms	= {{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2860_BOARD_TYPHOON_DVD_MAKER] = {
		.name          = "Typhoon DVD Maker",
		.vchannels     = 2,
		.norm          = V4L2_STD_PAL_BG,
		.decoder       = EM28XX_SAA7113,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type  = EM28XX_VMUX_COMPOSITE1,
			.vmux  = SAA7115_COMPOSITE0,
			.amux  = 1,
		},{
			.type  = EM28XX_VMUX_SVIDEO,
			.vmux  = SAA7115_SVIDEO3,
			.amux  = 1,
		}},
		.tvnorms	= {{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2860_BOARD_GADMEI_UTV330] = {
		.name         = "Gadmei UTV330",
		.vchannels    = 3,
		.norm         = V4L2_STD_NTSC,
		.tuner_type   = TUNER_TNF_5335MF,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes    = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
		},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
		}},
	},
	[EM2820_BOARD_GADMEI_UTV310] = {
		.name         = "Gadmei UTV310",
		.vchannels    = 3,
		.norm         = V4L2_STD_NTSC,
		.tuner_type   = TUNER_TNF_5335MF,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes    = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
		}},
	},
	[EM2800_BOARD_VGEAR_POCKETTV] = {
		.name         = "V-Gear PocketTV",
		.em_type      = EM2800,
		.i2c_devs     = em28xx_vgear_pockettv_i2c_devs,
		.vchannels    = 3,
		.norm         = V4L2_STD_PAL_BG,
		.tuner_type   = TUNER_LG_PAL_NEW_TAPC,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_SAA7113,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE2,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = SAA7115_SVIDEO3,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "PAL-DK",
				.id = V4L2_STD_PAL_DK,
			},{
				.name = "PAL-I",
				.id = V4L2_STD_PAL_I,
			},{
				.name = "PAL-M",
				.id = V4L2_STD_PAL_M,
		}},
	},
	[EM2861_BOARD_YAKUMO_MOVIE_MIXER] = {
		.name          = "Yakumo MovieMixer",
		.vchannels     = 1,
		.norm          = V4L2_STD_PAL_BG,
		.decoder       = EM28XX_TVP5150,
		.has_tuner     = 0,
		.dev_modes      = EM28XX_VIDEO,
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 0,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
		}},
	},
	[EM2860_BOARD_NETGMBH_CAM] = {
		.name          = "NetGMBH Cam", /* Beijing Huaqi Information Digital Technology Co., Ltd */
		.vchannels     = 1,
		.norm	       = V4L2_STD_PAL_BG,
		.has_tuner     = 0,
#if 1
		.dev_modes      = EM28XX_VIDEO,
#else
		.dev_modes	= EM28XX_RAWMODE,
#endif

		.input         = {{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = 0,
			.amux     = 0,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
		}},
	},
	[EM2750_BOARD_DLCW_130] = {
		.name          = "Huaqi DLCW-130", /* Beijing Huaqi Information Digital Technology Co., Ltd */
		.vchannels     = 1,
		.norm	       = V4L2_STD_PAL_BG,
		.dev_modes      = EM28XX_VIDEO,
		.em_type	= EM2751,
		.input         = {{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = 0,
			.amux     = 0,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
		}},
	},
	[EM2751_BOARD_EMPIA_SAMPLE] = {
		.name          = "EM2751 Webcam + Audio",
		.vchannels     = 1,
		.norm	       = V4L2_STD_PAL_BG,
		.dev_modes      = EM28XX_VIDEO,
		.em_type	= EM2751,
		.input         = {{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = 0,
			.amux     = 0,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
		}},
	},
	[EM2861_BOARD_PLEXTOR_PX_TV100U] = {
		.name         = "Plextor ConvertX PX-TV100U",
		.vchannels    = 3,
		.norm         = V4L2_STD_NTSC,
		.tuner_type   = TUNER_TNF_5335MF,
		.tda9887_conf = TDA9887_PRESENT,
		.has_tuner    = 1,
		.decoder      = EM28XX_TVP5150,
		.dev_modes    = EM28XX_VIDEO,  // EM28XX_AUDIO ?
		.input          = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = TVP5150_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = TVP5150_COMPOSITE1,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_SVIDEO,
			.vmux     = TVP5150_SVIDEO1,
			.amux     = 1,
		}},
		.tvnorms	= {{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
		}},
	},
	[EM2861_BOARD_POLLIN_USB_R1] = {
		.name          = "Pollin USB-R1",
		.vchannels     = 2,
		.norm          = V4L2_STD_PAL_BG,
		.decoder       = EM28XX_SAA7113,
		.has_tuner     = 0,
		.dev_modes      = EM28XX_VIDEO,
		.input           = {{
			.type     = EM28XX_VMUX_TELEVISION,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		},{
			.type     = EM28XX_VMUX_COMPOSITE1,
			.vmux     = SAA7115_COMPOSITE0,
			.amux     = 1,
		}},
		.tvnorms	= {
			{
				.name = "PAL-BG",
				.id = V4L2_STD_PAL_BG,
			},{
				.name = "NTSC",
				.id = V4L2_STD_NTSC,
		}},
	},
	
};

const unsigned int em28xx_bcount = ARRAY_SIZE(em28xx_boards);

/*
 * seems like it's possible to flash the eeprom, somehow one of my HVR 900 devices suddenly
 * had the content of a WinTV USB 2 eeprom and thus identified itself as a Wintv USB 2
 * device which of course didn't work...
 *
 */

/* table of devices that work with this driver */
struct usb_device_id em28xx_id_table [] = {
	{ USB_DEVICE(0xeb1a, 0x2800), .driver_info = EM2800_BOARD_GENERIC },
	{ USB_DEVICE(0xeb1a, 0x2820), .driver_info = EM2820_BOARD_GENERIC },
	{ USB_DEVICE(0xeb1a, 0x2821), .driver_info = EM2821_BOARD_GENERIC },
	{ USB_DEVICE(0xeb1a, 0x2750), .driver_info = EM2750_BOARD_GENERIC },
	{ USB_DEVICE(0xeb1a, 0x2860), .driver_info = EM2860_BOARD_GENERIC },
	{ USB_DEVICE(0xeb1a, 0x2861), .driver_info = EM2861_BOARD_GENERIC },
	{ USB_DEVICE(0xeb1a, 0x2881), .driver_info = EM2881_BOARD_GENERIC },
	{ USB_DEVICE(0xeb1a, 0x2870), .driver_info = EM2870_BOARD_GENERIC },
	{ USB_DEVICE(0xeb1a, 0xe310), .driver_info = EM2880_BOARD_MSI_DIGIVOX_AD },
	{ USB_DEVICE(0xeb1a, 0xe320), .driver_info = EM2880_BOARD_MSI_DIGIVOX_AD_II },
	{ USB_DEVICE(0xeb1a, 0xe323), .driver_info = EM2882_BOARD_KWORLD_VS_DVBT },
	{ USB_DEVICE(0xeb1a, 0xe300), .driver_info = EM2861_BOARD_KWORLD_PVRTV_300U },
	{ USB_DEVICE(0xeb1a, 0xe350), .driver_info = EM2870_BOARD_KWORLD_350U },
	{ USB_DEVICE(0xeb1a, 0xe355), .driver_info = EM2870_BOARD_KWORLD_355U },
	{ USB_DEVICE(0xeb1a, 0xe357), .driver_info = EM2870_BOARD_KWORLD_355U },
	{ USB_DEVICE(0x0ccd, 0x0036), .driver_info = EM2820_BOARD_TERRATEC_CINERGY_250 },
	{ USB_DEVICE(0x2304, 0x0208), .driver_info = EM2820_BOARD_PINNACLE_USB_2 },
	{ USB_DEVICE(0x2040, 0x4200), .driver_info = EM2820_BOARD_HAUPPAUGE_WINTV_USB_2 },
	{ USB_DEVICE(0x2040, 0x4201), .driver_info = EM2820_BOARD_HAUPPAUGE_WINTV_USB_2_R2 },
	{ USB_DEVICE(0x2304, 0x0207), .driver_info = EM2820_BOARD_PINNACLE_DVC_90 },
	{ USB_DEVICE(0x2304, 0x021a), .driver_info = EM2820_BOARD_PINNACLE_DVC_100 },
	{ USB_DEVICE(0x2040, 0x6500), .driver_info = EM2880_BOARD_HAUPPAUGE_WINTV_HVR_900 },
	{ USB_DEVICE(0x2040, 0x6502), .driver_info = EM2880_BOARD_HAUPPAUGE_WINTV_HVR_900_R2 },
	{ USB_DEVICE(0x0ccd, 0x0042), .driver_info = EM2880_BOARD_TERRATEC_HYBRID_XS },
	{ USB_DEVICE(0x0ccd, 0x004f), .driver_info = EM2860_BOARD_TERRATEC_HYBRID_XS },
	{ USB_DEVICE(0x0ccd, 0x004c), .driver_info = EM2880_BOARD_TERRATEC_HYBRID_XS_FR },
	{ USB_DEVICE(0x0ccd, 0x005e), .driver_info = EM2882_BOARD_TERRATEC_HYBRID_XS },
	{ USB_DEVICE(0x0ccd, 0x0043), .driver_info = EM2870_BOARD_TERRATEC_XS },
	{ USB_DEVICE(0x0ccd, 0x0047), .driver_info = EM2880_BOARD_TERRATEC_PRODIGY_XS },
	{ USB_DEVICE(0x185b, 0x2870), .driver_info = EM2870_BOARD_COMPRO_VIDEOMATE },
	{ USB_DEVICE(0x0413, 0x6023), .driver_info = EM2800_BOARD_LEADTEK_WINFAST_USBII },
	{ USB_DEVICE(0x2001, 0xf112), .driver_info = EM2820_BOARD_DLINK_USB_TV },
	{ USB_DEVICE(0xeb1a, 0x2883), .driver_info = EM2883_BOARD_GENERIC },
	{ USB_DEVICE(0x2040, 0x6513), .driver_info = EM2883_BOARD_HAUPPAUGE_WINTV_HVR_950 },
	{ USB_DEVICE(0x2304, 0x0227), .driver_info = EM2883_BOARD_PINNACLE_PCTV_HD_PRO },
	{ USB_DEVICE(0x2304, 0x0226), .driver_info = EM2882_BOARD_PINNACLE_HYBRID_PRO },
	{ USB_DEVICE(0xeb1a, 0x2751), .driver_info = EM2751_BOARD_EMPIA_SAMPLE },
	{ USB_DEVICE(0xeb1a, 0xe305), .driver_info = EM2880_BOARD_KWORLD_DVB_305U },
	{ USB_DEVICE(0xeb1a, 0xa316), .driver_info = EM2883_BOARD_KWORLD_HYBRID_A316 },
#if 0
	EM2880_BOARD_KWORLD_DVB_310U },
#endif
	{ USB_DEVICE(0x093b, 0xa005), .driver_info = EM2861_BOARD_PLEXTOR_PX_TV100U },
	{ },
};

/* TODO: make int function and check if GPIO writes are ok */
#if 0
	/* just for testing.. because a windriver set my HVR to 0x4200 */
	{ US B_D EVI CE(0x2040, 0x42 00), .drive r_info = EM2 880_BOARD_HAUPPAUGE_WINTV_HVR_900 },
#endif

void em28xx_card_setup(struct em28xx *dev)
{
	switch(dev->model){
		case EM2750_BOARD_DLCW_130:
		case EM2751_BOARD_EMPIA_SAMPLE:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x0a", 1);
				break;
			}
		case EM2820_BOARD_HAUPPAUGE_WINTV_USB_2:
		case EM2820_BOARD_HAUPPAUGE_WINTV_USB_2_R2:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);

#if 0
				struct tveeprom tv;
#endif
#ifdef CONFIG_MODULES
				request_module("tveeprom");
				request_module("ir-kbd-i2c");
				request_module("msp3400");
#endif
				/* Call first TVeeprom */

#if 0
				dev->i2c_client.addr = 0xa0 >> 1;
				tveeprom_hauppauge_analog(&dev->i2c_client, &tv, dev->eedata);

				dev->tuner_type= tv.tuner_type;
				if (tv.audio_processor == AUDIO_CHIP_MSP34XX) {
					dev->i2s_speed=2048000;
					dev->has_msp34xx=1;
				} else
					dev->has_msp34xx=0;

				if (dev->has_msp34xx) {
					/* Send a reset to other chips via gpio */
					em28xx_write_regs_req(dev, 0x00, 0x08, "\xf7", 1);
					udelay(2500);
					em28xx_write_regs_req(dev, 0x00, 0x08, "\xff", 1);
					udelay(2500);
				}
#endif

				break;
			}
		case EM2820_BOARD_KWORLD_PVRTV2800RF:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				em28xx_write_regs_req(dev,0x00,0x08, "\xf9", 1); // GPIO enables sound on KWORLD PVR TV 2800RF
				break;
			}
		case EM2880_BOARD_HAUPPAUGE_WINTV_HVR_900:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x4c", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\x6d", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\x7d", 1);
				msleep(10);
#ifdef CONFIG_MODULES
				request_module("tveeprom");
#endif
				break;
			}

		case EM2820_BOARD_MSI_VOX_USB_2:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				/* enables audio for that device */
				em28xx_write_regs_req(dev, 0x00, 0x08, "\xfd", 1);
				break;
			}
		case EM2880_BOARD_TERRATEC_HYBRID_XS_FR:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				em28xx_write_regs(dev, 0x08, "\xfd", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\xff", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\xfe", 1);
				msleep(10);

				em28xx_write_regs(dev, 0x04, "\x00", 1);
				msleep(100);
				em28xx_write_regs(dev, 0x04, "\x08", 1);
				msleep(100);
				em28xx_write_regs(dev, 0x08, "\xfe", 1);
				msleep(100);
				break;
			}

		case EM2880_BOARD_TERRATEC_HYBRID_XS:
		case EM2860_BOARD_TERRATEC_HYBRID_XS:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x97", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\x6d", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\x7d", 1);
				msleep(10);
				em2880_ir_attach(dev,ir_codes_em_terratec2,ARRAY_SIZE(ir_codes_em_terratec2), em2880_get_key_terratec);
				break;
			}
		case EM2882_BOARD_TERRATEC_HYBRID_XS:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x07", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x4c", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\x6d", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\x7d", 1);
				msleep(10);
				em2880_ir_attach(dev,ir_codes_em_terratec2,ARRAY_SIZE(ir_codes_em_terratec2), em2880_get_key_terratec);
				break;
			}
		case EM2881_BOARD_DNT_DA2_HYBRID:
		case EM2861_BOARD_KWORLD_PVRTV_300U:
		case EM2880_BOARD_KWORLD_DVB_310U:
		case EM2880_BOARD_KWORLD_DVB_305U:
		case EM2880_BOARD_MSI_DIGIVOX_AD:
		case EM2880_BOARD_MSI_DIGIVOX_AD_II:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x4c", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\x6d", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\x7d", 1);
				msleep(10);
				break;
			}
/*		case EM2870_BOARD_COMPRO_VIDEOMATE: TODO */
		case EM2870_BOARD_PINNACLE_PCTV_DVB:
			{
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				/* this device needs some gpio writes to get the DVB-T demod work */
				em28xx_write_regs(dev, 0x08, "\xfe", 1);
				mdelay(70);
				em28xx_write_regs(dev, 0x08, "\xde", 1);
				mdelay(70);
				em28xx_write_regs(dev, 0x08, "\xfe", 1);
				mdelay(70);
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x22", 1); /* switch em2880 rc protocol */
				em2880_ir_attach(dev,ir_codes_pinnacle2,ARRAY_SIZE(ir_codes_pinnacle2), em2880_get_key_pinnacle);
				break;
			}
		case EM2870_BOARD_TERRATEC_XS_MT2060:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				/* this device needs some gpio writes to get the DVB-T demod work */
				em28xx_write_regs(dev, 0x08, "\xfe", 1);
				mdelay(70);
				em28xx_write_regs(dev, 0x08, "\xde", 1);
				mdelay(70);
				dev->em28xx_write_regs(dev, 0x08, "\xfe", 1);
				mdelay(70);
				break;
			}
		case EM2870_BOARD_KWORLD_350U:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				em28xx_write_regs(dev, 0x08, "\xdf", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\xde", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\xfe", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\x6e", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\x7e", 1);
				msleep(10);
				break;
			}
		case EM2870_BOARD_KWORLD_355U:
		case EM2870_BOARD_COMPRO_VIDEOMATE:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				/* TODO: someone can do some cleanup here... not everything's needed */
				em28xx_write_regs(dev, 0x04, "\x00", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x04, "\x01", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\xfd", 1);
				mdelay(70);
				em28xx_write_regs(dev, 0x08, "\xfc", 1);
				mdelay(70);
				em28xx_write_regs(dev, 0x08, "\xdc", 1);
				mdelay(70);
				em28xx_write_regs(dev, 0x08, "\xfc", 1);
				mdelay(70);
				break;
			}

		case EM2883_BOARD_PINNACLE_PCTV_HD_PRO:
		case EM2882_BOARD_PINNACLE_HYBRID_PRO:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				em28xx_write_regs(dev, 0x08, "\xff", 1);
				em28xx_write_regs(dev, 0x04, "\x00", 1);
				msleep(100);
				em28xx_write_regs(dev, 0x04, "\x08", 1);
				msleep(100);
				em28xx_write_regs(dev, 0x08, "\xff", 1);
				msleep(50);
				em28xx_write_regs(dev, 0x08, "\x2d", 1);
				msleep(50);
				em28xx_write_regs(dev, 0x08, "\x3d", 1);

				em28xx_write_regs(dev, 0x0f,"\xa7", 1); /* enable audio 12 mhz i2s*/
				msleep(10);
				em2880_ir_attach(dev,ir_codes_em_pinnacle2_usb,ARRAY_SIZE(ir_codes_em_pinnacle2_usb), em2880_get_key_pinnacle);
				break;
			}
		case EM2880_BOARD_HAUPPAUGE_WINTV_HVR_900_R2:
		case EM2883_BOARD_HAUPPAUGE_WINTV_HVR_950:
		case EM2883_BOARD_KWORLD_HYBRID_A316:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				em28xx_write_regs(dev, 0x08, "\xff", 1);
				em28xx_write_regs(dev, 0x04, "\x00", 1);
				msleep(100);
				em28xx_write_regs(dev, 0x04, "\x08", 1);
				msleep(100);
				em28xx_write_regs(dev, 0x08, "\xff", 1);
				msleep(50);
				em28xx_write_regs(dev, 0x08, "\x2d", 1);
				msleep(50);
				em28xx_write_regs(dev, 0x08, "\x3d", 1);

				em28xx_write_regs(dev, 0x0f,"\xa7", 1); /* enable audio 12 mhz i2s*/
				msleep(10);
				break;
			}
		case EM2881_BOARD_PINNACLE_HYBRID_PRO:
			{
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x4c", 1);
				msleep(10);
				em28xx_write_regs(dev, 0x08, "\xfd", 1);
				msleep(100);
				em28xx_write_regs(dev, 0x08, "\xfd", 1);
				msleep(100);
				em28xx_write_regs(dev, 0x08, "\xff", 1);
				msleep(5);
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1); /* switch em2880 rc protocol */
				em2880_ir_attach(dev,ir_codes_pinnacle2,ARRAY_SIZE(ir_codes_pinnacle2), em2880_get_key_pinnacle);
#if 0

				dev->em28xx_write_regs(dev, 0x08, "\x6d", 1);
				msleep(10);
				dev->em28xx_write_regs(dev, 0x08, "\x7d", 1);
				msleep(10);
#endif
#if 0
				em2880_ir_attach(dev,ir_codes_pinnacle2,ARRAY_SIZE(ir_codes_pinnacle2), em2880_get_key_pinnacle);
#endif
				break;
			}
		case EM2820_BOARD_GADMEI_UTV310:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				/* Turn on analog audio output */
				em28xx_write_regs_req(dev,0x00,0x08, "\xfd", 1);
				break;
			}
		case EM2860_BOARD_GADMEI_UTV330:
			{
				/* Turn on IR */
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x07", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				em2880_ir_attach(dev, ir_codes_em_gadmei_usb, ARRAY_SIZE(ir_codes_em_gadmei_usb), em2880_get_key_pinnacle);
				break;
			}
		case EM2861_BOARD_PLEXTOR_PX_TV100U:
			{
				em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);
				//FIXME guess
				/* Turn on analog audio output */
				em28xx_write_regs_req(dev,0x00,0x08, "\xfd", 1);
				break;
			}
		default:
				dev->em28xx_write_regs(dev, R0F_XCLK_REG, "\x27", 1);
				if(dev->em_type != EM2800)
					dev->em28xx_write_regs(dev, R06_I2C_CLK_REG, "\x40", 1);


	}
}

void em28xx_card_disconnect(struct em28xx *dev)
{
	switch(dev->model){
		case EM2880_BOARD_TERRATEC_HYBRID_XS:
#if 0
		case EM2880_BOARD_TERRATEC_HYBRID_XS_FR:
#endif
		case EM2860_BOARD_TERRATEC_HYBRID_XS:
		case EM2882_BOARD_TERRATEC_HYBRID_XS:
		case EM2881_BOARD_PINNACLE_HYBRID_PRO:
		case EM2883_BOARD_PINNACLE_PCTV_HD_PRO:
		case EM2882_BOARD_PINNACLE_HYBRID_PRO:
		case EM2870_BOARD_PINNACLE_PCTV_DVB:
		case EM2860_BOARD_GADMEI_UTV330:
			{
				em2880_ir_detach(dev);
				break;
			}
	}
}

EXPORT_SYMBOL(em28xx_boards);
EXPORT_SYMBOL(em28xx_bcount);
EXPORT_SYMBOL(em28xx_id_table);

MODULE_DEVICE_TABLE (usb, em28xx_id_table);
