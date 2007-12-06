/*
   em28xx.h - driver for Empia EM2800/EM2820/2840/2880 USB video capture devices

   Copyright (C) 2005 Markus Rechberger <mrechberger@gmail.com>
		      Ludovico Cavedon <cavedon@sssup.it>
		      Mauro Carvalho Chehab <mchehab@infradead.org>

   Based on the em2800 driver from Sascha Sommer <saschasommer@freenet.de>

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

#ifndef _EM28XX_H
#define _EM28XX_H

#include "compat.h"
#include <linux/videodev.h>
#include <linux/i2c.h>
#include <linux/dvb/frontend.h>
#include "dmxdev.h"
#include "dvb_demux.h"
#include "dvb_net.h"
#include "dvb_frontend.h"
#include <linux/soundcard.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "xc3028-tuner.h"


#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)
#include <linux/mutex.h>
#endif
#include <media/ir-kbd-i2c.h>

/* Boards supported by driver */

#define EM2800_BOARD_GENERIC			  0
#define EM2820_BOARD_GENERIC			  1
#define EM2821_BOARD_GENERIC                      2
#define EM2870_BOARD_GENERIC                      3
#define EM2881_BOARD_GENERIC                      4
#define EM2860_BOARD_GENERIC 			  5
#define EM2861_BOARD_GENERIC 			  6
#define EM2820_BOARD_TERRATEC_CINERGY_250	  7
#define EM2820_BOARD_PINNACLE_USB_2		  8
#define EM2820_BOARD_HAUPPAUGE_WINTV_USB_2        9
#define EM2820_BOARD_MSI_VOX_USB_2                10
#define EM2800_BOARD_TERRATEC_CINERGY_200         11
#define EM2800_BOARD_LEADTEK_WINFAST_USBII        12
#define EM2800_BOARD_KWORLD_USB2800               13
#define EM2820_BOARD_PINNACLE_DVC_90		  14
#define EM2880_BOARD_HAUPPAUGE_WINTV_HVR_900	  15
#define EM2880_BOARD_TERRATEC_HYBRID_XS		  16
#define EM2880_BOARD_TERRATEC_HYBRID_XS_FR	  17
#define EM2820_BOARD_KWORLD_PVRTV2800RF		  18
#define EM2880_BOARD_TERRATEC_PRODIGY_XS	  19
#define EM2820_BOARD_VIDEOLOGY_20K14XUSB	  20
#define EM2821_BOARD_USBGEAR_VD204                21
#define EM2870_BOARD_TERRATEC_XS                  22
#define EM2870_BOARD_PINNACLE_PCTV_DVB            23
#define EM2881_BOARD_DNT_DA2_HYBRID               24
#define EM2881_BOARD_PINNACLE_HYBRID_PRO          25
#define EM2820_BOARD_HERCULES_SMART_TV_USB2	  26
#define EM2870_BOARD_COMPRO_VIDEOMATE             27
#define EM2880_BOARD_KWORLD_DVB_310U              28
#define EM2821_BOARD_PROLINK_PLAYTV_USB2          29
#define EM2870_BOARD_TERRATEC_XS_MT2060           30
#define EM2880_BOARD_MSI_DIGIVOX_AD               31
#define EM2820_BOARD_DLINK_USB_TV                 32
#define EM2820_BOARD_GADMEI_UTV310                33
#define EM2870_BOARD_KWORLD_355U		  34
#define EM2821_BOARD_SUPERCOMP_USB_2              35
#define EM2880_BOARD_HAUPPAUGE_WINTV_HVR_900_R2	  36
#define EM2860_BOARD_GADMEI_UTV330		  37
#define EM2800_BOARD_VGEAR_POCKETTV               38
#define EM2870_BOARD_KWORLD_350U		  39
#define EM2882_BOARD_TERRATEC_HYBRID_XS		  40
#define EM2820_BOARD_PINNACLE_DVC_100		  41
#define EM2861_BOARD_YAKUMO_MOVIE_MIXER           43
#define EM2750_BOARD_DLCW_130                     44
#define EM2750_BOARD_GENERIC			  42
#define EM2883_BOARD_GENERIC                      45
#define EM2883_BOARD_HAUPPAUGE_WINTV_HVR_950      46
#define EM2883_BOARD_PINNACLE_PCTV_HD_PRO	  47
#define EM2882_BOARD_PINNACLE_HYBRID_PRO          48
#define EM2820_BOARD_HAUPPAUGE_WINTV_USB_2_R2     49
#define EM2860_BOARD_NETGMBH_CAM		  50
#define EM2820_BOARD_LEADTEK_WINFAST_USBII_DELUXE 51
#define EM2880_BOARD_MSI_DIGIVOX_AD_II            52
#define EM2860_BOARD_TYPHOON_DVD_MAKER            53
#define EM2820_BOARD_PINNACLE_USB_2_FM1216ME      54
#define EM2751_BOARD_EMPIA_SAMPLE		  55
#define EM2880_BOARD_KWORLD_DVB_305U              56
#define EM2861_BOARD_KWORLD_PVRTV_300U 		  57
#define EM2883_BOARD_KWORLD_HYBRID_A316 	  58
#define EM2860_BOARD_TERRATEC_HYBRID_XS		  59
#define EM2861_BOARD_PLEXTOR_PX_TV100U		  60
#define EM2882_BOARD_KWORLD_VS_DVBT		  61
#define EM2861_BOARD_POLLIN_USB_R1		  62

#define EM28XX_VBI_LINES 34

#define UNSET -1

/* maximum number of em28xx boards */
/* TODO: we should distinct here between
   * DVB-T
   * analogue only
   * hybrid devices
   1 analogue tuner requires up to 170 mbit
   2 analogue tuners will work in alt 3 mode, but the default
     is alt 7 at the moment (it's possible to override
     with the module parameter alt=3), though it should be
     implemented properly.
     2 devices take up to 340 mbit which should work.
     (3 tuners would be 510 mbit which is too much for
     one usb controller)

   DVB-T tuner only take around 15 mbit, so we should be able
   to support more than the limit below.

   I think a controller based bandwidth table would do here
   and guarantee that the initialized devices will work
*/
#define EM28XX_MAXBOARDS 3 /*FIXME: should be bigger */

/* maximum number of frames that can be queued */
#define EM28XX_NUM_FRAMES 5
/* number of frames that get used for v4l2_read() */
#define EM28XX_NUM_READ_FRAMES 2

/* number of buffers for isoc transfers */
#define EM28XX_NUM_BUFS 5
#define EM2880_DVB_NUM_BUFS 5
#define EM28XX_AUDIO_BUFS 5
#define EM28XX_NUM_AUDIO_PACKETS 64
#define EM28XX_AUDIO_MAX_PACKET_SIZE 196 /* static value */


/* number of packets for each buffer
   windows requests only 40 packets .. so we better do the same
   this is what I found out for all alternate numbers there!
 */
#define EM28XX_NUM_PACKETS 40

/* default alternate; 0 means choose the best */
#define EM28XX_PINOUT 0

#define EM28XX_INTERLACED_DEFAULT 1

/*
#define (use usbview if you want to get the other alternate number infos)
#define
#define alternate number 2
#define 			Endpoint Address: 82
			Direction: in
			Attribute: 1
			Type: Isoc
			Max Packet Size: 1448
			Interval: 125us

  alternate number 7

			Endpoint Address: 82
			Direction: in
			Attribute: 1
			Type: Isoc
			Max Packet Size: 3072
			Interval: 125us
*/

/* time to wait when stopping the isoc transfer */
#define EM28XX_URB_TIMEOUT       msecs_to_jiffies(EM28XX_NUM_BUFS * EM28XX_NUM_PACKETS)

/* time in msecs to wait for i2c writes to finish */
#define EM2800_I2C_WRITE_TIMEOUT 20

/* the various frame states */
enum em28xx_frame_state {
	F_UNUSED = 0,
	F_QUEUED,
	F_GRABBING,
	F_DONE,
	F_ERROR,
};

/* stream states */
enum em28xx_stream_state {
	STREAM_OFF,
	STREAM_ON,
	STREAM_INTERRUPT,
};

/* frames */
struct em28xx_frame_t {
	void *bufmem;
	struct v4l2_buffer buf;
	enum em28xx_frame_state state;
	struct list_head frame;
	unsigned long vma_use_count;
	int top_field;
	int fieldbytesused;
};

/* io methods */
enum em28xx_io_method {
	IO_NONE,
	IO_READ,
	IO_MMAP,
};

/* inputs */

#define MAX_EM28XX_INPUT 4
#define MAX_EM28XX_TVNORMS 10

enum enum28xx_itype {
	EM28XX_VMUX_COMPOSITE1 = 1,
	EM28XX_VMUX_COMPOSITE2,
	EM28XX_VMUX_COMPOSITE3,
	EM28XX_VMUX_COMPOSITE4,
	EM28XX_VMUX_SVIDEO,
	EM28XX_VMUX_TELEVISION,
	EM28XX_VMUX_CABLE,
	EM28XX_VMUX_DVB,
	EM28XX_VMUX_DEBUG,
	EM28XX_RADIO,
};

enum enum28xx_mixchannel {
	EM28XX_MIX_NOTOUCH = 0,
	EM28XX_MIX_LINE_IN = 1,
	EM28XX_MIX_VIDEO = 2,
};

struct em28xx_input {
	enum enum28xx_itype type;
	unsigned int vmux;
	unsigned int amux;
	enum enum28xx_mixchannel amix;
};

#define INPUT(nr) (&em28xx_boards[dev->model].input[nr])

enum em28xx_decoder {
	EM28XX_TVP5150,
	EM28XX_SAA7113,
	EM28XX_SAA7114
};

struct em28xx;

/* 0x0 - undef */
enum empia_type {
	EM2800 = 1,
	EM2820,
	EM2840,
	EM2750,
	EM2751,
	EM2860,
	EM2880
};

#define EM28XX_VIDEO   0x01
#define EM28XX_DVB     0x02
#define EM28XX_VBI     0x04
#define EM28XX_RAWMODE 0x08
#define EM28XX_AUDIO   0x10

/* private ioctl */
#define EM28XX_SWITCH_MODE 0x100

#define EM28XX_CAPTURE_STREAM_EN 1

/* tvnorms */
struct em28xx_tvnorm {
	char *name;
	v4l2_std_id id;
#if 0
	/* mode for saa7113h */
	int mode;
#endif
};

struct em28xx_board {
	char *name;
	int vchannels;
	v4l2_std_id norm;
	int tuner_type;
	int tuner_addr;

	/* i2c flags */
	unsigned int em_type;
	/* null terminated list of used i2c addresses */
	/* used to autodetect some em2800 devices without eeprom */
	const u8* i2c_devs;
	unsigned int tda9887_conf;

	unsigned int has_tuner:1;
	unsigned int has_msp34xx:1;

	unsigned int dev_modes;

	enum em28xx_decoder decoder;

	struct em28xx_tvnorm	tvnorms[MAX_EM28XX_TVNORMS];

	int (*ctrl)(struct em28xx *dev, struct v4l2_control *ctrl);
	int (*gctrl)(struct em28xx *dev, struct v4l2_control *ctrl);
	int (*qctrl)(struct v4l2_queryctrl *qctrl);

	struct em28xx_input       input[MAX_EM28XX_INPUT];
};

struct em28xx_eeprom {
	u32 id;			/* 0x9567eb1a */
	u16 vendor_ID;
	u16 product_ID;

	u16 chip_conf;

	u16 board_conf;

	u16 string1, string2, string3;

	u8 string_idx_table;
};

/* device states */
enum em28xx_dev_state {
	DEV_INITIALIZED = 0x01,
	DEV_DISCONNECTED = 0x02,
	DEV_MISCONFIGURED = 0x04,
};


/* digital main device struct */

enum mtype {
	EM28XX_ZL10353,
	EM28XX_MT352,
	EM28XX_DRX3975D,
	EM28XX_LGDT330X,
};

struct em2880_dvb {
	char *name;
	struct dvb_demux demux;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)
	struct mutex sem;
#else
	struct semaphore sem;
#endif
	int streaming;
	enum mtype mod_type;
	struct dvb_adapter adapter;
	struct dvb_frontend        *frontend;
	struct dvb_device *fedev;
	struct dmxdev dmxdev;
	struct em28xx *em28xx_dev; /* please get rid of it lateron */
	struct dvb_net dvbnet;
	struct usb_device *udev;	/* the usb device */
	char *transfer_buffer[EM2880_DVB_NUM_BUFS];     /* transfer buffers for isoc transfer */
	struct urb *urb[EM2880_DVB_NUM_BUFS];   /* urb for isoc transfers */
	int (*demod_init)(struct dvb_frontend* fe);
	int (*get_status)(struct em2880_dvb *dvb);
	int (*dvb_lock)(struct em2880_dvb *dvb, int lock);

};

struct em28xx_audio {
	char name[50];
	char *transfer_buffer[EM28XX_AUDIO_BUFS];
	struct urb *urb[EM28XX_AUDIO_BUFS];
	struct usb_device *udev;
	unsigned int capture_transfer_done;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16)
	snd_pcm_substream_t        *capture_pcm_substream;
#else
	struct snd_pcm_substream   *capture_pcm_substream;
#endif

	unsigned int hwptr_done_capture;
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,16)
	snd_card_t                 *sndcard;
#else
	struct snd_card            *sndcard;
#endif

	int users;
	unsigned int capture_stream:1;
	spinlock_t slock;
};


struct em2880_ir {
	u8      old;
	u8      sequence[4];
	IR_KEYTAB_TYPE		*keymap;
	struct work_struct 	work;
	struct timer_list 	timer;
	struct input_dev	*input;
	struct em28xx		*dev;
	int                     keypressed;
	u32                     keycode;
	int (*get_key)(struct em28xx *dev, u32 *ir_key, u32 *keystatus);
};

struct em28xx_fh {
	struct em28xx *dev;
	unsigned int reader:1;
	int type;
};

/* main device struct */
struct em28xx {
	/* generic device properties */
	char name[30];		/* name (including minor) of the device */
	int model;		/* index in the device_data struct */
	int devno;		/* marks the number of this device */
	unsigned int em_type;
	int video_inputs;	/* number of video inputs */
	struct list_head	devlist;
	unsigned int has_tuner:1;
	unsigned int has_msp34xx:1;
	unsigned int has_tda9887:1;
	unsigned int has_vbi:1;
	unsigned int device_mode:1; /* EM28XX_VIDEO | EM28XX_DVB  FIXME */
	unsigned int dev_modes;
	enum v4l2_tuner_type mode;

	struct work_struct request_module_wk;

	struct em28xx_board *board;

	u32 i2s_speed;		/* I2S speed for audio digital stream */

	enum em28xx_decoder decoder;

	int tuner_type;		/* type of the tuner */
	int tuner_addr;		/* tuner address */
	int tda9887_conf;
	/* i2c i/o */
	struct i2c_adapter i2c_adap;
	struct i2c_client i2c_client;

	/* dvb */
	struct em2880_dvb *dvb_dev;
	struct em28xx_audio *adev;
	struct em2880_ir *ir_em2880;

	/* video for linux */
	int users;		/* user count for exclusive use */

	int vbi_users;
	int video_users;
	struct video_device *vdev;	/* video for linux device struct */
	struct video_picture vpic;	/* picture settings only used to init saa7113h */
	struct em28xx_tvnorm *tvnorm;	/* selected tv norm */
	int ctl_freq;		/* selected frequency */
	unsigned int ctl_input;	/* selected input */
	unsigned int ctl_ainput;	/* slected audio input */
	enum enum28xx_mixchannel ctl_amix;	/* slected audio mixer channel */
	int mute;
	int volume;
	/* frame properties */
	struct em28xx_frame_t frame[EM28XX_NUM_FRAMES];	/* list of frames */
	struct em28xx_frame_t vbi_frame[EM28XX_NUM_FRAMES];	/* list of frames */
	int num_frames;		/* number of frames currently in use */
	int vbi_num_frames;
	unsigned int frame_count;	/* total number of transfered frames */
	struct em28xx_frame_t *frame_current;	/* the frame that is being filled */
	struct em28xx_frame_t *vbi_frame_current;	/* the frame that is being filled */
	int width;		/* current frame width */
	int height;		/* current frame height */
	int frame_size;		/* current frame size */
	int vbi_frame_size;
	int field_size;		/* current field size */
	int vbi_field_size;
	int bytesperline;
	int vbi_bytesperline;
	int hscale;		/* horizontal scale factor (see datasheet) */
	int vscale;		/* vertical scale factor (see datasheet) */
	int interlaced;		/* 1=interlace fileds, 0=just top fileds */
	int type;

	struct xc3028_config xc3028conf;

	unsigned int reader:1;

	/* new 18 APRIL */

	unsigned int vbi_reader:1;
	unsigned int video_reader:1;

	/* endnew */



	/* states */
	enum em28xx_dev_state state;

	/* new 18 APRIL */
	enum em28xx_stream_state stream;
	enum em28xx_io_method io;

	enum em28xx_io_method video_io;
	enum em28xx_io_method vbi_io;

	/* endnew */


	/* locks */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)
	struct mutex lock, fileop_lock, vbi_fileop_lock;
#else
	struct semaphore lock, fileop_lock, vbi_fileop_lock;
#endif
	spinlock_t queue_lock;
	spinlock_t vbi_queue_lock;

	struct list_head inqueue, outqueue, vbi_inqueue, vbi_outqueue;
	wait_queue_head_t open, wait_frame, wait_vbi_frame, wait_stream;
	struct video_device *vbi_dev;
	int video_bytesread;
	/* new 18 APRIL */
	int vbi_bytesread;
	int vbi_dropbytes;
	/* endnew */

	unsigned char eedata[256];

	/* usb transfer */
	struct usb_device *udev;	/* the usb device */
	int alt;		/* alternate */
	int max_pkt_size;	/* max packet size of isoc transaction */
	int num_alt;		/* Number of alternative settings */
	unsigned int *alt_max_pkt_size;	/* array of wMaxPacketSize */
	struct urb *urb[EM28XX_NUM_BUFS];	/* urb for isoc transfers */
	char *transfer_buffer[EM28XX_NUM_BUFS];	/* transfer buffers for isoc transfer */
	/* helper funcs that call usb_control_msg */

	int (*em28xx_callback)(void *priv, int ptr, int mode);

	int (*em28xx_write_regs) (struct em28xx * dev, u16 reg, char *buf,
				  int len);
	int (*em28xx_read_reg) (struct em28xx * dev, u16 reg);
	int (*em28xx_read_reg_req_len) (struct em28xx * dev, u8 req, u16 reg,
					char *buf, int len);
	int (*em28xx_write_regs_req) (struct em28xx * dev, u8 req, u16 reg,
				      char *buf, int len);
	int (*em28xx_read_reg_req) (struct em28xx * dev, u8 req, u16 reg);

	int (*em28xx_ctrl)(struct em28xx *dev, struct v4l2_control *ctrl);
	int (*em28xx_gctrl)(struct em28xx *dev, struct v4l2_control *ctrl);
	int (*em28xx_qctrl)(struct v4l2_queryctrl *qctrl);
};

struct em28xx_ops {
	struct list_head next;
	char *name;
	int id;
	int (*init)(struct em28xx *);
	int (*fini)(struct em28xx *);
};

/* Provided by em28xx-i2c.c */
int em2800_i2c_check_for_device(struct em28xx *dev, unsigned char addr);
void em28xx_i2c_call_clients(struct em28xx *dev, unsigned int cmd, void *arg);
int em28xx_i2c_register(struct em28xx *dev);
int em28xx_i2c_unregister(struct em28xx *dev);

/* Provided by em28xx-input.c */

void em28xx_set_ir(struct em28xx * dev,struct IR_i2c *ir);

/* Provided by em28xx-core.c */

u32 em28xx_request_buffers(struct em28xx *dev, u32 count, int type);
void em28xx_queue_unusedframes(struct em28xx *dev, int type);
void em28xx_release_buffers(struct em28xx *dev, int type);

int em28xx_read_reg_req_len(struct em28xx *dev, u8 req, u16 reg,
			    char *buf, int len);
int em28xx_read_reg_req(struct em28xx *dev, u8 req, u16 reg);
int em28xx_read_reg(struct em28xx *dev, u16 reg);
int em28xx_write_regs_req(struct em28xx *dev, u8 req, u16 reg, char *buf,
			  int len);
int em28xx_write_regs(struct em28xx *dev, u16 reg, char *buf, int len);
int em28xx_write_reg_bits(struct em28xx *dev, u16 reg, u8 val,
			  u8 bitmask);
int em28xx_write_ac97(struct em28xx *dev, u8 reg, u8 * val);
int em28xx_audio_analog_set(struct em28xx *dev);
int em28xx_colorlevels_set_default(struct em28xx *dev);
int em28xx_capture_start(struct em28xx *dev, int start);
int em28xx_outfmt_set_yuv422(struct em28xx *dev);
int em28xx_accumulator_set(struct em28xx *dev, u8 xmin, u8 xmax, u8 ymin,
			   u8 ymax);
int em28xx_capture_area_set(struct em28xx *dev, u8 hstart, u8 vstart,
			    u16 width, u16 height);
int em28xx_scaler_set(struct em28xx *dev, u16 h, u16 v);
int em28xx_resolution_set(struct em28xx *dev);
#if 0
void em28xx_isocIrq(struct urb *urb, struct pt_regs *regs);
void em28xx_rawisocIrq(struct urb *urb, struct pt_regs *regs);
#endif
int em28xx_init_isoc(struct em28xx *dev);
void em28xx_uninit_isoc(struct em28xx *dev);
int em28xx_set_alternate(struct em28xx *dev);
int em28xx_register_extension(struct em28xx_ops *dev);
void em28xx_unregister_extension(struct em28xx_ops *dev);
int em2880_ir_detach(struct em28xx *dev);
int em2880_ir_attach(struct em28xx *dev,IR_KEYTAB_TYPE *keymap, int kmaplen, int (*get_key)(struct em28xx *dev, u32 *ir_key,u32 *keystatus));
int em2880_get_key_terratec(struct em28xx *dev, u32 *ir_key,u32 *keystatus);
int em2880_get_key_pinnacle(struct em28xx *dev, u32 *ir_key, u32 *keystatus);




/* Provided by em28xx-cards.c */
extern int em2800_variant_detect(struct usb_device* udev,int model);
extern void em28xx_card_setup(struct em28xx *dev);
extern void em28xx_card_disconnect(struct em28xx *dev);
extern struct em28xx_board em28xx_boards[];
extern struct usb_device_id em28xx_id_table[];
extern const unsigned int em28xx_bcount;

/* Videology specific functions */

#define V4L2_VY_WBM                 (V4L2_CID_PRIVATE_BASE+0)  /* white balance mode */
#define V4L2_VY_SHUTTER             (V4L2_CID_PRIVATE_BASE+1)  /* shutter speed */
#define V4L2_VY_MIRROR              (V4L2_CID_PRIVATE_BASE+2)  /* mirror mode */
#define V4L2_VY_GAIN_CONTROL_EN     (V4L2_CID_PRIVATE_BASE+3)  /* gain control on/off */
#define V4L2_VY_GAIN_CONTROL        (V4L2_CID_PRIVATE_BASE+4)  /* gain control values 0x00 - 0x7f */
#define V4L2_VY_EDGE_ENHANCE_EN     (V4L2_CID_PRIVATE_BASE+5)  /* enable edge enhance */
#define V4L2_VY_EDGE_ENHANCE        (V4L2_CID_PRIVATE_BASE+6)  /* edge enhance values 0x00 - 0x1f */
#define V4L2_VY_BLC                 (V4L2_CID_PRIVATE_BASE+7)  /* BLC (???) values 0x00 (off) - 0x40 */
#define V4L2_VY_RGAIN               (V4L2_CID_PRIVATE_BASE+8)  /* RGain value    0x00-0xff */
#define V4L2_VY_BGAIN               (V4L2_CID_PRIVATE_BASE+9)  /* BGain value    0x00-0xff */


int em28xx_vy_cctrl(struct em28xx *dev, struct v4l2_control *ctrl);
int em28xx_vy_gctrl(struct em28xx *dev, struct v4l2_control *ctrl);
int em28xx_vy_qctrl(struct v4l2_queryctrl *qctrl);

/* end Videology specific functions/defines */


/* em28xx registers */
#define R06_I2C_CLK_REG         0x06
#define R0A_CHIPID_REG		0x0a
#define R0C_USBSUSP_REG		0x0c    /* */

#define R0E_AUDIOSRC_REG	0x0e
#define R0F_XCLK_REG  		0x0f

#define R10_VINMODE_REG        	0x10
#define R11_VINCTRL_REG        	0x11
#define R12_VINENABLE_REG      	0x12    /* */

#define R14_GAMMA_REG  		0x14
#define R15_RGAIN_REG  		0x15
#define R16_GGAIN_REG  		0x16
#define R17_BGAIN_REG  		0x17
#define R18_ROFFSET_REG        	0x18
#define R19_GOFFSET_REG        	0x19
#define R1A_BOFFSET_REG        	0x1a

#define R1B_OFLOW_REG  		0x1b
#define R1C_HSTART_REG 		0x1c
#define R1D_VSTART_REG 		0x1d
#define R1E_CWIDTH_REG 		0x1e
#define R1F_CHEIGHT_REG        	0x1f

#define R20_YGAIN_REG  		0x20
#define R21_YOFFSET_REG        	0x21
#define R22_UVGAIN_REG 		0x22
#define R23_UOFFSET_REG        	0x23
#define R24_VOFFSET_REG        	0x24
#define R25_SHARPNESS_REG      	0x25

#define R26_COMPR_REG  		0x26
#define R27_OUTFMT_REG 		0x27

#define R28_XMIN_REG   		0x28
#define R29_XMAX_REG   		0x29
#define R2A_YMIN_REG   		0x2a
#define R2B_YMAX_REG   		0x2b

#define R30_HSCALELOW_REG      	0x30
#define R31_HSCALEHIGH_REG     	0x31
#define R32_VSCALELOW_REG      	0x32
#define R33_VSCALEHIGH_REG     	0x33

#define R40_AC97LSB_REG        	0x40
#define R41_AC97MSB_REG        	0x41
#define R42_AC97ADDR_REG       	0x42
#define R43_AC97BUSY_REG       	0x43

/* em202 registers */
#define R02_MASTER_AC97     	0x02
#define R10_LINE_IN_AC97        0x10
#define R14_VIDEO_AC97 		0x14

/* em2800 registers */
#define EM2800_AUDIOSRC_REG 	0x08

/* register settings */
#define EM2800_AUDIO_SRC_TUNER  0x0d
#define EM2800_AUDIO_SRC_LINE   0x0c
#define EM28XX_AUDIO_SRC_TUNER	0xc0
#define EM28XX_AUDIO_SRC_LINE	0x80

/* printk macros */

#define em28xx_err(fmt, arg...) do {\
	printk(KERN_ERR fmt , ##arg); } while (0)

#define em28xx_errdev(fmt, arg...) do {\
	printk(KERN_ERR "%s: "fmt,\
			dev->name , ##arg); } while (0)

#define em28xx_info(fmt, arg...) do {\
	printk(KERN_INFO "%s: "fmt,\
			dev->name , ##arg); } while (0)
#define em28xx_warn(fmt, arg...) do {\
	printk(KERN_WARNING "%s: "fmt,\
			dev->name , ##arg); } while (0)

inline static int em28xx_audio_source(struct em28xx *dev, int input)
{
	if(dev->em_type == EM2800){
		u8 tmp = EM2800_AUDIO_SRC_TUNER;
		if(input == EM28XX_AUDIO_SRC_LINE)
			tmp = EM2800_AUDIO_SRC_LINE;
		em28xx_write_regs(dev, EM2800_AUDIOSRC_REG, &tmp, 1);
	}
	return em28xx_write_reg_bits(dev, R0E_AUDIOSRC_REG, input, 0xc0);
}

/* FIXME: return something sane here */

inline static int em28xx_audio_usb_mute(struct em28xx *dev, int mute)
{
	switch(dev->em_type){
	case EM2750:
		em28xx_write_regs(dev, R0F_XCLK_REG, "\x0a", 1);
		break;
	default:
		em28xx_write_reg_bits(dev, R0F_XCLK_REG, mute ? 0x00 : 0x80, 0x80);
	}
	return 0;
}

inline static int em28xx_audio_analog_setup(struct em28xx *dev)
{
	/* unmute video mixer with default volume level */
	return em28xx_write_ac97(dev, R14_VIDEO_AC97, "\x08\x08");
}

inline static int em28xx_audio_set_mixer(struct em28xx *dev, enum  enum28xx_mixchannel chan)
{
	int ret = 0;

	if (chan == EM28XX_MIX_NOTOUCH)
		return ret;
	if ((ret = em28xx_write_ac97(dev, R10_LINE_IN_AC97,  chan==EM28XX_MIX_LINE_IN?"\x08\x08":"\x08\x88")))
		return ret;
	if ((ret = em28xx_write_ac97(dev, R14_VIDEO_AC97,  chan==EM28XX_MIX_VIDEO?"\x08\x08":"\x08\x88")))
		return ret;
	return ret;
}

inline static int em28xx_compression_disable(struct em28xx *dev)
{
	/* side effect of disabling scaler and mixer */
	return em28xx_write_regs(dev, R26_COMPR_REG, "\x00", 1);
}

inline static int em28xx_contrast_get(struct em28xx *dev)
{
	return em28xx_read_reg(dev, R20_YGAIN_REG) & 0x1f;
}

inline static int em28xx_brightness_get(struct em28xx *dev)
{
	return em28xx_read_reg(dev, R21_YOFFSET_REG);
}

inline static int em28xx_saturation_get(struct em28xx *dev)
{
	return em28xx_read_reg(dev, R22_UVGAIN_REG) & 0x1f;
}

inline static int em28xx_u_balance_get(struct em28xx *dev)
{
	return em28xx_read_reg(dev, R23_UOFFSET_REG);
}

inline static int em28xx_v_balance_get(struct em28xx *dev)
{
	return em28xx_read_reg(dev, R24_VOFFSET_REG);
}

inline static int em28xx_gamma_get(struct em28xx *dev)
{
	return em28xx_read_reg(dev, R14_GAMMA_REG) & 0x3f;
}

inline static int em28xx_contrast_set(struct em28xx *dev, s32 val)
{
	u8 tmp = (u8) val;
	return em28xx_write_regs(dev, R20_YGAIN_REG, &tmp, 1);
}

inline static int em28xx_brightness_set(struct em28xx *dev, s32 val)
{
	u8 tmp = (u8) val;
	return em28xx_write_regs(dev, R21_YOFFSET_REG, &tmp, 1);
}

inline static int em28xx_saturation_set(struct em28xx *dev, s32 val)
{
	u8 tmp = (u8) val;
	return em28xx_write_regs(dev, R22_UVGAIN_REG, &tmp, 1);
}

inline static int em28xx_u_balance_set(struct em28xx *dev, s32 val)
{
	u8 tmp = (u8) val;
	return em28xx_write_regs(dev, R23_UOFFSET_REG, &tmp, 1);
}

inline static int em28xx_v_balance_set(struct em28xx *dev, s32 val)
{
	u8 tmp = (u8) val;
	return em28xx_write_regs(dev, R24_VOFFSET_REG, &tmp, 1);
}

inline static int em28xx_gamma_set(struct em28xx *dev, s32 val)
{
	u8 tmp = (u8) val;
	return em28xx_write_regs(dev, R14_GAMMA_REG, &tmp, 1);
}

/*FIXME: maxw should be dependent of alt mode */
inline static unsigned int norm_maxw(struct em28xx *dev)
{
	switch(dev->model){
//		case (EM2820_BOARD_MSI_VOX_USB_2): return(640);
		default: return(720);
	}
}

inline static unsigned int norm_maxh(struct em28xx *dev)
{
	switch(dev->model){
//		case (EM2820_BOARD_MSI_VOX_USB_2): return(480);
		default: return (dev->tvnorm->id & V4L2_STD_625_50) ? 576 : 480;
	}
}

#endif
