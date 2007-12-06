bin_PROGRAMS = spook spookctl
EXTRA_DIST = spook.conf.dist

spookctl_SOURCES = spookctl.c event.c event.h

spook_SOURCES = conf_parse.c conf_scan.l conversions.c log.c pmsg.c md5.c \
		event.c filter-framedrop.c frame.c control.c tcp.c http.c \
		audio.c rtp.c session.c rtsp.c spook.c stream.c access_log.c \
		http-auth.c live.c rtp-mpeg4.c rtp-rawaudio.c rtp-mpa.c \
		rtp-mpv.c rtp-h263.c rtp-jpeg.c input-oss.c \
		encoder-mp2.c mpegaudio.c encoder-alaw.c sound_conv.c \
		conf_parse.h conversions.h encoders.h event.h filters.h \
		frame.h global_config.h inputs.h outputs.h rtp.h stream.h \
		pmsg.h rtp_media.h base64_table.h log.h mpegaudio.h bswap.h \
		jpeg_tables.h md5.h
spook_LDADD = -lm

if BUILD_INPUT_V4L
spook_SOURCES += input-v4l.c
endif

if BUILD_INPUT_V4L2
spook_SOURCES += input-v4l2.c
endif

if BUILD_INPUT_DC1394
spook_SOURCES += input-dc1394.c
endif

if BUILD_INPUT_VDIG
spook_SOURCES += input-vdig.c
spook_LDADD += $(QTLIBS)
endif

if BUILD_ENCODER_MPEG4
spook_SOURCES += encoder-mpeg4.c decoder-mpeg4.c
endif

if BUILD_ENCODER_JPEG
spook_SOURCES += encoder-jpeg.c
endif
