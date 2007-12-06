AC_INIT(Spook, 20050207)
AC_CONFIG_SRCDIR(spook.c)
AM_INIT_AUTOMAKE([foreign dist-bzip2])
AC_CONFIG_HEADERS(config.h)

AC_CANONICAL_HOST

AC_PROG_CC
AM_PROG_LEX

CFLAGS="$CFLAGS -pthread -Wall -fno-strict-aliasing"
SPOOK_INPUT_ENABLED=""
SPOOK_INPUT_DISABLED=""
SPOOK_ENCODER_ENABLED=" mp2"
SPOOK_ENCODER_DISABLED=""

AC_ARG_WITH(kernel-source,
	AC_HELP_STRING([--with-kernel-source],
		       [location of your Linux kernel source tree (if not /usr/src/linux)]),
	[
		linux_source=$with_kernel_source
	],[
		linux_source=/usr/src/linux
	])
CFLAGS="$CFLAGS -I$linux_source/include"

AC_ARG_ENABLE(input-v4l,
	AC_HELP_STRING([--enable-input-v4l],
		       [compile support for Video4Linux]),,
	[enable_input_v4l=auto])

AC_ARG_ENABLE(input-v4l2,
	AC_HELP_STRING([--enable-input-v4l2],
		       [compile support for Video4Linux2]),,
	[enable_input_v4l2=auto])

AC_ARG_ENABLE(input-dc1394,
	AC_HELP_STRING([--enable-input-dc1394],
		       [compile support for Firewire IIDC cameras in Linux]),,
	[enable_input_dc1394=auto])

AC_ARG_ENABLE(input-vdig,
	AC_HELP_STRING([--enable-input-vdig],
		       [compile support for Apple QuickTime digitizers]),,
	[enable_input_vdig=auto])

AC_ARG_ENABLE(encoder-mpeg4,
	AC_HELP_STRING([--enable-encoder-mpeg4],
		       [compile support for MPEG4 encoding (requires XviD 1.0.0)]),,
	[enable_encoder_mpeg4=auto])

AC_ARG_ENABLE(encoder-jpeg,
	AC_HELP_STRING([--enable-encoder-jpeg],
		       [compile support for JPEG encoding]),,
	[enable_encoder_jpeg=auto])

case $host in
	*-*-linux*)
		if test x$enable_input_vdig = xyes; then
			AC_MSG_ERROR([QuickTime digitizer support is only available on Mac OS X])
		fi
		enable_input_vdig=no
		AC_CHECK_HEADERS([asm/types.h linux/compiler.h])
		;;
	*-*-darwin*)
		AC_MSG_CHECKING([whether Fink is installed])
		if test -d /sw/fink; then
			AC_MSG_RESULT(yes)
			CFLAGS="$CFLAGS -I/sw/include"
			LIBS="-L/sw/lib $LIBS"
		else
			AC_MSG_RESULT(no)
		fi
		if test x$enable_input_v4l = xyes; then
			AC_MSG_ERROR([V4L is only available on Linux])
		fi
		enable_input_v4l=no
		if test x$enable_input_v4l2 = xyes; then
			AC_MSG_ERROR([V4L2 is only available on Linux])
		fi
		enable_input_v4l2=no
		;;
	*)
		if test x$enable_input_v4l = xyes; then
			AC_MSG_ERROR([V4L is only available on Linux])
		fi
		enable_input_v4l=no
		if test x$enable_input_v4l2 = xyes; then
			AC_MSG_ERROR([V4L2 is only available on Linux])
		fi
		enable_input_v4l2=no
		if test x$enable_input_vdig = xyes; then
			AC_MSG_ERROR([QuickTime digitizer support is only available on Mac OS X])
		fi
		enable_input_vdig=no
		;;
esac

build_input_v4l=false
if test x$enable_input_v4l != xno; then
	AC_MSG_CHECKING([for Video4Linux header files])
	AC_COMPILE_IFELSE(
		AC_LANG_PROGRAM([[
			#include <sys/types.h>
			#include <sys/time.h>
			#ifdef HAVE_ASM_TYPES_H
			#include <asm/types.h>
			#endif
			#ifdef HAVE_LINUX_COMPILER_H
			#include <linux/compiler.h>
			#endif
			#include <linux/videodev.h>
		]],[[
			struct video_capability vc;
			return 0;
		]]),
	[
		AC_MSG_RESULT(yes)
		build_input_v4l=true
		AC_MSG_CHECKING([for pwc-ioctl.h in the kernel source tree])
		if test -d $linux_source/drivers/usb/media; then
			CFLAGS="$CFLAGS -I$linux_source/drivers/usb/media"
		elif test -d $linux_source/drivers/usb; then
			CFLAGS="$CFLAGS -I$linux_source/drivers/usb"
		fi
		AC_COMPILE_IFELSE(
			AC_LANG_PROGRAM([[
				#include <sys/types.h>
				#include <sys/time.h>
				#ifdef HAVE_ASM_TYPES_H
				#include <asm/types.h>
				#endif
				#ifdef HAVE_LINUX_COMPILER_H
				#include <linux/compiler.h>
				#endif
				#include <sys/ioctl.h>
				#include <linux/videodev.h>
				#include <pwc-ioctl.h>
			]],[[
				int x = PWC_FPS_MASK;
				return 0;
			]]),
		[
			AC_MSG_RESULT(yes)
			AC_DEFINE(HAVE_PWC_IOCTL_H,1,
				[Define to 1 if you have the <pwc-ioctl.h> header file.])
		],[
			AC_MSG_RESULT(no)
		])
	],[
		AC_MSG_RESULT(no)
		if test x$enable_input_v4l != xauto; then
			AC_MSG_ERROR([Unable to find the V4L headers])
		fi
	])
fi

build_input_v4l2=false
if test x$enable_input_v4l2 != xno; then
	AC_MSG_CHECKING([for Video4Linux2 header files])
	AC_COMPILE_IFELSE(
		AC_LANG_PROGRAM([[
			#include <sys/types.h>
			#include <sys/mman.h>
			#include <sys/ioctl.h>
			#ifdef HAVE_ASM_TYPES_H
			#include <asm/types.h>
			#endif
			#ifdef HAVE_LINUX_COMPILER_H
			#include <linux/compiler.h>
			#endif
			#include <linux/videodev2.h>
		]],[[
			struct v4l2_capability vc;
			return 0;
		]]),
	[
		AC_MSG_RESULT(yes)
		build_input_v4l2=true
		AC_MSG_CHECKING([for go7007.h in the kernel source tree])
		AC_COMPILE_IFELSE(
			AC_LANG_PROGRAM([[
				#include <sys/types.h>
				#include <sys/mman.h>
				#include <sys/ioctl.h>
				#ifdef HAVE_ASM_TYPES_H
				#include <asm/types.h>
				#endif
				#ifdef HAVE_LINUX_COMPILER_H
				#include <linux/compiler.h>
				#endif
				#include <linux/videodev2.h>
				#include <linux/go7007.h>
			]],[[
				unsigned int x = GO7007IOC_S_BITRATE;
				return 0;
			]]),
		[
			AC_MSG_RESULT(yes)
			AC_DEFINE(HAVE_GO7007_H,1,
				[Define to 1 if you have the <go7007.h> header file.])
		],[
			AC_MSG_RESULT(no)
		])
	],[
		AC_MSG_RESULT(no)
		if test x$enable_input_v4l2 != xauto; then
			AC_MSG_ERROR([Unable to find the V4L2 headers; is your kernel source installed?])
		fi
	])
fi

build_input_dc1394=false
if test x$enable_input_dc1394 != xno; then
	AC_CHECK_LIB(dc1394_control,dc1394_create_handle,[
		LIBS="-ldc1394_control -lraw1394 $LIBS"
		build_input_dc1394=true
		AC_MSG_CHECKING([whether the extra_buffering flag is needed])
		AC_COMPILE_IFELSE(
			AC_LANG_PROGRAM([[
				#include <libraw1394/raw1394.h>
				#include <libdc1394/dc1394_control.h>
			]],[[
				dc1394_dma_setup_capture( NULL, NULL, 1, FORMAT_VGA_NONCOMPRESSED, MODE_320x240_YUV422, SPEED_400, FRAMERATE_30, 0, 0, 1, NULL, NULL );
				return 0;
			]]),
		[
			AC_MSG_RESULT(yes)
			AC_DEFINE(DC1394_EXTRA_BUFFERING_FLAG,1,
				[Define to 1 if dc1394_dma_setup_capture needs the extra_buffering argument.])
		],[
			AC_MSG_RESULT(no)
		])
	],[
		if test x$enable_input_dc1394 != xauto; then
			AC_MSG_ERROR([Unable to find the libdc1394_control library])
		fi
	],[-lraw1394])
fi

build_input_vdig=false
if test x$enable_input_vdig != xno; then
	if test x$enable_input_vdig = xyes; then
		build_input_vdig=true
		QTLIBS="/System/Library/Frameworks/QuickTime.framework/QuickTime /System/Library/Frameworks/CoreServices.framework/CoreServices"
	fi
fi

build_encoder_mpeg4=false
if test x$enable_encoder_mpeg4 != xno; then
	AC_CHECK_LIB(xvidcore,xvid_encore,[
		AC_MSG_CHECKING([for XviD version 1.0.0])
		AC_RUN_IFELSE(
			AC_LANG_PROGRAM([[#include <xvid.h>]],[[
				exit( XVID_VERSION_MAJOR(XVID_VERSION) != 1 );
				]]),
		[
			AC_MSG_RESULT(yes)
			LIBS="-lxvidcore $LIBS"
			build_encoder_mpeg4=true
		],[
			AC_MSG_RESULT(no)
			if test x$enable_encoder_mpeg4 != xauto; then
				AC_MSG_ERROR([XviD is too old])
			fi
		])
	],[
		if test x$enable_encoder_mpeg4 != xauto; then
			AC_MSG_ERROR([Unable to find libxvidcore.so])
		fi
	])
fi

build_encoder_jpeg=false
if test x$enable_encoder_jpeg != xno; then
	AC_CHECK_LIB(jpeg,jpeg_set_defaults,[
		LIBS="-ljpeg $LIBS"
		build_encoder_jpeg=true
	],[
		if test x$enable_encoder_jpeg != xauto; then
			AC_MSG_ERROR([Unable to find JPEG-6b])
		fi
	])
fi

if test x$build_input_v4l = xtrue; then
	AC_DEFINE(SPOOK_INPUT_V4L,1,
		[Define to include the input-v4l module.])
	SPOOK_INPUT_ENABLED="$SPOOK_INPUT_ENABLED v4l"
else
	SPOOK_INPUT_DISABLED="$SPOOK_INPUT_DISABLED v4l"
fi
AM_CONDITIONAL(BUILD_INPUT_V4L, test x$build_input_v4l = xtrue)

if test x$build_input_v4l2 = xtrue; then
	AC_DEFINE(SPOOK_INPUT_V4L2,1,
		[Define to include the input-v4l2 module.])
	SPOOK_INPUT_ENABLED="$SPOOK_INPUT_ENABLED v4l2"
else
	SPOOK_INPUT_DISABLED="$SPOOK_INPUT_DISABLED v4l2"
fi
AM_CONDITIONAL(BUILD_INPUT_V4L2, test x$build_input_v4l2 = xtrue)

if test x$build_input_dc1394 = xtrue; then
	AC_DEFINE(SPOOK_INPUT_DC1394,1,
		[Define to include the input-dc1394 module.])
	SPOOK_INPUT_ENABLED="$SPOOK_INPUT_ENABLED dc1394"
else
	SPOOK_INPUT_DISABLED="$SPOOK_INPUT_DISABLED dc1394"
fi
AM_CONDITIONAL(BUILD_INPUT_DC1394, test x$build_input_dc1394 = xtrue)

if test x$build_input_vdig = xtrue; then
	AC_DEFINE(SPOOK_INPUT_VDIG,1,
		[Define to include the input-vdig module.])
	SPOOK_INPUT_ENABLED="$SPOOK_INPUT_ENABLED vdig"
else
	SPOOK_INPUT_DISABLED="$SPOOK_INPUT_DISABLED vdig"
fi
AM_CONDITIONAL(BUILD_INPUT_VDIG, test x$build_input_vdig = xtrue)

if test x$build_encoder_mpeg4 = xtrue; then
	AC_DEFINE(SPOOK_ENCODER_MPEG4,1,
		[Define to include the encoder-mpeg4 module.])
	SPOOK_ENCODER_ENABLED="$SPOOK_ENCODER_ENABLED mpeg4"
else
	SPOOK_ENCODER_DISABLED="$SPOOK_ENCODER_DISABLED mpeg4"
fi
AM_CONDITIONAL(BUILD_ENCODER_MPEG4, test x$build_encoder_mpeg4 = xtrue)

if test x$build_encoder_jpeg = xtrue; then
	AC_DEFINE(SPOOK_ENCODER_JPEG,1,
		[Define to include the encoder-jpeg module.])
	SPOOK_ENCODER_ENABLED="$SPOOK_ENCODER_ENABLED jpeg"
else
	SPOOK_ENCODER_DISABLED="$SPOOK_ENCODER_DISABLED jpeg"
fi
AM_CONDITIONAL(BUILD_ENCODER_JPEG, test x$build_encoder_jpeg = xtrue)

AC_SUBST(SPOOK_INPUT_V4L)
AC_SUBST(SPOOK_INPUT_V4L2)
AC_SUBST(SPOOK_INPUT_DC1394)
AC_SUBST(SPOOK_INPUT_VDIG)
AC_SUBST(SPOOK_ENCODER_MPEG4)
AC_SUBST(SPOOK_ENCODER_JPEG)
AC_SUBST(QTLIBS)

AC_OUTPUT([Makefile])

echo 
echo "Here are the results of configure.  Check to make sure the modules you"
echo "need have been enabled."
echo
echo "  Input modules"
echo "     Enabled:  $SPOOK_INPUT_ENABLED"
echo "     Disabled: $SPOOK_INPUT_DISABLED"
echo
echo "  Encoder modules"
echo "     Enabled:  $SPOOK_ENCODER_ENABLED"
echo "     Disabled: $SPOOK_ENCODER_DISABLED"
echo
echo "If some modules are unexpectedly disabled, check the result of the"
echo "configure tests to see if any required libraries are missing.  You"
echo "can check which libraries are necessary for a specific module by"
echo "invoking configure with the \`--enable-module\` argument, and"
echo "configure will abort when a check fails for a library necessary for"
echo "that module.  (Run \`configure --help\` for a list of --enable options.)"
echo
