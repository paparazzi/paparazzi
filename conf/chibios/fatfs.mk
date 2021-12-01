# FATFS files.
FATFSSRC = $(CHIBIOS)/os/various/fatfs_bindings/fatfs_diskio.c \
           $(CHIBIOS)/os/various/fatfs_bindings/fatfs_syscall.c \
           $(PAPARAZZI_SRC)/sw/ext/fatfs/source/ff.c \
           $(PAPARAZZI_SRC)/sw/ext/fatfs/source/ffunicode.c

FATFSINC = $(PAPARAZZI_SRC)/sw/ext/fatfs/source

# Shared variables
ALLCSRC += $(FATFSSRC)
ALLINC  += $(FATFSINC)
