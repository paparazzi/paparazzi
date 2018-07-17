# FATFS files.
FATFSSRC = $(CHIBIOS)/os/various/fatfs_bindings/fatfs_diskio.c \
           $(CHIBIOS)/os/various/fatfs_bindings/fatfs_syscall.c \
           $(PAPARAZZI_SRC)/sw/ext/fatfs/src/ff.c \
           $(PAPARAZZI_SRC)/sw/ext/fatfs/src/ffunicode.c

FATFSINC = $(PAPARAZZI_SRC)/sw/ext/fatfs/src

# Shared variables
ALLCSRC += $(FATFSSRC)
ALLINC  += $(FATFSINC)
