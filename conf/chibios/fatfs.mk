# FATFS files.
FATFSSRC = ${CHIBIOS}/os/various/fatfs_bindings/fatfs_diskio.c \
           ${CHIBIOS}/os/various/fatfs_bindings/fatfs_syscall.c \
           ${PAPARAZZI_SRC}/sw/ext/fatfs/src/ff.c \
           ${PAPARAZZI_SRC}/sw/ext/fatfs/src/option/ccsbcs.c

FATFSINC = ${PAPARAZZI_SRC}/sw/ext/fatfs/src
