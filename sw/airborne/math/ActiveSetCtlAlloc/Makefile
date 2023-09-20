#####
# Package config
AS_N_U ?= 20
AS_N_V ?= 6
DEFINES = -DAS_N_U=$(AS_N_U) -DAS_N_V=$(AS_N_V)

VERBOSE?=n
ifeq ($(VERBOSE), y)
DEFINES += -DAS_VERBOSE
endif

SINGLE?=n
ifeq ($(SINGLE), y)
DEFINES += -DAS_SINGLE_FLOAT
endif

TRUNCATE?=y
ifeq ($(TRUNCATE), y)
DEFINES += -DAS_COST_TRUNCATE
endif

RECORD_COST?=n
ifeq ($(RECORD_COST), y)
DEFINES += -DAS_RECORD_COST
RECORD_COST_N?=15
DEFINES += -DAS_RECORD_COST_N=$(RECORD_COST_N)
endif

ifdef RTOL
DEFINES += -DAS_RTOL=$(RTOL)
endif

ifdef CTOL
DEFINES += -DAS_CTOL=$(CTOL)
endif

INCLUDE_CG?=n
ifeq ($(INCLUDE_CG),y)
DEFINES += -DAS_INCLUDE_CG
endif

STATIC?=n
ifeq ($(STATIC), y)
LIB_EXT = a
else
LIB_EXT = so
endif

OPTI?=3
DEBUG?=n
ifeq ($(DEBUG), y)
# override optimisations
OPTI=0
DEBUG_FLAG=-g
endif

OPTIM = -O$(OPTI) -fno-loop-optimize -fno-aggressive-loop-optimizations
CONF = $(DEFINES) $(DEBUG_FLAG) $(OPTIM) $(VERBOSE_FLAG)

#########
# library paths/names
LIB_NAME = as
SRC_DIR = ./src
BIN_DIR = ./bin
LIBRARY = $(BIN_DIR)/lib$(LIB_NAME).$(LIB_EXT)
SOURCES_INSIDE_SRC = common/solveActiveSet.c solveActiveSet_chol.c solveActiveSet_qr.c solveActiveSet_qr_naive.c common/setupWLS.c lib/chol_math.c lib/qr_updates.c lib/qr_wrapper.c lib/qr_solve/qr_solve.c lib/qr_solve/r8lib_min.c lib/sparse_math.c
ifeq ($(INCLUDE_CG),y)
SOURCES_INSIDE_SRC += solveActiveSet_cg.c
endif
SOURCES = $(addprefix $(SRC_DIR)/, $(SOURCES_INSIDE_SRC))
BINARIES = $(addprefix $(BIN_DIR)/, $(SOURCES_INSIDE_SRC:%.c=%.o))

# tester paths/names
TESTER = tests
TEST_SOURCES = tests.c
TEST_BINARIES = $(TEST_SOURCES:%.c=%.o)

#########
# compiler conf
CC = gcc
WARN_FLAGS = -Wall -W -Wwrite-strings -Winline -Wstrict-prototypes -Wnested-externs -Wpointer-arith -Wcast-align -Wcast-qual -Wshadow -Werror=vla
CC_FLAGS = -fstack-usage -fwrapv -fPIC ${WARN_FLAGS} $(CONF)
INCLUDES = -Isrc/common -Isrc/lib -Isrc
LINK_FLAGS = -lm

# other programs
AR = ar rcsD
RM = rm -f

#########
# gateway goals
.DEFAULT_GOAL = library
perform_tests : tester
	@echo QR_NAIVE: && ./$(TESTER) verify 0
	@echo QR: && ./$(TESTER) verify 1
	@echo CHOL: && ./$(TESTER) verify 2
	@echo CG, only representative if compiled with -DAS_INCLUDE_CG: && ./$(TESTER) verify 3
tester : $(TESTER)
library : $(LIBRARY)
clean : cleaner

#########
# actual targets
$(TESTER) : $(TEST_BINARIES) $(LIBRARY)
	$(CC) ${CC_FLAGS} $+ -o $@ $(INCLUDES) $(LINK_FLAGS)
# $+ is all prereqs including douplicates and in order

%.o : %.c
ifeq ($(OPTI), 0)
	$(CC) -c ${CC_FLAGS} -O0 $^ -o $@ $(INCLUDES)
else
# O3 is buggy here for some reason and generates spurious warnings
	$(CC) -c ${CC_FLAGS} -O1 $^ -o $@ $(INCLUDES)
endif

$(LIBRARY) : $(BINARIES)
ifeq ($(STATIC),y)
	$(AR) $@ $^
# the $? would only copies the changed $(BINARIES) into the archive thanks to the r 
# flag in the command for ar, pretty neat
else
	$(CC) -shared $(CC_FLAGS) $^ -o $@ $(INCLUDES)
endif

$(BIN_DIR)/%.o : $(SRC_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) -c $(CC_FLAGS) $^ -o $@ $(INCLUDES)

cleaner : 
	$(RM) -rf bin
	$(RM) *.o
	${RM} *.su
	$(RM) $(TESTER)
