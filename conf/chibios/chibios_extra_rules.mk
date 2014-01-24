# Extra rules for ChibiOS rules.mk


#
# Initial directory tree is preserved for the build
#
ECOBJS		= $(addprefix $(OBJDIR)/, $(ECSRC:.c=.o))

$(ECOBJS) : $(OBJDIR)/%.o : %.c Makefile
ifeq ($(USE_VERBOSE_COMPILE),yes)
	@echo
	VPATH=
	test -d $(dir $@) || mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
else
	@echo Compiling $(<F)
	@VPATH=
	@test -d $(dir $@) || mkdir -p $(dir $@)
	@$(CC) -c $(CFLAGS) $(TOPT) -I. $(IINCDIR) $< -o $@
endif

OBJS	  += $(ECOBJS)

# *** EOF ***
