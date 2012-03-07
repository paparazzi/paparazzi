Q = @
PERL = /usr/bin/perl
TEST_VERBOSE = 0
ifeq ($(TARGET_BOARD),)
	TARGET_BOARD = *
endif
TEST_FILES := $(shell ls $(TARGET_BOARD)/*.t)

ifneq ($(JUNIT),)
	PERLENV=PERL_TEST_HARNESS_DUMP_TAP=$(PAPARAZZI_SRC)/tests/results
	RUNTESTS=use TAP::Harness; TAP::Harness->new({ formatter_class => 'TAP::Formatter::JUnit', merge => 1, } )->runtests(qw($(TEST_FILES)))
else
	PERLENV=
	RUNTESTS=use TAP::Harness;TAP::Harness->new()->runtests(qw($(TEST_FILES)))
endif

test:
	$(Q)$(PERLENV) $(PERL) "-e" "$(RUNTESTS)"

