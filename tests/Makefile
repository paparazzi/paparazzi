Q = @
PERL = /usr/bin/perl
TEST_VERBOSE = 0
TEST_JOBS ?= 1
NON_TEST_DIRS = lib|results
HARDWARE_REGEX = Lisa|Tiny|Umarim
HARDWARE_TEST_DIRS = $(shell find -L * -maxdepth 1 -type d | grep -Ev "$(NON_TEST_DIRS)" | grep -E "$(HARDWARE_REGEX)")
NON_HARDWARE_TEST_DIRS = $(shell find -L * -maxdepth 1 -type d | grep -Ev "$(NON_TEST_DIRS)" | grep -Ev "$(HARDWARE_REGEX)")
ifneq ($(TARGET_BOARD),)
	TEST_DIRECTORIES = $(TARGET_BOARD)
else
ifeq ($(TEST_HARDWARE),)
	TEST_DIRECTORIES = $(NON_HARDWARE_TEST_DIRS)
else
	TEST_DIRECTORIES = $(NON_HARDWARE_TEST_DIRS) $(HARDWARE_TEST_DIRS)
endif
endif
TEST_FILES ?= $(shell ls $(TEST_DIRECTORIES:%=%/*.t) 2> /dev/null)

ifneq ($(JUNIT),)
	PERLENV=PERL_TEST_HARNESS_DUMP_TAP=$(PAPARAZZI_SRC)/tests/results
	RUNTESTS=use TAP::Harness; TAP::Harness->new({ formatter_class => 'TAP::Formatter::JUnit', verbosity => $(TEST_VERBOSE), merge => 1, } )->runtests(qw($(TEST_FILES)))
else
	PERLENV=
	RUNTESTS=use TAP::Harness; exit TAP::Harness->new( { verbosity => $(TEST_VERBOSE), jobs => $(TEST_JOBS) } )->runtests(qw($(TEST_FILES)))->failed
endif

test:
	$(Q)make -C math test
	$(Q)$(PERLENV) $(PERL) "-e" "$(RUNTESTS)"

clean:
	$(Q)rm -rf results/*

