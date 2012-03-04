PERL = perl
TEST_VERBOSE=0
ifeq ($(AIRCRAFT),)
AIRCRAFT=*
endif
TEST_FILES = $(AIRCRAFT)/*.t

test:
	$(PERL) "-MExtUtils::Command::MM" "-e" "test_harness($(TEST_VERBOSE))" $(TEST_FILES)

