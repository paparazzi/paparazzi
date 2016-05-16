# Hey Emacs, this is a -*- makefile -*-

DATADIR = $(PAPARAZZI_HOME)/conf/maps_data
Q=@

UNAME = $(shell uname -s)
ifeq ("$(UNAME)","Darwin")
	MKTEMP = gmktemp
	SED = gsed
else
	MKTEMP = mktemp
	SED = sed
endif


all: $(PAPARAZZI_HOME)/conf/maps.xml

clean:
	$(Q)rm -f $(DATADIR)/maps.google.com

$(DATADIR):
	mkdir $(DATADIR)

$(DATADIR)/maps.google.com: $(DATADIR) FORCE
	@echo "-----------------------------------------------"
	@echo "DOWNLOAD: google maps version code";
	$(Q)wget -q -t 1 -T 10 --no-check-certificate -O $(@) http://maps.googleapis.com/maps/api/js || \
		(rm -f $(@) && \
		echo "Could not download google maps version code" && \
		echo "-----------------------------------------------" && \
		exit 1)

$(PAPARAZZI_HOME)/conf/maps.xml: $(DATADIR)/maps.google.com
	$(eval GOOGLE_VERSION := $(shell tr -s '[[:space:]]' '\n' < $(DATADIR)/maps.google.com | grep -E "http[s]?://khm[s]?[0-9]+.google.com/kh\?v=[0-9]+\\\u" -m 1 | $(SED) -r 's#.*http[s]?://khm[s]?[0-9]+.google.com/kh\?v=##;s#.u0026.*##'))
	$(eval $@_TMP := $(shell $(MKTEMP)))
	$(Q)if [ "$(GOOGLE_VERSION)" -eq "$(GOOGLE_VERSION)" ] 2>/dev/null ; then \
		echo "Updated google maps version to $(GOOGLE_VERSION)" ; \
		echo "-----------------------------------------------" ; \
		echo "<!DOCTYPE maps SYSTEM \"maps.dtd\">" > $($@_TMP) ; \
		echo "" >> $($@_TMP) ; \
		echo "<maps google_version=\"$(GOOGLE_VERSION)\"/>" >> $($@_TMP) ; \
		echo "" >> $($@_TMP) ; \
		mv $($@_TMP) $@ ; \
		chmod a+r $@ ; \
	else \
		echo "Extracted google maps version was not a valid number: $(GOOGLE_VERSION)" ; \
	fi

FORCE:
.PHONY: all clean
