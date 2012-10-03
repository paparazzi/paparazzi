DATADIR = $(PAPARAZZI_HOME)/conf/maps_data
Q=@

all: $(DATADIR)/maps.google.com $(PAPARAZZI_HOME)/conf/maps.xml

clean:


$(DATADIR):
	mkdir $(DATADIR)

$(DATADIR)/maps.google.com: $(DATADIR) FORCE
	@echo "DOWNLOAD: google maps version code";
	$(Q)wget -q -O $(@) http://maps.google.com/ || \
		(rm -f $(@) && echo "Could not download google maps version code" && exit 1)

$(PAPARAZZI_HOME)/conf/maps.xml: $(DATADIR)/maps.google.com
	$(Q)echo "<!DOCTYPE maps SYSTEM \"maps.dtd\">" > $(@)
	$(Q)echo "" >> $(@)
	$(Q)echo "<maps google_version=\""`grep -P "http://khm[0-9]+.google.com/kh/v=[0-9]+.x26" $(DATADIR)/maps.google.com | sed -E 's#.*http://khm[0-9]+.google.com/kh/v=##;s#.x26.*##'`"\"/>" >> $(@)
	$(Q)echo "" >> $(@)

FORCE:
