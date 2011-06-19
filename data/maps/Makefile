DATADIR = $(PAPARAZZI_HOME)/conf/maps_data
Q=@

all: $(DATADIR)/index $(DATADIR)/version

$(DATADIR):
	mkdir $(DATADIR)

$(DATADIR)/index: $(DATADIR)
	$(@)wget -O $(@) http://maps.google.com/

$(DATADIR)/version: $(DATADIR)/index
	$(Q)grep -P "http://khm[0-9]+.google.com/kh/v=[0-9]+.x26" $(DATADIR)/index | sed -E s#.*http://khm[0-9]+.google.com/kh/v=## | sed -E s#.x26.*## > $(@)
