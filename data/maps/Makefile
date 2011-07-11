DATADIR = $(PAPARAZZI_HOME)/conf/maps_data
Q=@

all: $(DATADIR)/maps.google.com $(DATADIR)/maps.conf

$(DATADIR):
	mkdir $(DATADIR)

$(DATADIR)/maps.google.com: $(DATADIR)
	wget -O $(@) http://maps.google.com/

$(DATADIR)/maps.conf: $(DATADIR)/maps.google.com
	$(Q)echo "[GOOGLE]" > $(@)
	$(Q)echo "google_version:" `grep -P "http://khm[0-9]+.google.com/kh/v=[0-9]+.x26" $(DATADIR)/maps.google.com | sed -E s#.*http://khm[0-9]+.google.com/kh/v=## | sed -E s#.x26.*##` >> $(@)
	$(Q)echo "" >> $(@)

