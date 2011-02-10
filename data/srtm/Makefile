DATADIR = $(PAPARAZZI_HOME)/conf/srtm_data
Q=@

SRTMData: $(DATADIR)/Africa $(DATADIR)/Australia $(DATADIR)/Eurasia $(DATADIR)/Islands $(DATADIR)/North_America $(DATADIR)/South_America

$(DATADIR):
	mkdir $(DATADIR)

$(DATADIR)/Africa: $(DATADIR)
	wget -O $(@) http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/$(@F)

$(DATADIR)/Australia: $(DATADIR)
	wget -O $(@) http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/$(@F)

$(DATADIR)/Eurasia: $(DATADIR)
	wget -O $(@) http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/$(@F)

$(DATADIR)/Islands: $(DATADIR)
	wget -O $(@) http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/$(@F)

$(DATADIR)/North_America: $(DATADIR)
	wget -O $(@) http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/$(@F)

$(DATADIR)/South_America: $(DATADIR)
	wget -O $(@) http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/$(@F)

.SUFFIXES: .hgt.zip

%.hgt.zip: SRTMData
	$(Q)wget -c -nv -N http://dds.cr.usgs.gov/srtm/version2_1/SRTM3/$(shell grep -l $(@F) $(DATADIR)/* | sed -e s#$(DATADIR)/##)/$(@F)

