# analog pitotube on ADC

#  <section name="adc" prefix="ADC_CHANNEL_">
#    <define name="AIRSPEED" value="ADC_5"/>
#    <define name="AIRSPEED_NB_SAMPLES" value="16"/> 
#  </section>

#  <section name="AIRSPEED" prefix="AIRSPEED_">
#    <define name="BIAS" value="(168.0f)"/>
#    <define name="QUADRATIC_SCALE" value="1.2588f"/>
#  </section>

ifdef USE_AIRSPEED
	$(TARGET).CFLAGS += -DUSE_AIRSPEED -DADC_CHANNEL_AIRSPEED_NB_SAMPLES=$(AIRSPEED_NB_SAMPLES)
        $(TARGET).CFLAGS += -DADC_CHANNEL_AIRSPEED=$(USE_AIRSPEED) -DUSE_$(USE_AIRSPEED)
	$(TARGET).srcs += airspeed.c
else
	ifdef MEASURE_AIRSPEED
		$(TARGET).CFLAGS += -DMEASURE_AIRSPEED -DAGR_CLIMB 
	endif
endif
