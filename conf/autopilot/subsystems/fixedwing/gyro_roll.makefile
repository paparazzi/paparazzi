$(info Error: Please replace <subsystem name="gyro" type="roll"/> with)
$(info   <subsystem name="imu" type="analog_gyro">)
$(info     <configure name="GYRO_P" value="ADC_3"/>)
$(info   </subsystem>)
$(info in your airframe file.)

$(error The gyro_pitch subsystem has been removed)
