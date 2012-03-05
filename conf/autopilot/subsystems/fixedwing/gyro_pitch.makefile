$(info Error: Please replace <subsystem name="gyro" type="pitch"/> with)
$(info   <subsystem name="imu" type="analog">)
$(info     <configure name="GYRO_P" value="ADC_3"/>)
$(info     <configure name="GYRO_Q" value="ADC_4"/>)
$(info   </subsystem>)
$(info and replace the GYRO section with the appropriate IMU section
$(info in your airframe file.)

$(error The gyro_pitch subsystem has been removed)
