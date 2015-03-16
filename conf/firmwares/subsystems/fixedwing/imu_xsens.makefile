# Hey Emacs, this is a -*- makefile -*-

# XSens Mti just providing IMU measurements

#    <subsystem name="imu" type="xsens">
#      <configure name="XSENS_UART_NR" value="0"/>
#      <configure name="XSENS_UART_BAUD" value="B115200"/>
#    </subsystem>
#
#  <section name="IMU" prefix="IMU_">
#    <define name="GYRO_P_SIGN"  value="1"/>
#    <define name="GYRO_Q_SIGN"  value="1"/>
#    <define name="GYRO_R_SIGN"  value="1"/>
#
#    <define name="GYRO_P_NEUTRAL" value="0"/>
#    <define name="GYRO_R_NEUTRAL" value="0"/>
#    <define name="GYRO_Q_NEUTRAL" value="0"/>
#
#    <define name="GYRO_P_SENS" value="1" integer="16"/>
#    <define name="GYRO_R_SENS" value="1" integer="16"/>
#    <define name="GYRO_Q_SENS" value="1" integer="16"/>
#
#    <define name="ACCEL_X_SIGN"  value="1"/>
#    <define name="ACCEL_Y_SIGN"  value="1"/>
#    <define name="ACCEL_Z_SIGN"  value="1"/>
#
#    <define name="ACCEL_X_SENS" value="1" integer="16"/>
#    <define name="ACCEL_Z_SENS" value="1" integer="16"/>
#    <define name="ACCEL_Y_SENS" value="1" integer="16"/>
#
#    <define name="ACCEL_X_NEUTRAL" value="0"/>
#    <define name="ACCEL_Z_NEUTRAL" value="0"/>
#    <define name="ACCEL_Y_NEUTRAL" value="0"/>
#
#    <define name="MAG_X_SIGN" value="1"/>
#    <define name="MAG_Y_SIGN" value="1"/>
#    <define name="MAG_Z_SIGN" value="1"/>
#
#    <define name="MAG_X_NEUTRAL" value="-45"/>
#    <define name="MAG_Y_NEUTRAL" value="334"/>
#    <define name="MAG_Z_NEUTRAL" value="7"/>
#
#    <define name="MAG_X_SENS" value="4.47647816128" integer="16"/>
#    <define name="MAG_Y_SENS" value="4.71085671542" integer="16"/>
#    <define name="MAG_Z_SENS" value="4.41585354498" integer="16"/>
#
#    <define name="BODY_TO_IMU_PHI" value="0" unit="deg"/>
#    <define name="BODY_TO_IMU_THETA" value="0" unit="deg"/>
#    <define name="BODY_TO_IMU_PSI" value="0" unit="deg"/>
#  </section>


#########################################
## IMU

ap.CFLAGS += -DUSE_IMU
ap.CFLAGS += -DIMU_TYPE_H=\"modules/ins/ins_xsens.h\"
ap.srcs   += $(SRC_MODULES)/ins/ins_xsens.c
ap.srcs   += $(SRC_SUBSYSTEMS)/imu.c

ifndef XSENS_UART_BAUD
	XSENS_UART_BAUD = B115200
endif

ap.CFLAGS += -DUSE_UART$(XSENS_UART_NR)
ap.CFLAGS += -DINS_LINK=uart$(XSENS_UART_NR)
ap.CFLAGS += -DUART$(XSENS_UART_NR)_BAUD=$(XSENS_UART_BAUD)
ap.CFLAGS += -DXSENS_OUTPUT_MODE=0x1836
