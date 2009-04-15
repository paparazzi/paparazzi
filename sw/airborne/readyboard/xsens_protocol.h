/* Generated from /home/poine/work/savannah/paparazzi3/conf/xsens_MTi-G.xml */
/* Please DO NOT EDIT */

#define XSENS_START 0xFA
#define XSENS_BID 0xFF


#define XSENS_WakeUp_ID 0x3E

#define XSENS_WakeUpAck_ID 0x3F
#define XSENS_WakeUpAck() { \
  XsensHeader(XSENS_WakeUpAck_ID, 0);\
  XsensTrailer();\
}
#define XSENS_GoToConfig_ID 0x30
#define XSENS_GoToConfig() { \
  XsensHeader(XSENS_GoToConfig_ID, 0);\
  XsensTrailer();\
}
#define XSENS_GoToConfigAck_ID 0x31

#define XSENS_GoToMeasurment_ID 0x10
#define XSENS_GoToMeasurment() { \
  XsensHeader(XSENS_GoToMeasurment_ID, 0);\
  XsensTrailer();\
}
#define XSENS_GoToMeasurmentAck_ID 0x11

#define XSENS_Reset_ID 0x40
#define XSENS_Reset() { \
  XsensHeader(XSENS_Reset_ID, 0);\
  XsensTrailer();\
}
#define XSENS_ResetAck_ID 0x41

#define XSENS_ReqDID_ID 0x00
#define XSENS_ReqDID() { \
  XsensHeader(XSENS_ReqDID_ID, 0);\
  XsensTrailer();\
}
#define XSENS_DeviceID_ID 0x01
#define XSENS_DeviceID_id(_xsens_payload) (uint32_t)(*((uint8_t*)_xsens_payload+3+0)|*((uint8_t*)_xsens_payload+2+0)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0))<<24)

#define XSENS_ReqDataLength_ID 0x0A
#define XSENS_ReqDataLength() { \
  XsensHeader(XSENS_ReqDataLength_ID, 0);\
  XsensTrailer();\
}
#define XSENS_DataLength_ID 0x0B
#define XSENS_DataLength_length(_xsens_payload) (uint16_t)(*((uint8_t*)_xsens_payload+1+0)|*((uint8_t*)_xsens_payload+0)<<8)

#define XSENS_Error_ID 0x42
#define XSENS_Error_errorcode(_xsens_payload) (uint8_t)(*((uint8_t*)_xsens_payload+0))

#define XSENS_ReqBaudrate_ID 0x18
#define XSENS_ReqBaudrate() { \
  XsensHeader(XSENS_ReqBaudrate_ID, 0);\
  XsensTrailer();\
}
#define XSENS_ReqBaudrateAck_ID 0x19
#define XSENS_ReqBaudrateAck_baudrate(_xsens_payload) (uint8_t)(*((uint8_t*)_xsens_payload+0))

#define XSENS_SetBaudrate_ID 0x18
#define XSENS_SetBaudrate(baudrate) { \
  XsensHeader(XSENS_SetBaudrate_ID, 1);\
  uint8_t _baudrate = baudrate; XsensSend1ByAddr((uint8_t*)&_baudrate);\
  XsensTrailer();\
}
#define XSENS_SetBaudrateAck_ID 0x19

#define XSENS_RestoreFactoryDef_ID 0x0E
#define XSENS_RestoreFactoryDef() { \
  XsensHeader(XSENS_RestoreFactoryDef_ID, 0);\
  XsensTrailer();\
}
#define XSENS_RestoreFactoryDefAck_ID 0x0F

#define XSENS_ReqConfiguration_ID 0x0C
#define XSENS_ReqConfiguration() { \
  XsensHeader(XSENS_ReqConfiguration_ID, 0);\
  XsensTrailer();\
}
#define XSENS_Configuration_ID 0x0D
#define XSENS_Configuration_master_id(_xsens_payload) (uint32_t)(*((uint8_t*)_xsens_payload+3+0)|*((uint8_t*)_xsens_payload+2+0)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0))<<24)
#define XSENS_Configuration_sampling(_xsens_payload) (uint16_t)(*((uint8_t*)_xsens_payload+1+4)|*((uint8_t*)_xsens_payload+4)<<8)
#define XSENS_Configuration_skip(_xsens_payload) (uint16_t)(*((uint8_t*)_xsens_payload+1+6)|*((uint8_t*)_xsens_payload+6)<<8)
#define XSENS_Configuration_syn_mode(_xsens_payload) (uint16_t)(*((uint8_t*)_xsens_payload+1+8)|*((uint8_t*)_xsens_payload+8)<<8)
#define XSENS_Configuration_syn_skip(_xsens_payload) (uint16_t)(*((uint8_t*)_xsens_payload+1+10)|*((uint8_t*)_xsens_payload+10)<<8)
#define XSENS_Configuration_syn_offset(_xsens_payload) (uint32_t)(*((uint8_t*)_xsens_payload+3+12)|*((uint8_t*)_xsens_payload+2+12)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+12))<<16|((uint32_t)*((uint8_t*)_xsens_payload+12))<<24)
#define XSENS_Configuration_number(_xsens_payload) (uint16_t)(*((uint8_t*)_xsens_payload+1+96)|*((uint8_t*)_xsens_payload+96)<<8)
#define XSENS_Configuration_id(_xsens_payload) (uint32_t)(*((uint8_t*)_xsens_payload+3+98)|*((uint8_t*)_xsens_payload+2+98)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+98))<<16|((uint32_t)*((uint8_t*)_xsens_payload+98))<<24)
#define XSENS_Configuration_data_length(_xsens_payload) (uint16_t)(*((uint8_t*)_xsens_payload+1+102)|*((uint8_t*)_xsens_payload+102)<<8)
#define XSENS_Configuration_out_mode(_xsens_payload) (uint16_t)(*((uint8_t*)_xsens_payload+1+104)|*((uint8_t*)_xsens_payload+104)<<8)
#define XSENS_Configuration_out_settings(_xsens_payload) (uint32_t)(*((uint8_t*)_xsens_payload+3+106)|*((uint8_t*)_xsens_payload+2+106)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+106))<<16|((uint32_t)*((uint8_t*)_xsens_payload+106))<<24)

#define XSENS_ReqPeriod_ID 0x04
#define XSENS_ReqPeriod() { \
  XsensHeader(XSENS_ReqPeriod_ID, 0);\
  XsensTrailer();\
}
#define XSENS_ReqPeriodAck_ID 0x05
#define XSENS_ReqPeriodAck_period(_xsens_payload) (uint16_t)(*((uint8_t*)_xsens_payload+1+0)|*((uint8_t*)_xsens_payload+0)<<8)

#define XSENS_SetPeriod_ID 0x04
#define XSENS_SetPeriod(period) { \
  XsensHeader(XSENS_SetPeriod_ID, 2);\
  uint16_t _period = period; XsensSend2ByAddr((uint8_t*)&_period);\
  XsensTrailer();\
}
#define XSENS_SetPeriodAck_ID 0x05

#define XSENS_ReqOutputMode_ID 0xD0
#define XSENS_ReqOutputMode() { \
  XsensHeader(XSENS_ReqOutputMode_ID, 0);\
  XsensTrailer();\
}
#define XSENS_ReqOutputModeAck_ID 0xD1
#define XSENS_ReqOutputModeAck_mode(_xsens_payload) (uint16_t)(*((uint8_t*)_xsens_payload+1+0)|*((uint8_t*)_xsens_payload+0)<<8)

#define XSENS_SetOutputMode_ID 0xD0
#define XSENS_SetOutputMode(mode) { \
  XsensHeader(XSENS_SetOutputMode_ID, 2);\
  uint16_t _mode = mode; XsensSend2ByAddr((uint8_t*)&_mode);\
  XsensTrailer();\
}
#define XSENS_SetOutputModeAck_ID 0xD1

#define XSENS_ReqOutputSettings_ID 0xD2
#define XSENS_ReqOutputSettings() { \
  XsensHeader(XSENS_ReqOutputSettings_ID, 0);\
  XsensTrailer();\
}
#define XSENS_ReqOutputSettingsAck_ID OxD3
#define XSENS_ReqOutputSettingsAck_settings(_xsens_payload) (uint32_t)(*((uint8_t*)_xsens_payload+3+0)|*((uint8_t*)_xsens_payload+2+0)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0))<<24)

#define XSENS_SetOutputSettings_ID 0xD2
#define XSENS_SetOutputSettings(settings) { \
  XsensHeader(XSENS_SetOutputSettings_ID, 4);\
  uint32_t _settings = settings; XsensSend4ByAddr((uint8_t*)&_settings);\
  XsensTrailer();\
}
#define XSENS_SetOutputSettingsAck_ID 0xD3

#define XSENS_ReqData_ID 0x34
#define XSENS_ReqData() { \
  XsensHeader(XSENS_ReqData_ID, 0);\
  XsensTrailer();\
}
#define XSENS_MTData_ID 0x32
/* XSENS_MTData_data: variable data size */

#define XSENS_ReqHeading_ID Ox82
#define XSENS_ReqHeading() { \
  XsensHeader(XSENS_ReqHeading_ID, 0);\
  XsensTrailer();\
}
#define XSENS_ReqHeadingAck_ID 0x83
#define XSENS_ReqHeadingAck_heading(_xsens_payload) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+0)|*((uint8_t*)_xsens_payload+2+0)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0))<<24); _f.f; })

#define XSENS_SetHeading_ID 0x82
#define XSENS_SetHeading(heading) { \
  XsensHeader(XSENS_SetHeading_ID, 4);\
  float _heading = heading; XsensSend4ByAddr((uint8_t*)&_heading);\
  XsensTrailer();\
}
#define XSENS_SetHeadingAck_ID 0x83

#define XSENS_ReqMagneticDeclination_ID 0x6A
#define XSENS_ReqMagneticDeclination() { \
  XsensHeader(XSENS_ReqMagneticDeclination_ID, 0);\
  XsensTrailer();\
}
#define XSENS_ReqMagneticDeclinationAck_ID 0x6B
#define XSENS_ReqMagneticDeclinationAck_declination(_xsens_payload) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+0)|*((uint8_t*)_xsens_payload+2+0)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0))<<24); _f.f; })

#define XSENS_SetMagneticDeclination_ID 0x6A
#define XSENS_SetMagneticDeclination(declination) { \
  XsensHeader(XSENS_SetMagneticDeclination_ID, 4);\
  float _declination = declination; XsensSend4ByAddr((uint8_t*)&_declination);\
  XsensTrailer();\
}
#define XSENS_SetMagneticDeclinationAck_ID 0x6B

#define XSENS_ReqGPSStatus_ID 0xA6
#define XSENS_ReqGPSStatus() { \
  XsensHeader(XSENS_ReqGPSStatus_ID, 0);\
  XsensTrailer();\
}
#define XSENS_GPSStatus_ID 0xA7
#define XSENS_GPSStatus_nch(_xsens_payload) (uint8_t)(*((uint8_t*)_xsens_payload+0))
#define XSENS_GPSStatus_chn(_xsens_payload,_xsens_block) (uint8_t)(*((uint8_t*)_xsens_payload+1+5*_xsens_block))
#define XSENS_GPSStatus_svid(_xsens_payload,_xsens_block) (uint8_t)(*((uint8_t*)_xsens_payload+2+5*_xsens_block))
#define XSENS_GPSStatus_bitmask(_xsens_payload,_xsens_block) (uint8_t)(*((uint8_t*)_xsens_payload+3+5*_xsens_block))
#define XSENS_GPSStatus_qi(_xsens_payload,_xsens_block) (int8_t)(*((uint8_t*)_xsens_payload+4+5*_xsens_block))
#define XSENS_GPSStatus_cnr(_xsens_payload,_xsens_block) (int8_t)(*((uint8_t*)_xsens_payload+5+5*_xsens_block))

#define XSENS_ReqUTCTime_ID 0x60
#define XSENS_ReqUTCTime() { \
  XsensHeader(XSENS_ReqUTCTime_ID, 0);\
  XsensTrailer();\
}
#define XSENS_UTCTime_ID 0x61
#define XSENS_UTCTime_time(_xsens_payload) (uint32_t)(*((uint8_t*)_xsens_payload+3+0)|*((uint8_t*)_xsens_payload+2+0)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0))<<24)

#define XSENS_DATA_RAWInertial_LENGTH 20
#define XSENS_DATA_RAWInertial_accX(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+0+1*_xsens_block)<<8)
#define XSENS_DATA_RAWInertial_accY(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+2+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+1*_xsens_block)<<8)
#define XSENS_DATA_RAWInertial_accZ(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+4+1*_xsens_block)|*((uint8_t*)_xsens_payload+4+1*_xsens_block)<<8)
#define XSENS_DATA_RAWInertial_gyrX(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+6+1*_xsens_block)|*((uint8_t*)_xsens_payload+6+1*_xsens_block)<<8)
#define XSENS_DATA_RAWInertial_gyrY(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+8+1*_xsens_block)|*((uint8_t*)_xsens_payload+8+1*_xsens_block)<<8)
#define XSENS_DATA_RAWInertial_gyrZ(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+10+1*_xsens_block)|*((uint8_t*)_xsens_payload+10+1*_xsens_block)<<8)
#define XSENS_DATA_RAWInertial_magX(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+12+1*_xsens_block)|*((uint8_t*)_xsens_payload+12+1*_xsens_block)<<8)
#define XSENS_DATA_RAWInertial_magY(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+14+1*_xsens_block)|*((uint8_t*)_xsens_payload+14+1*_xsens_block)<<8)
#define XSENS_DATA_RAWInertial_magZ(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+16+1*_xsens_block)|*((uint8_t*)_xsens_payload+16+1*_xsens_block)<<8)
#define XSENS_DATA_RAWInertial_temp(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+18+1*_xsens_block)|*((uint8_t*)_xsens_payload+18+1*_xsens_block)<<8)

#define XSENS_DATA_RAWGPS_LENGTH 44
#define XSENS_DATA_RAWGPS_press(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+0+1*_xsens_block)<<8)
#define XSENS_DATA_RAWGPS_bPrs(_xsens_payload,_xsens_block) (uint8_t)(*((uint8_t*)_xsens_payload+2+1*_xsens_block))
#define XSENS_DATA_RAWGPS_itow(_xsens_payload,_xsens_block) (uint32_t)(*((uint8_t*)_xsens_payload+3+3+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+3+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+3+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+3+1*_xsens_block))<<24)
#define XSENS_DATA_RAWGPS_lat(_xsens_payload,_xsens_block) (int32_t)(*((uint8_t*)_xsens_payload+3+7+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+7+1*_xsens_block)<<8|((int32_t)*((uint8_t*)_xsens_payload+1+7+1*_xsens_block))<<16|((int32_t)*((uint8_t*)_xsens_payload+7+1*_xsens_block))<<24)
#define XSENS_DATA_RAWGPS_lon(_xsens_payload,_xsens_block) (int32_t)(*((uint8_t*)_xsens_payload+3+11+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+11+1*_xsens_block)<<8|((int32_t)*((uint8_t*)_xsens_payload+1+11+1*_xsens_block))<<16|((int32_t)*((uint8_t*)_xsens_payload+11+1*_xsens_block))<<24)
#define XSENS_DATA_RAWGPS_alt(_xsens_payload,_xsens_block) (int32_t)(*((uint8_t*)_xsens_payload+3+15+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+15+1*_xsens_block)<<8|((int32_t)*((uint8_t*)_xsens_payload+1+15+1*_xsens_block))<<16|((int32_t)*((uint8_t*)_xsens_payload+15+1*_xsens_block))<<24)
#define XSENS_DATA_RAWGPS_vel_n(_xsens_payload,_xsens_block) (int32_t)(*((uint8_t*)_xsens_payload+3+19+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+19+1*_xsens_block)<<8|((int32_t)*((uint8_t*)_xsens_payload+1+19+1*_xsens_block))<<16|((int32_t)*((uint8_t*)_xsens_payload+19+1*_xsens_block))<<24)
#define XSENS_DATA_RAWGPS_vel_e(_xsens_payload,_xsens_block) (int32_t)(*((uint8_t*)_xsens_payload+3+23+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+23+1*_xsens_block)<<8|((int32_t)*((uint8_t*)_xsens_payload+1+23+1*_xsens_block))<<16|((int32_t)*((uint8_t*)_xsens_payload+23+1*_xsens_block))<<24)
#define XSENS_DATA_RAWGPS_vel_d(_xsens_payload,_xsens_block) (int32_t)(*((uint8_t*)_xsens_payload+3+27+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+27+1*_xsens_block)<<8|((int32_t)*((uint8_t*)_xsens_payload+1+27+1*_xsens_block))<<16|((int32_t)*((uint8_t*)_xsens_payload+27+1*_xsens_block))<<24)
#define XSENS_DATA_RAWGPS_hacc(_xsens_payload,_xsens_block) (uint32_t)(*((uint8_t*)_xsens_payload+3+31+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+31+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+31+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+31+1*_xsens_block))<<24)
#define XSENS_DATA_RAWGPS_vacc(_xsens_payload,_xsens_block) (uint32_t)(*((uint8_t*)_xsens_payload+3+35+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+35+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+35+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+35+1*_xsens_block))<<24)
#define XSENS_DATA_RAWGPS_sacc(_xsens_payload,_xsens_block) (uint32_t)(*((uint8_t*)_xsens_payload+3+39+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+39+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+39+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+39+1*_xsens_block))<<24)
#define XSENS_DATA_RAWGPS_bGPS(_xsens_payload,_xsens_block) (uint8_t)(*((uint8_t*)_xsens_payload+43+1*_xsens_block))

#define XSENS_DATA_Temp_LENGTH 4
#define XSENS_DATA_Temp_temp(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+0+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0+1*_xsens_block))<<24); _f.f; })

#define XSENS_DATA_Calibrated_LENGTH 36
#define XSENS_DATA_Calibrated_accX(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+0+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Calibrated_accY(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+4+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+4+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+4+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+4+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Calibrated_accZ(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+8+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+8+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+8+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+8+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Calibrated_gyrX(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+12+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+12+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+12+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+12+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Calibrated_gyrY(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+16+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+16+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+16+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+16+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Calibrated_gyrZ(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+20+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+20+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+20+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+20+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Calibrated_magX(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+24+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+24+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+24+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+24+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Calibrated_magY(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+28+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+28+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+28+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+28+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Calibrated_magZ(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+32+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+32+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+32+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+32+1*_xsens_block))<<24); _f.f; })

#define XSENS_DATA_Quaternion_LENGTH 16
#define XSENS_DATA_Quaternion_q0(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+0+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Quaternion_q1(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+4+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+4+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+4+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+4+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Quaternion_q2(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+8+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+8+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+8+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+8+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Quaternion_q3(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+12+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+12+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+12+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+12+1*_xsens_block))<<24); _f.f; })

#define XSENS_DATA_Euler_LENGTH 12
#define XSENS_DATA_Euler_roll(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+0+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Euler_pitch(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+4+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+4+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+4+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+4+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Euler_yaw(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+8+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+8+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+8+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+8+1*_xsens_block))<<24); _f.f; })

#define XSENS_DATA_Matrix_LENGTH 36
#define XSENS_DATA_Matrix_a(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+0+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Matrix_b(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+4+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+4+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+4+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+4+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Matrix_c(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+8+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+8+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+8+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+8+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Matrix_d(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+12+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+12+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+12+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+12+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Matrix_e(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+16+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+16+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+16+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+16+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Matrix_f(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+20+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+20+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+20+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+20+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Matrix_g(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+24+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+24+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+24+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+24+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Matrix_h(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+28+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+28+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+28+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+28+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Matrix_i(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+32+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+32+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+32+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+32+1*_xsens_block))<<24); _f.f; })

#define XSENS_DATA_Auxiliary_LENGTH 4
#define XSENS_DATA_Auxiliary_ain_1(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+0+1*_xsens_block)<<8)
#define XSENS_DATA_Auxiliary_ain_2(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+2+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+1*_xsens_block)<<8)

#define XSENS_DATA_Position_LENGTH 12
#define XSENS_DATA_Position_lat(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+0+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Position_lon(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+4+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+4+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+4+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+4+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Position_alt(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+8+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+8+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+8+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+8+1*_xsens_block))<<24); _f.f; })

#define XSENS_DATA_Velocity_LENGTH 12
#define XSENS_DATA_Velocity_vx(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+0+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+0+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+0+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Velocity_vy(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+4+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+4+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+4+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+4+1*_xsens_block))<<24); _f.f; })
#define XSENS_DATA_Velocity_vz(_xsens_payload,_xsens_block) ({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+8+1*_xsens_block)|*((uint8_t*)_xsens_payload+2+8+1*_xsens_block)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+8+1*_xsens_block))<<16|((uint32_t)*((uint8_t*)_xsens_payload+8+1*_xsens_block))<<24); _f.f; })

#define XSENS_DATA_Status_LENGTH 1
#define XSENS_DATA_Status_status(_xsens_payload,_xsens_block) (uint8_t)(*((uint8_t*)_xsens_payload+0+1*_xsens_block))

#define XSENS_DATA_TimeStamp_LENGTH 2
#define XSENS_DATA_TimeStamp_ts(_xsens_payload,_xsens_block) (uint16_t)(*((uint8_t*)_xsens_payload+1+0+1*_xsens_block)|*((uint8_t*)_xsens_payload+0+1*_xsens_block)<<8)

#define XSENS_MASK_Temp(_conf) ((_conf & 0x0001))

#define XSENS_MASK_Calibrated(_conf) ((_conf & 0x0002))

#define XSENS_MASK_Orientation(_conf) ((_conf & 0x0004))

#define XSENS_MASK_Auxiliary(_conf) ((_conf & 0x0008))

#define XSENS_MASK_Position(_conf) ((_conf & 0x0010))

#define XSENS_MASK_Velocity(_conf) ((_conf & 0x0020))

#define XSENS_MASK_Status(_conf) ((_conf & 0x0800))

#define XSENS_MASK_RAWGPS(_conf) ((_conf & 0x1000))

#define XSENS_MASK_RAWInertial(_conf) ((_conf & 0x4000))

#define XSENS_MASK_TimeStamp(_conf) ((_conf & 0x00000003))

#define XSENS_MASK_OrientationMode(_conf) ((_conf & 0x00000006)>>2)

#define XSENS_MASK_AccOut(_conf) ((_conf & 0x00000010))

#define XSENS_MASK_GyrOut(_conf) ((_conf & 0x00000020))

#define XSENS_MASK_MagOut(_conf) ((_conf & 0x00000040))

#define XSENS_MASK_Format(_conf) ((_conf & 0x00000300)>>8)

#define XSENS_MASK_Aux1Out(_conf) ((_conf & 0x00000400))

#define XSENS_MASK_Aux2Out(_conf) ((_conf & 0x00000800))

#define XSENS_MASK_LLA(_conf) ((_conf & 0x0001C000))

#define XSENS_MASK_Vel(_conf) ((_conf & 0x00060000))

#define XSENS_MASK_Convention(_conf) ((_conf & 0x80000000))
