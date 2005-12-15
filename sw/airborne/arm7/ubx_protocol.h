/* Generated from conf/ubx.xml */
/* Please DO NOT EDIT */

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

#define UBX_NAV_ID 0x01

#define UBX_NAV_POSLLH_ID 0x02
#define UBX_NAV_POSLLH_ITOW(_ubx_payload) (*((uint32_t*)(_ubx_payload+0)))
#define UBX_NAV_POSLLH_LON(_ubx_payload) (*((int32_t*)(_ubx_payload+4)))
#define UBX_NAV_POSLLH_LAT(_ubx_payload) (*((int32_t*)(_ubx_payload+8)))
#define UBX_NAV_POSLLH_HEIGHT(_ubx_payload) (*((int32_t*)(_ubx_payload+12)))
#define UBX_NAV_POSLLH_HMSL(_ubx_payload) (*((int32_t*)(_ubx_payload+16)))
#define UBX_NAV_POSLLH_Hacc(_ubx_payload) (*((uint32_t*)(_ubx_payload+20)))
#define UBX_NAV_POSLLH_Vacc(_ubx_payload) (*((uint32_t*)(_ubx_payload+24)))

#define UBX_NAV_POSUTM_ID 0x08
#define UBX_NAV_POSUTM_ITOW(_ubx_payload) (*((uint32_t*)(_ubx_payload+0)))
#define UBX_NAV_POSUTM_EAST(_ubx_payload) (*((int32_t*)(_ubx_payload+4)))
#define UBX_NAV_POSUTM_NORTH(_ubx_payload) (*((int32_t*)(_ubx_payload+8)))
#define UBX_NAV_POSUTM_ALT(_ubx_payload) (*((int32_t*)(_ubx_payload+12)))
#define UBX_NAV_POSUTM_ZONE(_ubx_payload) (*((int8_t*)(_ubx_payload+16)))
#define UBX_NAV_POSUTM_HEM(_ubx_payload) (*((int8_t*)(_ubx_payload+17)))

#define UBX_NAV_STATUS_ID 0x03
#define UBX_NAV_STATUS_ITOW(_ubx_payload) (*((uint32_t*)(_ubx_payload+0)))
#define UBX_NAV_STATUS_GPSfix(_ubx_payload) (*((uint8_t*)(_ubx_payload+4)))
#define UBX_NAV_STATUS_Flags(_ubx_payload) (*((uint8_t*)(_ubx_payload+5)))
#define UBX_NAV_STATUS_DiffS(_ubx_payload) (*((uint8_t*)(_ubx_payload+6)))
#define UBX_NAV_STATUS_res(_ubx_payload) (*((uint8_t*)(_ubx_payload+7)))
#define UBX_NAV_STATUS_TTFF(_ubx_payload) (*((uint32_t*)(_ubx_payload+8)))
#define UBX_NAV_STATUS_MSSS(_ubx_payload) (*((uint32_t*)(_ubx_payload+12)))

#define UBX_NAV_VELNED_ID 0x12
#define UBX_NAV_VELNED_ITOW(_ubx_payload) (*((uint32_t*)(_ubx_payload+0)))
#define UBX_NAV_VELNED_VEL_N(_ubx_payload) (*((int32_t*)(_ubx_payload+4)))
#define UBX_NAV_VELNED_VEL_E(_ubx_payload) (*((int32_t*)(_ubx_payload+8)))
#define UBX_NAV_VELNED_VEL_D(_ubx_payload) (*((int32_t*)(_ubx_payload+12)))
#define UBX_NAV_VELNED_Speed(_ubx_payload) (*((uint32_t*)(_ubx_payload+16)))
#define UBX_NAV_VELNED_GSpeed(_ubx_payload) (*((uint32_t*)(_ubx_payload+20)))
#define UBX_NAV_VELNED_Heading(_ubx_payload) (*((int32_t*)(_ubx_payload+24)))
#define UBX_NAV_VELNED_SAcc(_ubx_payload) (*((uint32_t*)(_ubx_payload+28)))
#define UBX_NAV_VELNED_CAcc(_ubx_payload) (*((uint32_t*)(_ubx_payload+32)))

#define UBX_NAV_SVINFO_ID 0x30
#define UBX_NAV_SVINFO_ITOW(_ubx_payload) (*((uint32_t*)(_ubx_payload+0)))
#define UBX_NAV_SVINFO_NCH(_ubx_payload) (*((uint8_t*)(_ubx_payload+4)))
#define UBX_NAV_SVINFO_RES1(_ubx_payload) (*((uint8_t*)(_ubx_payload+5)))
#define UBX_NAV_SVINFO_RES2(_ubx_payload) (*((uint16_t*)(_ubx_payload+6)))
#define UBX_NAV_SVINFO_chn(_ubx_payload,_ubx_block) (*((uint8_t*)(_ubx_payload+8+12*_ubx_block)))
#define UBX_NAV_SVINFO_SVID(_ubx_payload,_ubx_block) (*((uint8_t*)(_ubx_payload+9+12*_ubx_block)))
#define UBX_NAV_SVINFO_Flags(_ubx_payload,_ubx_block) (*((uint8_t*)(_ubx_payload+10+12*_ubx_block)))
#define UBX_NAV_SVINFO_QI(_ubx_payload,_ubx_block) (*((int8_t*)(_ubx_payload+11+12*_ubx_block)))
#define UBX_NAV_SVINFO_CNO(_ubx_payload,_ubx_block) (*((uint8_t*)(_ubx_payload+12+12*_ubx_block)))
#define UBX_NAV_SVINFO_Elev(_ubx_payload,_ubx_block) (*((int8_t*)(_ubx_payload+13+12*_ubx_block)))
#define UBX_NAV_SVINFO_Azim(_ubx_payload,_ubx_block) (*((int16_t*)(_ubx_payload+14+12*_ubx_block)))
#define UBX_NAV_SVINFO_PRRes(_ubx_payload,_ubx_block) (*((int32_t*)(_ubx_payload+16+12*_ubx_block)))
