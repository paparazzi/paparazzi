#define IGNORED 0
#define RESERVED 0

static bool_t user_gps_configure(bool_t cpt) {
  switch (cpt) {
  case 0:
    /* mask: we set only fixMode and dyn */
    UbxSend_CFG_NAV5(0x05, NAV5_DYN_AIRBORNE_2G, NAV5_3D_ONLY, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, IGNORED, RESERVED, RESERVED, RESERVED, RESERVED);
    break;
  case 1:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_POSLLH_ID, 1, 0, 0, 0);
    break;
  case 2:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_VELNED_ID, 1, 0, 0, 0);
    break;
  case 3:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_SVINFO_ID, 4, 0, 0, 0);
    break;
  case 4:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_SOL_ID, 8, 0, 0, 0);
    break;
  case 5:
    UbxSend_CFG_RATE(250 /*ms*/, 0x0001, 0x0000);
    return FALSE;
  }
  return TRUE; /* Continue, except for the last case */
}
