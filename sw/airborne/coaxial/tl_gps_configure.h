static bool_t user_gps_configure(bool_t cpt) {
  switch (cpt) {
  case 0:
    UbxSend_CFG_NAV(NAV_DYN_STATIONARY, 3, 16, 24, 20, 5, 0, 0x3C, 0x3C, 0x14, 0x03E8 ,0x0000, 0x0, 0x17, 0x00FA, 0x00FA, 0x0064, 0x012C, 0x000F, 0x00, 0x00);
    break;
  case 1:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_POSUTM_ID, 0, 1, 0, 0);
    break;
  case 2:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_VELNED_ID, 0, 1, 0, 0);
    break;
  case 3:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_STATUS_ID, 0, 1, 0, 0);
    break;
  case 4:
    UbxSend_CFG_MSG(UBX_NAV_ID, UBX_NAV_SVINFO_ID, 0, 4, 0, 0);
    break;
  case 5:
    UbxSend_CFG_SBAS(0x00, 0x00, 0x00, 0x00, 0x00);
    break;
  case 6:
    UbxSend_CFG_RATE(0x00FA, 0x0001, 0x0000);   
    return FALSE;
  }
  return TRUE; /* Continue, except for the last case */
}
