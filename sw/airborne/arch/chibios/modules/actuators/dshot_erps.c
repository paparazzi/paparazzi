#include "modules/actuators/dshot_erps.h"
//#include "stdutil.h"


/**
 * @brief   IBM GCR encoding lookup table
 */
static const uint8_t gcrNibble[16] = {
  0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
  0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F};

/**
 * @brief   IBM GCR decoding lookup table
 */
static const uint8_t gcrNibbleInv[32] = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0x9, 0xa, 0xb, 0xff, 0xd, 0xe, 0xf, 
  0xff, 0xff, 0x2, 0x3, 0xff, 0x5, 0x6, 0x7, 
  0xff, 0x0, 0x8, 0x1, 0xff, 0x4, 0xc, 0xff};

static uint8_t crc4(uint16_t val);
static DshotEPeriodPacket eperiodToPacked(const uint32_t eperiod);
static uint32_t greyEncode(uint32_t num);
static uint32_t greyDecode(const uint32_t num);
static uint32_t gcrEncode(uint32_t from);
static uint32_t gcrDecode(uint32_t from);
static uint32_t eperiodEncode(const uint16_t eperiod);
static uint32_t eperiodDecode(const uint32_t frame);



static  void setFromEperiod(DshotErps *derpsp, uint32_t eperiod);
static  void frameToPacket(DshotErps *derpsp);
static  void packetToFrame(DshotErps *derpsp);



/**
 * @brief   initialise from GCR encoded frame
 *
 * @param[in] derpsp    pointer to the @p DshotErps object
 * @api
 */
const DshotErps* DshotErpsSetFromFrame(DshotErps *derpsp, uint32_t frame)
{
  derpsp->ef = frame;
  frameToPacket(derpsp);
  return derpsp;
}

/**
 * @brief   initialise from rpm value
 *
 * @param[in] derpsp    pointer to the @p DshotErps object
 * @api
 */
const DshotErps* DshotErpsSetFromRpm(DshotErps *derpsp, uint32_t rpm)
{
  uint32_t eperiod = ((uint32_t) 60e6f) / rpm;
  setFromEperiod(derpsp, eperiod);
  return derpsp;
}

/**
 * @brief   return eperiod from mantisse and exponent
 *
 * @param[in] derpsp    pointer to the @p DshotErps object
 * @return eperiod in microseconds
 * @note   getEperiod avoid a division and is more cpu friendly than getRpm
 * @private
 */
uint32_t DshotErpsGetEperiod(const DshotErps *derpsp)
{
  return derpsp->ep.mantisse << derpsp->ep.exponent;
}


/**
 * @brief   calculate and return rpm
 *
 * @param[in] derpsp    pointer to the @p DshotErps object
 * @return    rotational speed in RPM
 * @note      involve a division, which is a cpu cycle hog, If you can
              use getEperiod instead, it will be less calculus intensive
 * @api
 */
uint32_t DshotErpsGetRpm(const DshotErps *derpsp)
{
  return ((uint32_t) 60e6f) / DshotErpsGetEperiod(derpsp);
}

/**
 * @brief   check packed validity
 *
 * @param[in] derpsp    pointer to the @p DshotErps object
 * @return    true if CRC is OK
 * @api
 */
bool DshotErpsCheckCrc4(const DshotErps *derpsp)
{
   return (crc4(derpsp->ep.rawFrame) == derpsp->ep.crc);
}

/**
 * @brief   calculate crc4
 *
 * @param[in] val
 * @return    true if CRC is OK
 * @private
 */
static uint8_t crc4(uint16_t val)
{
  val >>= 4;
  return (~(val ^ (val >> 4) ^ (val >> 8))) & 0x0F;
}

/**
 * @brief   encode packet from eperiod
 *
 * @param[in] eperiod in microseconds
 * @return    encoded DshotEPeriodPacket
 * @private
 */
static DshotEPeriodPacket eperiodToPacked(const uint32_t eperiod)
{
  DshotEPeriodPacket p;
  const uint32_t exponent = eperiod >> 9;
  
  if (exponent > 128) {
    p = (DshotEPeriodPacket) {.crc = 0, .mantisse = 511, .exponent = 7};
  } else {
    p.exponent = 32U - __builtin_clz(exponent);
    p.mantisse = eperiod >> p.exponent;
  }
  p.crc = crc4(p.rawFrame);
  return p;
}

/**
 * @brief   get grey binary value from natural binary
 *
 * @param[in] num natural binary value
 * @return    grey encoded value
 * @private
 */
static uint32_t greyEncode(uint32_t num)
{
  num ^= (num >> 16);
  num ^= (num >>  8);
  num ^= (num >>  4);
  num ^= (num >>  2);
  num ^= (num >>  1);
  return num;
}

/**
 * @brief   get natural binary value from grey binary
 *
 * @param[in] grey encoded value
 * @return    num natural binary value
 * @private
 */
static uint32_t greyDecode(const uint32_t num)
{
  return num ^ (num >> 1);
}

/**
 * @brief   encode 16 bit value to 20 bits GCR
 *
 * @param[in] binary value
 * @return    GCR(0,2) encoded value
 * @private
 */
static uint32_t gcrEncode(uint32_t from)
{
  uint32_t ret = 0;
  for (size_t i = 0U; i < 4U; i++) {
    //   printf("nibble %u from = 0x%x to = 0x%x\n", 
    //   i, from & 0xf, gcrNibble[from & 0xf]);
    ret |= (gcrNibble[from & 0xf] << (i*5));
    from >>= 4U;
  }
  return ret;
}

/**
 * @brief   decode 20 bits GCR value to 16 bits natural
 *
 * @param[in] GCR(0,2) encoded value
 * @return    binary value
 * @private
 */
static uint32_t gcrDecode(uint32_t from)
{
  uint32_t ret = 0;
  for (size_t i = 0; i < 4U; i++) {
    const uint32_t nibble = gcrNibbleInv[from & 0x1f];
    if (nibble == 0xff) {
      ret = 0x0;
      break;
    }
    //   printf("nibble %u from = 0x%x to = 0x%x\n", 
    //   i, from & 0xf, gcrNibble[from & 0xf]);
    ret |= (nibble << (i << 2U));
    from >>= 5U;
  }
  return ret;
}

/**
 * @brief   encode eperiod to 20 bits GCR of grey value
 *
 * @param[in] eperiod in microseconds
 * @return    GCR(0,2) encoded value ready to be transmitted
 * @private
 */
static uint32_t eperiodEncode(const uint16_t eperiod)
{
  return greyEncode(gcrEncode(eperiod));
}

/**
 * @brief   decode 20 bits GCR of grey value to eperiod
 *
 * @param[in] GCR(0,2) encoded value received from ESC
 * @return    eperiod in microseconds
 * @private
 */
static uint32_t eperiodDecode(const uint32_t frame)
{
  return gcrDecode(greyDecode(frame));
}

/**
 * @brief   initialize from eperiod in microseconds
 *
 * @param[in] derpsp    pointer to the @p DshotErps object
 * @param[in] eperiod in microseconds
 * @private
 */
static  void setFromEperiod(DshotErps *derpsp, uint32_t eperiod)
{
  derpsp->ep = eperiodToPacked(eperiod);
  packetToFrame(derpsp);
}

/**
 * @brief   decode eperiod
 *
 * @param[in out] derpsp    pointer to the @p DshotErps object
 * @private
 */
static  void frameToPacket(DshotErps *derpsp)
{
  derpsp->ep.rawFrame = eperiodDecode(derpsp->ef);
  //  DebugTrace("DBG> 0x%lx => 0x%x", derpsp->ef, derpsp->ep.rawFrame);
}

/**
 * @brief   encode eperiod
 *
 * @param[in out] derpsp    pointer to the @p DshotErps object
 * @private
 */
static  void packetToFrame(DshotErps *derpsp)
{
  derpsp->ef = eperiodEncode(derpsp->ep.rawFrame);
}

