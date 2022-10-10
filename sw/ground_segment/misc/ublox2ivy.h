/*
 * Paparazzi UBLox to Ivy
 *
 * Copyright (C) 2021 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#ifndef UBLOX2IVY_H
#define UBLOX2IVY_H

/* Parser status */
#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_CLASS     3
#define GOT_ID        4
#define GOT_LEN1      5
#define GOT_LEN2      6
#define GOT_PAYLOAD   7
#define GOT_CHECKSUM1 8

/* UBX / RTCM parsing */
#define GPS_RTCM_SYNC1            0xD3
#define GPS_RTCM_MAX_PAYLOAD      1024+6
#define GPS_UBX_SYNC1             0xB5
#define GPS_UBX_SYNC2             0x62
#define GPS_UBX_MAX_PAYLOAD       1024

/* UBX PVT message */
#define UBX_NAV_PVT_ID 0x07
#define UBX_NAV_PVT_iTOW(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+0)|(uint32_t)(*((uint8_t*)_ubx_payload+1+0))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+0))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+0))<<24)
#define UBX_NAV_PVT_year(_ubx_payload) (uint16_t)(*((uint8_t*)_ubx_payload+4)|(uint16_t)(*((uint8_t*)_ubx_payload+1+4))<<8)
#define UBX_NAV_PVT_month(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+6))
#define UBX_NAV_PVT_day(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+7))
#define UBX_NAV_PVT_hour(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+8))
#define UBX_NAV_PVT_min(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+9))
#define UBX_NAV_PVT_sec(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+10))
#define UBX_NAV_PVT_valid(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+11))
#define UBX_NAV_PVT_tAcc(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+12)|(uint32_t)(*((uint8_t*)_ubx_payload+1+12))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+12))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+12))<<24)
#define UBX_NAV_PVT_nano(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+16)|(int32_t)(*((uint8_t*)_ubx_payload+1+16))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+16))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+16))<<24)
#define UBX_NAV_PVT_fixType(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+20))
#define UBX_NAV_PVT_flags(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+21))
#define UBX_NAV_PVT_flags2(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+22))
#define UBX_NAV_PVT_numSV(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+23))
#define UBX_NAV_PVT_lon(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+24)|(int32_t)(*((uint8_t*)_ubx_payload+1+24))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+24))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+24))<<24)
#define UBX_NAV_PVT_lat(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+28)|(int32_t)(*((uint8_t*)_ubx_payload+1+28))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+28))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+28))<<24)
#define UBX_NAV_PVT_height(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+32)|(int32_t)(*((uint8_t*)_ubx_payload+1+32))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+32))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+32))<<24)
#define UBX_NAV_PVT_hMSL(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+36)|(int32_t)(*((uint8_t*)_ubx_payload+1+36))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+36))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+36))<<24)
#define UBX_NAV_PVT_hAcc(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+40)|(uint32_t)(*((uint8_t*)_ubx_payload+1+40))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+40))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+40))<<24)
#define UBX_NAV_PVT_vAcc(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+44)|(uint32_t)(*((uint8_t*)_ubx_payload+1+44))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+44))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+44))<<24)
#define UBX_NAV_PVT_velN(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+48)|(int32_t)(*((uint8_t*)_ubx_payload+1+48))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+48))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+48))<<24)
#define UBX_NAV_PVT_velE(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+52)|(int32_t)(*((uint8_t*)_ubx_payload+1+52))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+52))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+52))<<24)
#define UBX_NAV_PVT_velD(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+56)|(int32_t)(*((uint8_t*)_ubx_payload+1+56))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+56))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+56))<<24)
#define UBX_NAV_PVT_gSpeed(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+60)|(int32_t)(*((uint8_t*)_ubx_payload+1+60))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+60))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+60))<<24)
#define UBX_NAV_PVT_headMot(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+64)|(int32_t)(*((uint8_t*)_ubx_payload+1+64))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+64))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+64))<<24)
#define UBX_NAV_PVT_sAcc(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+68)|(uint32_t)(*((uint8_t*)_ubx_payload+1+68))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+68))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+68))<<24)
#define UBX_NAV_PVT_headAcc(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+72)|(uint32_t)(*((uint8_t*)_ubx_payload+1+72))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+72))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+72))<<24)
#define UBX_NAV_PVT_pDOP(_ubx_payload) (uint16_t)(*((uint8_t*)_ubx_payload+76)|(uint16_t)(*((uint8_t*)_ubx_payload+1+76))<<8)
#define UBX_NAV_PVT_reserved1a(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+78)|(uint32_t)(*((uint8_t*)_ubx_payload+1+78))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+78))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+78))<<24)
#define UBX_NAV_PVT_reserved1b(_ubx_payload) (uint16_t)(*((uint8_t*)_ubx_payload+82)|(uint16_t)(*((uint8_t*)_ubx_payload+1+82))<<8)
#define UBX_NAV_PVT_headVeh(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+84)|(int32_t)(*((uint8_t*)_ubx_payload+1+84))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+84))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+84))<<24)
#define UBX_NAV_PVT_magDec(_ubx_payload) (int16_t)(*((uint8_t*)_ubx_payload+88)|(int16_t)(*((uint8_t*)_ubx_payload+1+88))<<8)
#define UBX_NAV_PVT_magAcc(_ubx_payload) (uint16_t)(*((uint8_t*)_ubx_payload+90)|(uint16_t)(*((uint8_t*)_ubx_payload+1+90))<<8)

/* UBX RELPOSNED message */
#define UBX_NAV_RELPOSNED_ID 0x3C
#define UBX_NAV_RELPOSNED_version(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+0))
#define UBX_NAV_RELPOSNED_reserved1(_ubx_payload) (uint8_t)(*((uint8_t*)_ubx_payload+1))
#define UBX_NAV_RELPOSNED_refStationId(_ubx_payload) (uint16_t)(*((uint8_t*)_ubx_payload+2)|(uint16_t)(*((uint8_t*)_ubx_payload+1+2))<<8)
#define UBX_NAV_RELPOSNED_iTOW(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+4)|(uint32_t)(*((uint8_t*)_ubx_payload+1+4))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+4))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+4))<<24)
#define UBX_NAV_RELPOSNED_relPosN(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+8)|(int32_t)(*((uint8_t*)_ubx_payload+1+8))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+8))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+8))<<24)
#define UBX_NAV_RELPOSNED_relPosE(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+12)|(int32_t)(*((uint8_t*)_ubx_payload+1+12))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+12))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+12))<<24)
#define UBX_NAV_RELPOSNED_relPosD(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+16)|(int32_t)(*((uint8_t*)_ubx_payload+1+16))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+16))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+16))<<24)
#define UBX_NAV_RELPOSNED_relPosLength(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+20)|(int32_t)(*((uint8_t*)_ubx_payload+1+20))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+20))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+20))<<24)
#define UBX_NAV_RELPOSNED_relPosHeading(_ubx_payload) (int32_t)(*((uint8_t*)_ubx_payload+24)|(int32_t)(*((uint8_t*)_ubx_payload+1+24))<<8|((int32_t)*((uint8_t*)_ubx_payload+2+24))<<16|((int32_t)*((uint8_t*)_ubx_payload+3+24))<<24)
#define UBX_NAV_RELPOSNED_reserved2(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+28)|(uint32_t)(*((uint8_t*)_ubx_payload+1+28))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+28))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+28))<<24)
#define UBX_NAV_RELPOSNED_relPosHPN(_ubx_payload) (int8_t)(*((uint8_t*)_ubx_payload+32))
#define UBX_NAV_RELPOSNED_relPosHPE(_ubx_payload) (int8_t)(*((uint8_t*)_ubx_payload+33))
#define UBX_NAV_RELPOSNED_relPosHPD(_ubx_payload) (int8_t)(*((uint8_t*)_ubx_payload+34))
#define UBX_NAV_RELPOSNED_relPosHPLength(_ubx_payload) (int8_t)(*((uint8_t*)_ubx_payload+35))
#define UBX_NAV_RELPOSNED_accN(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+36)|(uint32_t)(*((uint8_t*)_ubx_payload+1+36))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+36))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+36))<<24)
#define UBX_NAV_RELPOSNED_accE(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+40)|(uint32_t)(*((uint8_t*)_ubx_payload+1+40))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+40))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+40))<<24)
#define UBX_NAV_RELPOSNED_accD(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+44)|(uint32_t)(*((uint8_t*)_ubx_payload+1+44))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+44))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+44))<<24)
#define UBX_NAV_RELPOSNED_accLength(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+48)|(uint32_t)(*((uint8_t*)_ubx_payload+1+48))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+48))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+48))<<24)
#define UBX_NAV_RELPOSNED_accHeading(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+52)|(uint32_t)(*((uint8_t*)_ubx_payload+1+52))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+52))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+52))<<24)
#define UBX_NAV_RELPOSNED_reserved3(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+56)|(uint32_t)(*((uint8_t*)_ubx_payload+1+56))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+56))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+56))<<24)
#define UBX_NAV_RELPOSNED_flags(_ubx_payload) (uint32_t)(*((uint8_t*)_ubx_payload+60)|(uint32_t)(*((uint8_t*)_ubx_payload+1+60))<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+60))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+60))<<24)

struct gps_ubx_t {
  bool msg_available;
  uint8_t msg_buf[GPS_UBX_MAX_PAYLOAD] __attribute__((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t status;
  uint16_t len;
  uint16_t msg_idx;
  uint8_t ck_a, ck_b;
  uint8_t error_cnt;
};

struct gps_rtcm_t {
  bool msg_available;
  uint8_t msg_buf[GPS_RTCM_MAX_PAYLOAD] __attribute__((aligned));

  uint8_t status;
  uint16_t len;
  uint16_t msg_idx;
  uint8_t error_cnt;
};

#endif /* UBLOX2IVY_H */