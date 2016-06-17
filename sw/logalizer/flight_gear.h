#ifndef FLIGHT_GEAR_H
#define FLIGHT_GEAR_H

#include <stdint.h>
#include "../simulator/flight_gear.h"


#define MP_MAX_CALLSIGN_LEN        8
#define MP_MAX_MODEL_NAME_LEN      96
#define MP_MSG_MAGIC  0x46474653
#define MP_PROTO_VER  0x00010001

struct FGMplayHdr {
  uint32_t magic;
  uint32_t version;
  uint32_t id;
  uint32_t len;
  uint32_t reply_addr;
  uint32_t reply_port;
  char callsign[MP_MAX_CALLSIGN_LEN];
};

struct FGMplayPosMsg {
  char model[MP_MAX_MODEL_NAME_LEN];
  double time;
  double lag;
  double position[3];
  float orientation[4];
  float linear_vel[3];
  float angular_vel[3];
  float linear_accel[3];
  float angular_accel[3];
};

struct FGMplayMsg {
  struct FGMplayHdr header;
  struct FGMplayPosMsg pos;
};

extern void mplay_msg_dump ( struct FGMplayMsg *msg );
extern void mplay_msg_ntoh ( struct FGMplayMsg* msg );
extern void mplay_msg_init ( struct FGMplayMsg* msg );
#endif /* FLIGHT_GEAR_H */
