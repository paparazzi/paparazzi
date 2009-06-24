#include "props_csc.h"
#include "airframe.h"
#include "csc_ap_link.h"
#include "csc_msg_def.h"

uint8_t csc_prop_speeds[PROPS_NB];

void props_init()
{
  for(uint8_t i = 0; i < PROPS_NB; i++)
    csc_prop_speeds[i] = 0;

}

void props_set(uint8_t i,uint8_t speed)
{
  if(i > PROPS_NB) return;
  csc_prop_speeds[i] = speed;
}

void props_commit()
{
  struct CscPropCmd cmd;
  uint8_t i;

  for(i = 0; i < PROPS_NB; i++)
    cmd.speeds[i] = csc_prop_speeds[i];

  csc_ap_send_msg(CSC_PROP_CMD_ID, (uint8_t *)&cmd, sizeof(struct CscPropCmd));

}
