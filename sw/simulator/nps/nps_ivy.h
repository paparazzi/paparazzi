#ifndef NPS_IVY
#define NPS_IVY

extern void nps_ivy_common_init(char *ivy_bus);
extern void nps_ivy_init(char *ivy_bus);
extern void nps_ivy_display(void);

#ifdef USE_MISSION_COMMANDS_IN_NPS
extern void nps_ivy_mission_commands_init(void);
#endif

#endif /* NPS_IVY */
