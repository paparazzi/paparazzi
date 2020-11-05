#if !defined(UAV_RECOVERY_H)
#define UAV_RECOVERY_H
#define MY_PARACHUTE_RADIUS DEFAULT_CIRCLE_RADIUS

extern unit_t parachute_compute_approach(uint8_t baseleg, uint8_t release, uint8_t wp_target);
extern float  parachute_start_qdr;
extern bool   deploy_parachute_var;
extern float  airborne_wind_dir;
extern float  airborne_wind_speed;
extern bool   wind_measurements_valid;
extern bool   wind_info_valid;

extern void uav_recovery_init(void);
extern void uav_recovery_periodic(void);
extern uint8_t DeployParachute(void);
extern uint8_t LockParachute(void);
extern uint8_t calculate_wind_no_airspeed(uint8_t wp, float radius, float height);

#ifndef CONCAT1
#define CONCAT1(a, b)  (a##b)
#endif
#ifndef CONCAT
#define CONCAT(a, b)  CONCAT1(a, b)
#endif

#define ParachuteComputeApproach(_baseleg, _release, _target) parachute_compute_approach(_baseleg, _release, _target)

#endif
