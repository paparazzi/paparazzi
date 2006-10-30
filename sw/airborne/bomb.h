#ifndef BOMB_H
#define BOMB_H

#define BOMB_RADIUS 100

extern unit_t bomb_compute_approach( void );
extern unit_t bomb_update_release( void );
extern unit_t bomb_shoot( void );
extern float bomb_trigger_delay, bomb_start_qdr;


#define BombComputeApproach() bomb_compute_approach()
#define BombUpdateRelease() bomb_update_release()
#define BombReadyToShoot() bomb_ready_to_shoot()
#define BombShoot() bomb_shoot()
#define BombCloseHatch() ({ ap_state->commands[COMMAND_HATCH] = MIN_PPRZ; })
#define BombAligned() Qdr(DegOfRad(bomb_qdr_aligned))


extern bool_t compute_tod( void );
extern unit_t compute_baseleg( void );
extern float baseleg_alt, downwind_altitude;
extern const float baseleg_alt_tolerance;

#endif
