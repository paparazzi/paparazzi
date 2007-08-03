#ifndef BOMB_H
#define BOMB_H

#define BOMB_RADIUS 100

extern unit_t bomb_compute_approach( uint8_t wp_target, uint8_t wp_start );
extern unit_t bomb_update_release( uint8_t wp_target );
extern unit_t bomb_shoot( void );
extern float bomb_trigger_delay, bomb_start_qdr;


#define BombComputeApproach(_target, _start) bomb_compute_approach(_target, _start)
#define BombUpdateRelease(_wp) bomb_update_release(_wp)
#define BombReadyToShoot() bomb_ready_to_shoot()
#define BombShoot() bomb_shoot()
#define BombCloseHatch() ({ ap_state->commands[COMMAND_HATCH] = MIN_PPRZ; })
#define BombAligned() Qdr(DegOfRad(bomb_qdr_aligned))


extern bool_t compute_tod( void );
extern unit_t compute_baseleg( void );
extern float baseleg_alt, downwind_altitude;
extern const float baseleg_alt_tolerance;

#endif
