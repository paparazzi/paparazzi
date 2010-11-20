#ifndef BOMB_H
#define BOMB_H

#define MY_BOMB_RADIUS DEFAULT_CIRCLE_RADIUS

extern unit_t bomb_compute_approach( uint8_t wp_target, uint8_t wp_start, float radius );
extern unit_t bomb_update_release( uint8_t wp_target );
extern unit_t bomb_shoot( void );
extern float bomb_trigger_delay, bomb_start_qdr;
extern bool_t compute_alignment(uint8_t w1, uint8_t w2, uint8_t start, uint8_t end, float d_before, float d_after);


#define BombComputeApproach(_target, _start, _radius) bomb_compute_approach(_target, _start, _radius)
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
