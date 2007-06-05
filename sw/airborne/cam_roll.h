#ifndef CAM_ROLL_H
#define CAM_ROLL_H

extern uint8_t cam_roll_mode;
extern float cam_roll_phi;
extern bool_t cam_roll_switch;

#define cam_roll_SetPhiDeg(_deg) { cam_roll_phi = RadOfDeg(_deg); }

#ifdef SITL
#define cam_roll_Switch(_x) { cam_roll_switch = _x; }
#else
#define cam_roll_Switch(_x) { cam_roll_switch = _x; if (_x) IO0SET = _BV(VIDEO_SWITCH_PIN); else IO0CLR = _BV(VIDEO_SWITCH_PIN); }
#endif


#endif /* CAM_ROLL_H */
