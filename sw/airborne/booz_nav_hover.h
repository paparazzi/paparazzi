#ifndef BOOZ_NAV_HOVER_H
#define BOOZ_NAV_HOVER_H


#ifndef DISABLE_NAV
extern float booz_nav_hover_x_sp;
extern float booz_nav_hover_y_sp;
extern float booz_nav_hover_z_sp;
extern float booz_nav_hover_psi_sp;

extern float booz_nav_hover_h_max_err;
extern float booz_nav_hover_h_pgain;
extern float booz_nav_hover_h_dgain;
extern float booz_nav_hover_v_max_err;
extern float booz_nav_hover_v_pgain;
extern float booz_nav_hover_v_dgain;

extern float booz_nav_hover_phi_command;
extern float booz_nav_hover_theta_command;
extern float booz_nav_hover_power_command;
#endif

extern void booz_nav_hover_init(void);
extern void booz_nav_hover_run(void);
extern void booz_nav_hover_read_setpoints_from_rc(void);


#endif /* BOOZ_NAV_HOVER_H */
