#ifndef BOOZ_NAV_H
#define BOOZ_NAV_H


#ifndef DISABLE_NAV
extern float booz_nav_horizontal_x_sp;
extern float booz_nav_horizontal_y_sp;
extern float booz_nav_horizontal_pgain;
extern float booz_nav_horizontal_dgain;
extern float booz_nav_phi_command;
extern float booz_nav_theta_command;

extern float booz_nav_vertical_z_sp;
extern float booz_nav_vertical_pgain;
extern float booz_nav_vertical_dgain;
extern float booz_nav_power_command;
#endif

extern void booz_nav_init(void);
extern void booz_nav_run(void);
extern void booz_nav_read_setpoints_from_rc(void);


#endif /* BOOZ_NAV_H */
