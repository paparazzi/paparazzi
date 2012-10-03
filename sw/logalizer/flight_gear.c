#include "flight_gear.h"

#include <stdio.h>
#include <netinet/in.h>
#include <string.h>

#include "utils.h"

void net_fdm_ntoh (struct FGNetFDM* fdm) {
  fdm->version = ntohl(fdm->version);
  htond(&fdm->latitude);
  htond(&fdm->longitude);
  htond(&fdm->altitude);
  htonf(&fdm->agl);

  htonf(&fdm->phi);
  htonf(&fdm->theta);
  htonf(&fdm->psi);
  htonf(&fdm->alpha);
  htonf(&fdm->beta);

  htonf(&fdm->phidot);
  htonf(&fdm->thetadot);
  htonf(&fdm->psidot);
  htonf(&fdm->vcas);
  htonf(&fdm->climb_rate);

  htonf(&fdm->v_north);
  htonf(&fdm->v_east);
  htonf(&fdm->v_down);
  htonf(&fdm->v_wind_body_north);
  htonf(&fdm->v_wind_body_east);
  htonf(&fdm->v_wind_body_down);

  htonf(&fdm->A_X_pilot);
  htonf(&fdm->A_Y_pilot);
  htonf(&fdm->A_Z_pilot);
  htonf(&fdm->stall_warning);
  htonf(&fdm->slip_deg);

  fdm->num_engines = ntohl(fdm->num_engines);
  int i;
  for (i=0; i<FG_NET_FDM_MAX_ENGINES; i++) {
    fdm->eng_state[i] = ntohl(fdm->eng_state[i]);
    htonf(&fdm->rpm[i]);
    htonf(&fdm->fuel_flow[i]);
    htonf(&fdm->egt[i]);
    htonf(&fdm->cht[i]);
    htonf(&fdm->mp_osi[i]);
    htonf(&fdm->tit[i]);
    htonf(&fdm->oil_temp[i]);
    htonf(&fdm->oil_px[i]);
  }

  fdm->num_tanks = ntohl(fdm->num_tanks);
  for (i=0; i<FG_NET_FDM_MAX_TANKS; i++) {
    htonf(&fdm->fuel_quantity[i]);
  }

  fdm->num_wheels = ntohl(fdm->num_wheels);
  for (i=0; i<FG_NET_FDM_MAX_WHEELS; i++) {
    fdm->wow[i] = ntohl(fdm->wow[i]);
    htonf(&fdm->gear_pos[i]);
    htonf(&fdm->gear_steer[i]);
    htonf(&fdm->gear_compression[i]);
  }

  fdm->cur_time = ntohl(fdm->cur_time);
  fdm->warp = ntohl(fdm->warp);

  htonf(&fdm->visibility);

  htonf(&fdm->elevator);
  htonf(&fdm->elevator_trim_tab);
  htonf(&fdm->left_flap);
  htonf(&fdm->right_flap);
  htonf(&fdm->left_aileron);
  htonf(&fdm->right_aileron);
  htonf(&fdm->rudder);
  htonf(&fdm->nose_wheel);
  htonf(&fdm->speedbrake);
  htonf(&fdm->spoilers);

}

void net_fdm_dump (struct FGNetFDM* fdm) {
  printf("net_fdm (version %d size %d)\n",fdm->version, sizeof( *fdm));
  printf("  lat, lon, alt, agl\n  [%f %f %f %f]\n",
	 fdm->latitude, fdm->longitude, fdm->altitude, fdm->agl);
  printf("  phi, theta, psi, alpha, beta\n  [%f %f %f %f %f]\n",
	 fdm->phi, fdm->theta, fdm->psi, fdm->alpha, fdm->beta);
  printf("  phidot, thetadot, psidot, vcas, climb_rate\n  [%f %f %f %f %f]\n",
	 fdm->phidot, fdm->thetadot, fdm->psidot, fdm->vcas, fdm->climb_rate);
  printf("  v_north, v_east, v_down\n  [%f %f %f]\n",
	 fdm->v_north, fdm->v_east, fdm->v_down);
  printf("  v_wind_body_north, v_wind_body_east, v_wind_body_down\n  [%f %f %f]\n",
	 fdm->v_wind_body_north, fdm->v_wind_body_east, fdm->v_wind_body_down);

  /* [......] */

  printf("  cur_time, warp\n  [%u %u]\n", fdm->cur_time, fdm->warp);
  printf("  visibility [%f]\n", fdm->visibility);
  printf("  elevator, elevator_trim_tab\n  [%f %f]\n",
	 fdm->elevator, fdm->elevator_trim_tab);
  printf("  left_flap, right_flap\n  [%f %f]\n",
	 fdm->left_flap, fdm->right_flap);
  printf("  left_aileron, right_aileron\n  [%f %f]\n",
	 fdm->left_aileron, fdm->right_aileron);
  printf("  rudder, nose_wheel\n  [%f %f]\n",
	 fdm->rudder, fdm->nose_wheel);
  printf("  speedbrake, spoilers\n  [%f %f]\n",
	 fdm->speedbrake, fdm->spoilers);

}

void net_fdm_init ( struct FGNetFDM* fdm ) {

  fdm->version = FG_NET_FDM_VERSION;

  fdm->latitude = 0.656480;
  fdm->longitude = -2.135537;
  fdm->altitude = 2.;
  fdm->agl = 1.111652;

  fdm->phi = 0.;
  fdm->theta = 0.;
  fdm->psi = 5.20;
  fdm->alpha = 0.;
  fdm->beta = 0.;

  fdm->phidot = 0.;
  fdm->thetadot = 0.;
  fdm->psidot = 0.;
  fdm->vcas = 0.;
  fdm->climb_rate = 0.;
  fdm->v_north = 0.;
  fdm->v_east = 0.;
  fdm->v_down = 0.;

  fdm->v_wind_body_north = 0.;
  fdm->v_wind_body_east = 0.;
  fdm->v_wind_body_down = 0.;

  fdm->A_X_pilot = 0.;
  fdm->A_Y_pilot = 0.;
  fdm->A_Z_pilot = 0.;

  fdm->stall_warning = 0.;
  fdm->slip_deg = 0.;

  fdm->num_engines = 2;
  fdm->eng_state[0] = 0;
  fdm->eng_state[1] = 0;
  fdm->rpm[0] = 0.;
  fdm->rpm[1] = 0.;
  fdm->fuel_flow[0] = 0.;
  fdm->fuel_flow[1] = 0.;
  fdm->egt[0] = 0.;
  fdm->egt[1] = 0.;
  fdm->egt[0] = 0.;
  fdm->egt[1] = 0.;
  fdm->cht[0] = 0.;
  fdm->cht[1] = 0.;
  fdm->mp_osi[0] = 0.;
  fdm->mp_osi[1] = 0.;
  fdm->tit[0] = 0.;
  fdm->tit[1] = 0.;
  fdm->oil_temp[0] = 0.;
  fdm->oil_temp[1] = 0.;
  fdm->oil_px[0] = 0.;
  fdm->oil_px[1] = 0.;

  fdm->num_tanks = 4;
  fdm->fuel_quantity[0] = 0.;
  fdm->fuel_quantity[1] = 0.;
  fdm->fuel_quantity[2] = 0.;
  fdm->fuel_quantity[3] = 0.;

  fdm->num_wheels = 1;
  fdm->wow[0] = 1;
  fdm->gear_pos[0] = 0.;
  fdm->gear_steer[0] = 0.;
  fdm->gear_compression[0] = 0.;

  fdm->cur_time = 3213082700ul;
  fdm->warp = 0;
  fdm->visibility = 1000.;

  fdm->elevator = 0.;
  fdm->elevator_trim_tab = 0.;
  fdm->left_flap = 0.;
  fdm->right_flap = 0.;
  fdm->left_aileron = 0.;
  fdm->right_aileron = 0.;
  fdm->rudder = 0.;
  fdm->nose_wheel = 0.;
  fdm->speedbrake = 0.;
  fdm->spoilers = 0.;

}

void net_gui_init (struct FGNetGUI* gui) {
  gui->version = FG_NET_GUI_VERSION;
  gui->latitude = 0.656480;
  gui->longitude = -2.135537;
  gui->altitude = 0.807609;
  gui->agl = 1.111652;

  gui->phi = 0.;
  gui->theta = 0.;
  gui->psi = 5.20;

  gui->vcas = 0.;
  gui->climb_rate = 0.;

  gui->num_tanks = 1;
  gui->fuel_quantity[0] = 0.;

  gui->cur_time = 3198060679ul;
  gui->warp = 1122474394ul;

  gui->ground_elev = 0.;

  gui->tuned_freq = 125.65;
  gui->nav_radial = 90.;
  gui->in_range = 1;
  gui->dist_nm = 10.;
  gui->course_deviation_deg = 0.;
  gui->gs_deviation_deg = 0.;
}

void net_gui_hton (struct FGNetGUI* gui) {
  gui->version = ntohl(gui->version);
  htond(&gui->latitude);
  htond(&gui->longitude);
  htonf(&gui->altitude);
  htonf(&gui->agl);

  htonf(&gui->phi);
  htonf(&gui->theta);
  htonf(&gui->psi);

}

void net_gui_dump (struct FGNetGUI* gui) {
  printf("net_gui (version %d size %d)\n",gui->version, sizeof( *gui));
  printf("  lat, lon, alt, agl\n  [%f %f %f %f]\n",
	 gui->latitude, gui->longitude, gui->altitude, gui->agl);
  printf("  phi, theta, psi\n  [%f %f %f]\n",
	 gui->phi, gui->theta, gui->psi);
  printf("  vcas, climb_rate\n  [%f %f]\n",
	 gui->vcas, gui->climb_rate);
  printf("  num_tanks, fuel[0], fuel[1], fuel[2], fuel[3]\n  [%u %f %f %f %f]\n",
	 gui->num_tanks, gui->fuel_quantity[0], gui->fuel_quantity[1],
	 gui->fuel_quantity[2], gui->fuel_quantity[3]);
  printf("  cur_time, warp\n  [%u %u]\n", gui->cur_time, gui->warp);
  printf("  ground_elev\n  [%f]\n",
	 gui->ground_elev);
  printf("  tuned_freq, nav_radial, in_range\n  [%f %f %u]\n",
	 gui->tuned_freq, gui->nav_radial, gui->in_range);
  printf("  dist_nm, course_deviation_deg, gs_deviation_deg\n  [%f %f %f]\n",
	 gui->dist_nm, gui->course_deviation_deg, gui->gs_deviation_deg);
}

void net_ctrls_init(struct FGNetCtrls* ctrls) {
  ctrls->version = FG_NET_CTRLS_VERSION;
  ctrls->aileron = 0.;
  ctrls->elevator = 0.;
  ctrls->rudder = 0.;
  ctrls->aileron_trim = 0.;
  ctrls->elevator_trim = 0.;
  ctrls->rudder_trim = 0.;
  ctrls->flaps = 0.;
  ctrls->spoilers = 0.;
  ctrls->speedbrake = 0.;

  ctrls->flaps_power = 1;
  ctrls->flap_motor_ok = 1;

  ctrls->num_engines = 2;
  int i;
  for (i=0; i< 2; i++) {
    ctrls->master_bat[i] = 1;
    ctrls->master_alt[i] = 1;
    ctrls->magnetos[i] = 1;
    ctrls->starter_power[i] = 0;
    ctrls->throttle[i] = 0.;
    ctrls->mixture[i] = 0.;
    ctrls->condition[i] = 0.;
    ctrls->fuel_pump_power[i] = 1;
    ctrls->prop_advance[i] = 1.;
    ctrls->feed_tank_to[i] = 1;
    ctrls->reverse[i] = 0;
    ctrls->engine_ok[i] = 1;
    ctrls->mag_left_ok[i] = 1;
    ctrls->mag_right_ok[i] = 1;
    ctrls->spark_plugs_ok[i] = 1;
    ctrls->oil_press_status[i] = 0;
    ctrls->fuel_pump_ok[i] = 1;
  }
  ctrls->num_tanks = 1;
  ctrls->fuel_selector[0] = 1;

  ctrls->xfer_pump[0] = 1;
  ctrls->cross_feed = 0;

  ctrls->brake_left = 0.;
  ctrls->brake_right = 0.;
  ctrls->copilot_brake_left = 0.;
  ctrls->copilot_brake_right = 0.;
  ctrls->brake_parking = 0.;

  ctrls->gear_handle = 1;
  ctrls->master_avionics = 0;

  ctrls->comm_1 = 123.4;
  ctrls->comm_2 = 123.4;
  ctrls->nav_1 = 123.4;
  ctrls->nav_2 = 123.4;

  ctrls->wind_speed_kt = 0.;
  ctrls->wind_dir_deg = 0.;
  ctrls->turbulence_norm = 0.;

  ctrls->temp_c = 25.;
  ctrls->press_inhg = 25.;

  ctrls->hground = 25.;
  ctrls->magvar = 0.;

  ctrls->speedup = 1;

}

void net_ctrls_ntoh(struct FGNetCtrls* ctrls) {
  ctrls->version = ntohl(ctrls->version);
  htond(&ctrls->aileron);
  htond(&ctrls->elevator);
  htond(&ctrls->rudder);
  htond(&ctrls->aileron_trim);
  htond(&ctrls->elevator_trim);
  htond(&ctrls->rudder_trim);
  htond(&ctrls->flaps);


}


void net_ctrls_dump(struct FGNetCtrls* ctrls) {
  printf("net_ctrls (version %d size %d)\n",ctrls->version, sizeof( *ctrls));
  printf("  aileron elevator rudder\n  [%f %f %f]\n",
	 ctrls->aileron, ctrls->elevator, ctrls->rudder);
  printf("  throttle\n  [%f %f %f]\n",
	 ctrls->throttle[0], ctrls->throttle[1], ctrls->throttle[2]);

}

void mplay_msg_dump ( struct FGMplayMsg *msg ) {
  printf("mplay_msg (size %d id %d)\n",msg->header.len, msg->header.id);
  printf(" reply_addr %d\n", msg->header.reply_addr);
  printf(" reply_port %d\n", msg->header.reply_port);
  printf(" callsign ");
  int i;
  for (i=0; i < MP_MAX_CALLSIGN_LEN; i++) {
    printf("%c", msg->header.callsign[i]);
  }
  printf("\n");
  printf(" model %-96s (%d)\n", msg->pos.model, strlen(msg->pos.model));
  printf(" time %f lag %f\n", msg->pos.time, msg->pos.lag);
  printf(" position %f %f %f\n", msg->pos.position[0], msg->pos.position[1], msg->pos.position[2]);
  printf(" orientation %f %f %f %f\n", msg->pos.orientation[0], msg->pos.orientation[1], msg->pos.orientation[2], msg->pos.orientation[3]);
  printf(" linear_vel %f %f %f\n", msg->pos.linear_vel[0], msg->pos.linear_vel[1], msg->pos.linear_vel[2]);
  printf(" angular_vel %f %f %f\n", msg->pos.angular_vel[0], msg->pos.angular_vel[1], msg->pos.angular_vel[2]);
  printf(" linear_accel %f %f %f\n", msg->pos.linear_accel[0], msg->pos.linear_accel[1], msg->pos.linear_accel[2]);
  printf(" angular_accel %f %f %f\n", msg->pos.angular_accel[0], msg->pos.angular_accel[1], msg->pos.angular_accel[2]);
  printf("sizeof %d\n", sizeof(struct FGMplayMsg));

}

void mplay_msg_ntoh ( struct FGMplayMsg* msg) {
  msg->header.magic = ntohl(msg->header.magic);
  msg->header.version = ntohl(msg->header.version);
  msg->header.id = ntohl(msg->header.id);
  msg->header.len = ntohl(msg->header.len);
  msg->header.reply_addr = ntohl(msg->header.reply_addr);
  msg->header.reply_port = ntohl(msg->header.reply_port);
  htond(&msg->pos.time);
  htond(&msg->pos.lag);
  htond(&msg->pos.position[0]);
  htond(&msg->pos.position[1]);
  htond(&msg->pos.position[2]);
  htonf(&msg->pos.orientation[0]);
  htonf(&msg->pos.orientation[1]);
  htonf(&msg->pos.orientation[2]);
  htonf(&msg->pos.orientation[3]);
  htonf(&msg->pos.linear_vel[0]);
  htonf(&msg->pos.linear_vel[1]);
  htonf(&msg->pos.linear_vel[2]);
  htonf(&msg->pos.angular_vel[0]);
  htonf(&msg->pos.angular_vel[1]);
  htonf(&msg->pos.angular_vel[2]);
  htonf(&msg->pos.linear_accel[0]);
  htonf(&msg->pos.linear_accel[1]);
  htonf(&msg->pos.linear_accel[2]);
  htonf(&msg->pos.angular_accel[0]);
  htonf(&msg->pos.angular_accel[1]);
  htonf(&msg->pos.angular_accel[2]);
}

void mplay_msg_init ( struct FGMplayMsg* msg) {
  msg->header.magic = MP_MSG_MAGIC;
  msg->header.version = MP_PROTO_VER;
  msg->header.id = 7;
  msg->header.len = 232;

}
