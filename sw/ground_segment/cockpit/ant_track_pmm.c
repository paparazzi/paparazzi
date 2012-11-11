/*
 * Copyright (C) 2010
 *
 * Modified by: Mark Griffin and Todd Sandercock
 * Modified by: Chris Efstathiou for the Pololu Micro Mestro usb servo controller Jun/2010
 * Added command line options, a 360 pan option and "MANUAL" control.
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 * The antenna tracker zero azimuth is to the NORTH (NORTH = 0, EAST = 90 WEST = -90, SOUTH = 180/0 degrees).
 * The elevation zero is totally horizontal, 90 is up and 180 is to the back.
 * The servo used must be able to do 180 degrees in order to get full 360 degree coverage from the tracker.
 */
#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#include <termios.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */

#define POLOLU_PROTOCOL_START          0xAA
#define POLOLU_BOARD_ID                12
#define SET_SERVO_POSITION_COMMAND     0x04
#define SET_SERVO_SPEED_COMMAND        0x07
#define SET_SERVO_ACCELERATION_COMMAND 0x09
#define SET_SERVO_CENTER_COMMAND       0x22

#define MANUAL 0
#define AUTO 1

static double gps_pos_x;
static double gps_pos_y;
static double gps_alt;
static double home_alt;
static double ant_azim = 180.;
static double ant_elev = 90.;
static int mode;
static int home_found;
static int ant_tracker_pan_mode = 180;
static double theta_servo_pw_span = 1000.;
static double psi_servo_pw_span = 1000.;
static double theta_servo_center_pw = 1500;
static double psi_servo_center_pw = 1500;
static char pololu_board_id = 12;
static char servo_acceleration = 3;
static char psi_servo_address = 1;
static char theta_servo_address = 0;

int fd; /* File descriptor for the port */
volatile int serial_error = 0;

double hfov = 180., vfov = 180.;
double hnp = 0., vnp = 0.;

unsigned char  speed = 0x00;

void set_servos(void);

GtkWidget *azim_scale;
GtkWidget *elev_scale;

void on_mode_changed(GtkRadioButton *radiobutton, gpointer user_data) {


  mode = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radiobutton)) ? MANUAL : AUTO;


	//IvySendMsg("1ME RAW_DATALINK 80 SETTING;0;0;%d", mode);
	//g_message("Mode changed to %d" , mode);
}

void on_azimuth_changed(GtkAdjustment *hscale, gpointer user_data) {

if (mode == MANUAL) {
   ant_azim = gtk_range_get_value(GTK_RANGE (azim_scale));
   ant_elev = gtk_range_get_value(GTK_RANGE (elev_scale));
   set_servos();
   }
	//IvySendMsg("1ME RAW_DATALINK 80 SETTING;0;0;%d", mode);
	//g_message("Mode changed to %d" , mode);
}

//void on_elevation_changed(GtkRange *elev_scale, gpointer user_data) {
void on_elevation_changed(GtkAdjustment *hscale, gpointer user_data) {

if (mode == MANUAL) {
   ant_azim = gtk_range_get_value(GTK_RANGE (azim_scale));
   ant_elev = gtk_range_get_value(GTK_RANGE (elev_scale));
   set_servos();
 }


	//IvySendMsg("1ME RAW_DATALINK 80 SETTING;0;0;%d", mode);
	//g_message("Mode changed to %d" , mode);
}


#define GLADE_HOOKUP_OBJECT(component,widget,name) \
  g_object_set_data_full (G_OBJECT (component), name, \
	gtk_widget_ref (widget), (GDestroyNotify) gtk_widget_unref)

#define GLADE_HOOKUP_OBJECT_NO_REF(component,widget,name) \
  g_object_set_data (G_OBJECT (component), name, widget)

GtkWidget* build_gui(void) {
	GtkWidget *window1;
	GtkWidget *vbox1;
	GtkWidget *vbox2;
	GtkWidget *table1;
	GtkWidget *label1;
	GtkWidget *label2;
	GtkWidget *label3;
	GtkWidget *label4;
	GtkWidget *radiobutton1;
	GSList *radiobutton1_group = NULL;
	GtkWidget *radiobutton2;
	GtkWidget *entry1;

	window1 = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title(GTK_WINDOW (window1), "tracking antenna");

	vbox1 = gtk_vbox_new(FALSE, 0);
	gtk_widget_show(vbox1);
	gtk_container_add(GTK_CONTAINER (window1), vbox1);

	vbox2 = gtk_vbox_new(FALSE, 0);
	gtk_widget_show(vbox2);
	gtk_box_pack_start(GTK_BOX (vbox1), vbox2, TRUE, TRUE, 0);

	table1 = gtk_table_new(4, 3, FALSE);
	gtk_widget_show(table1);
	gtk_box_pack_start(GTK_BOX (vbox2), table1, TRUE, TRUE, 0);
	gtk_table_set_col_spacings(GTK_TABLE (table1), 5);

	label1 = gtk_label_new("Azimuth");
	gtk_widget_show(label1);
	gtk_table_attach(GTK_TABLE (table1), label1, 0, 1, 1, 2,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_misc_set_alignment(GTK_MISC (label1), 0, 0.5);

	label2 = gtk_label_new("Elevation");
	gtk_widget_show(label2);
	gtk_table_attach(GTK_TABLE (table1), label2, 0, 1, 2, 3,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_misc_set_alignment(GTK_MISC (label2), 0, 0.5);

	label3 = gtk_label_new("Id");
	gtk_widget_show(label3);
	gtk_table_attach(GTK_TABLE (table1), label3, 0, 1, 3, 4,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_misc_set_alignment(GTK_MISC (label3), 0, 0.5);

	label4 = gtk_label_new("mode");
	gtk_widget_show(label4);
	gtk_table_attach(GTK_TABLE (table1), label4, 0, 1, 0, 1,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_misc_set_alignment(GTK_MISC (label4), 0, 0.5);

	radiobutton1 = gtk_radio_button_new_with_mnemonic(NULL, "manual");
	gtk_widget_show(radiobutton1);
	gtk_table_attach(GTK_TABLE (table1), radiobutton1, 1, 2, 0, 1,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_radio_button_set_group(GTK_RADIO_BUTTON (radiobutton1),
			radiobutton1_group);
	radiobutton1_group = gtk_radio_button_get_group(
			GTK_RADIO_BUTTON (radiobutton1));

	radiobutton2 = gtk_radio_button_new_with_mnemonic(NULL, "tracking");
	gtk_widget_show(radiobutton2);
	gtk_table_attach(GTK_TABLE (table1), radiobutton2, 2, 3, 0, 1,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (0), 0, 0);
	gtk_radio_button_set_group(GTK_RADIO_BUTTON (radiobutton2),
			radiobutton1_group);
	radiobutton1_group = gtk_radio_button_get_group(
			GTK_RADIO_BUTTON (radiobutton2));

	azim_scale = gtk_hscale_new(
			//GTK_ADJUSTMENT (gtk_adjustment_new (180, 0, 360, 1, 1, 1)));
			GTK_ADJUSTMENT (gtk_adjustment_new (0, 0, 360, 1, 1, 1)));
	gtk_widget_show(azim_scale);
	gtk_table_attach(GTK_TABLE (table1), azim_scale, 1, 3, 1, 2,
			(GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
			(GtkAttachOptions) (GTK_FILL), 0, 0);
//	gtk_range_set_update_policy(GTK_RANGE (azim_scale), GTK_UPDATE_DELAYED);

	elev_scale = gtk_hscale_new(
			//GTK_ADJUSTMENT (gtk_adjustment_new (45, 0, 90, 1, 1, 1)));
			GTK_ADJUSTMENT (gtk_adjustment_new (0, 0, 90, 1, 1, 1)));
	gtk_widget_show(elev_scale);
	gtk_table_attach(GTK_TABLE (table1), elev_scale, 1, 3, 2, 3,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (GTK_FILL), 0, 0);
//        gtk_range_set_update_policy(GTK_RANGE (elev_scale), GTK_UPDATE_DELAYED);

	entry1 = gtk_entry_new();
	gtk_widget_show(entry1);
	gtk_table_attach(GTK_TABLE (table1), entry1, 1, 3, 3, 4,
			(GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) (0),
			0, 0);

	g_signal_connect ((gpointer) radiobutton1, "toggled",
			G_CALLBACK (on_mode_changed),
			NULL);

	g_signal_connect ((gpointer) azim_scale, "value-changed",
			G_CALLBACK (on_azimuth_changed),
			NULL);

	g_signal_connect ((gpointer) elev_scale, "value-changed",
			G_CALLBACK (on_elevation_changed),
			NULL);

	/* Store pointers to all widgets, for use by lookup_widget(). */
	GLADE_HOOKUP_OBJECT_NO_REF (window1, window1, "window1");
	GLADE_HOOKUP_OBJECT (window1, vbox1, "vbox1");
	GLADE_HOOKUP_OBJECT (window1, vbox2, "vbox2");
	GLADE_HOOKUP_OBJECT (window1, table1, "table1");
	GLADE_HOOKUP_OBJECT (window1, label1, "label1");
	GLADE_HOOKUP_OBJECT (window1, label2, "label2");
	GLADE_HOOKUP_OBJECT (window1, label3, "label3");
	GLADE_HOOKUP_OBJECT (window1, label4, "label4");
	GLADE_HOOKUP_OBJECT (window1, radiobutton1, "radiobutton1");
	GLADE_HOOKUP_OBJECT (window1, radiobutton2, "radiobutton2");
	GLADE_HOOKUP_OBJECT (window1, entry1, "entry1");

	return window1;
}

void set_servos(void)
{

double hpos, vpos;
int hservo = theta_servo_center_pw, vservo = psi_servo_center_pw;

	// The magic is done here

		if (ant_tracker_pan_mode == 180){

	   // Take the vertical angle relative to the neutral point "vnp"
	   vpos = ant_elev - vnp;

		   // keep within the field of view "vfov"
		   if (vpos > (vfov / 2)) { vpos = vfov / 2; } else if(-vpos > (vfov / 2)){ vpos = -vfov / 2; }

		   // First take the horizontal angle relative to the neutral point "hnp"
	   hpos = ant_azim - hnp;

		   // Keep the range between (-180,180). this is done so that it consistently swaps sides
	   if (hpos < -180){ hpos += 360; }else if(hpos > 180){ hpos -= 360; }

		   // Swap sides to obtain 360 degrees of Azimuth coverage.
		   if(hpos > 90){ hpos = hpos-180;  vpos = 180-vpos; }else if(hpos < -90){ hpos = hpos+180;  vpos = 180-vpos; }

		   // keep the range within the field of view "hfov"
		   if (hpos > (hfov / 2)) { hpos = hfov / 2; } else if (-hpos > (hfov / 2)) { hpos = -hfov / 2;	}

		   // Convert angles to servo microsecond values suitable for the Pololu micro Maestro servo controller.
		   vpos = (psi_servo_center_pw-(psi_servo_pw_span/2)) + (vpos*(psi_servo_pw_span/vfov));
		   hpos = theta_servo_center_pw + (hpos*(theta_servo_pw_span/(hfov/2)));

		   //convert the values to integer.
		   hservo = hpos;
		   vservo = vpos;


		}else{
				vpos = ant_elev;

			// First take the horizontal angle relative to the neutral point "hnp"
			hpos = ant_azim - hnp;

				// Keep the range between (-180,180).
			if (hpos < -180){ hpos += 360; }else if(hpos > 180){ hpos -= 360; }

				// Keep the range between 0 to 360.
			//if (hpos < 0) { hpos += 360; } else if (hpos > 360){ hpos -= 360; }

				// keep the range within the field of view "hfov"
				if (hpos > (hfov / 2)) { hpos = hfov / 2; } else if (-hpos > (hfov / 2)) { hpos = -hfov / 2;	}

				// Convert angles to servo microsecond values suitable for the Pololu micro Maestro servo controller.
				vpos = (psi_servo_center_pw-(psi_servo_pw_span/2)) + (fabs(vpos)*(psi_servo_pw_span/vfov));
				hpos = theta_servo_center_pw + (hpos*(theta_servo_pw_span/hfov));

				//convert the values to integer.
				hservo = hpos;
				vservo = vpos;
			}

		   // Sanity check.
		   if (vservo < (psi_servo_center_pw-fabs(psi_servo_pw_span/2)) ){
			  vservo = (psi_servo_center_pw-fabs(psi_servo_pw_span/2));

		   }else if(vservo > (psi_servo_center_pw+fabs(psi_servo_pw_span/2))){ vservo = (psi_servo_center_pw+fabs(psi_servo_pw_span/2)); }

		   if (hservo < (theta_servo_center_pw-fabs(theta_servo_pw_span/2)) ){
			  hservo = (theta_servo_center_pw-fabs(theta_servo_pw_span/2));

		   }else if(hservo > (theta_servo_center_pw+fabs(theta_servo_pw_span/2))){ hservo = (theta_servo_center_pw+fabs(theta_servo_pw_span/2)); }

		hservo *= 4; //The pololu Maestro uses 0.25 microsecond increments so we need to multiply microseconds by 4.
		vservo *= 4; //The pololu Maestro uses 0.25 microsecond increments so we need to multiply microseconds by 4.
	//g_message("home_alt %f gps_alt %f azim %f elev %f", home_alt, gps_alt, ant_azim, ant_elev);

	// Send servo position.
	char buffer1[]={ POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_POSITION_COMMAND, psi_servo_address, vservo%128, vservo/128,
						 POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_POSITION_COMMAND, theta_servo_address, hservo%128, hservo/128
					   };

	serial_error = write(fd, buffer1, 12);

	g_message("vservo %i hservo %i", (int)(vservo/4), (int)(hservo/4)); //Divide by 4 so we can have the servo PW with 1 microsecond resolution.

return;
}



/* jump here when a GPS message is received */
void on_GPS_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]) {
	if (home_found == 0) {
		if (atof(argv[0]) == 3) { /* wait until we have a valid GPS fix */
			home_alt = atof(argv[4]) / 100.; /* get the altitude */
			home_found = 1;
		}
	}
	gps_alt = atof(argv[4]) / 100.;
}

/* jump here when a NAVIGATION message is received */
void on_NAV_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]) {


	if (mode == AUTO) {
		gps_pos_x = atof(argv[2]);
		gps_pos_y = atof(argv[3]);
		/* calculate azimuth */
				//should be "atan2(gps_pos_y, gps_pos_x)" but it is reversed to give 0 when North.
		ant_azim = atan2(gps_pos_x, gps_pos_y) * 180. / M_PI;

		if (ant_azim < 0)
			ant_azim += 360.;

		/* calculate elevation */
		ant_elev = atan2((gps_alt - home_alt), sqrt(atof(argv[5]))) * 180. / M_PI;
			// Sanity check
				if (ant_elev < 0){ ant_elev = 0.; }
		gtk_range_set_value(GTK_RANGE (azim_scale), ant_azim);
		gtk_range_set_value(GTK_RANGE (elev_scale), ant_elev);

				set_servos();

	   }


}

int open_port(char* port ) {
	struct termios options;

	// would probably be good to set the port up as an arg.
		// The Pololu micro maestro registers two ports /dev/ttyACM0 and /dev/ttyACM1, /dev/ttyACM0 is the data port.
	fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		//perror("open_port: Unable to open /dev/ttyUSB1");
				printf ("open_port: Unable to open %s \n", port);
				serial_error = fd;

	} else
				printf("Success %s %s \n", port, "opened");
		fcntl(fd, F_SETFL, 0);

	tcgetattr(fd, &options);

	// Set the baud rates to 19200. This can be between 2,000 to 40,000

	cfsetispeed(&options, B19200);
	cfsetospeed(&options, B19200);

	options.c_cflag |= (CLOCAL | CREAD);

	tcsetattr(fd, TCSANOW, &options);

	// Send initialisation to the pololu micro maestro board.
	// if "speed" is nonzero then 1 is the slowest 127 is the fastest. 0 = no speed restriction
	char buffer_0[] = { POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_SPEED_COMMAND, psi_servo_address, 0x00, 0x00,
				POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_SPEED_COMMAND, theta_servo_address, 0x00, 0x00
						  };

	serial_error = write(fd, buffer_0, 12);
	// Set servo acceleration to 3 for protecting the servo gears. Fastest = 0, slowest = 255
	char buffer_1[] = { POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_ACCELERATION_COMMAND, psi_servo_address, (servo_acceleration%128),
							(servo_acceleration/128),
						POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_ACCELERATION_COMMAND, theta_servo_address, (servo_acceleration%128),
							(servo_acceleration/128)
						  };

	serial_error = write(fd, buffer_1, 12);
		// Set the two servos to their neutral position, Azimuth = 1500us = EAST = 0 degrees & Elevation = 1000 = parallel to ground.
	char buffer_2[] = { POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_POSITION_COMMAND, theta_servo_address,
							(((int)theta_servo_center_pw*4) % 128), (((int)theta_servo_center_pw*4) / 128),
						POLOLU_PROTOCOL_START, pololu_board_id, SET_SERVO_POSITION_COMMAND, psi_servo_address,
							(((int)psi_servo_center_pw*4) % 128), (((int)psi_servo_center_pw*4) / 128)
						  };

	serial_error = write(fd, buffer_2, 12);


	return (fd);
}

int main(int argc, char** argv) {

	gtk_init(&argc, &argv);


		int x = 0, y = 0, z = 0;
		char buffer[20];
		char serial_open = 0;
		printf ("Antenna Tracker for the Paparazzi autopilot, Chris Efstathiou 2010 \n");

		if (argc > 1){
			char arg_string1[] = "--help";
			for (x=1; x < argc; x++){
			   if ( (strncmp(argv[x], arg_string1, (sizeof(arg_string1)-1))) == 0 ){
				  printf ("OPTIONS \n");
				  printf ("-------------------------------------------------------------------------------- \n");
				  printf ("'--help' displays this screen \n");
				  printf ("'--port=xxx..x' opens port xxx..x, example --port=/dev/ttyACM0 (Default) \n");
				  printf ("'--pan=xxx' sets pan mode to 180 or 360 degrees. Example --pan=180 (Default) \n");
				  printf ("'--zero_angle=xxx' set the mechanical zero angle. Default is 0 (North)\n");
				  printf ("'--id=xx' sets the Pololu board id. Example --id=12 (Default)\n");
				  printf ("'--servo_acc=xxx' sets the servo acceleration. Example --servo_acc=3 (Default)\n");
				  printf ("'--pan_servo=x' sets the pan (Theta) servo number. Example --pan_servo=0 (Default)\n");
				  printf ("'--tilt_servo=x' sets the tilt (Psi) servo number.Example --tilt_servo=1 (Default) \n");
				  printf ("'--pan_epa=xx..x' sets the Azimuth servo's max travel (Default is 1100us) \n");
				  printf ("'--tilt_epa=xx..x' sets the elevation servo's max travel (Default is 1100us). \n");
				  printf ("HINT a negative value EPA value reverses the servo direction \n");
				  printf ("'--pan_servo_center_pw=xx..x' sets the Azimuth servo's center position (Default is 1500us) \n");
				  printf ("'--tilt_servo_center_pw=xx..x' sets the elevation servo's center position (Default is 1500us) \n");
				  printf ("WARNING: The pololu board limit servo travel to 1000-2000 microseconds. \n");
				  printf ("WARNING: Use the pololu board setup program to change the above limits. \n");
				  printf ("Example --tilt_epa=1100 sets the PW from 950 to 2050 microseconds \n");
				  printf ("Example --pan_epa=-1000 sets the PW from 1000 to 2000 microseconds and reverses the servo direction \n");
				  printf ("An EPA of 1100 sets the servo travel from 1500+(1100/2)=2050us to 1500-(1100/2)=950us. \n");
				  printf ("Use programmable servos like the Hyperion Atlas. \n");
				  printf ("You can also use the proportional 360 degree GWS S125-1T as the Theta (Azimuth) \n");
				  printf ("servo or the mighty but expensive Futaba S5801 \n");
				  printf (" \n");
				  printf ("FOR THE 360 DEGREE PAN MODE: \n");
				  printf ("Mechanical zero (0 degrees or 1500 ms) is to the NORTH, 90 = EAST, +-180 = SOUTH and -90 = WEST. \n");
				  printf ("Elevation center is 45 degrees up (1500ms), 0 degrees = horizontal, 90 degrees is vertical (up) \n");
				  printf ("Of course use this mode if your PAN servo can do a full 360 degrees rotation (GWS S125-1T for example) \n");
				  printf (" \n");
				  printf ("FOR THE 180 DEGREE PAN MODE: \n");
				  printf ("Mechanical zero (0 degrees or 1500 ms) is to the NORTH, 90 = EAST, -90 = WEST. \n");
				  printf ("Elevation center is 90 degrees up (1500ms), 0 degrees = horizontal, 180 degrees is horizontal to the opposite side \n");
				  printf ("When the azimuth is > 90 or < -90 the azimuth and elevation servos swap sides to obtain the full 360 degree coverage. \n");
				  printf ("Of course your PAN and TILT servos must be true 180 degrees servos like the Hyperion ATLAS servos for example. \n");
				  printf (" \n");
				  printf ("-------------------------------------------------------------------------------- \n");
				  printf ("Antenna Tracker V1.2 for the Paparazzi autopilot 28/June/2010 \n");
				  printf ("-------------------------------------------------------------------------------- \n");
				  return 0;
			   }
			}
		   printf ("Type '--help' for help \n");

		   for(z=0; z < sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
		   char arg_string2[] = "--port=";
		   for (x=1; x < argc; x++){

			   if ( (strncmp(argv[x], arg_string2, (sizeof(arg_string2)-1))) == 0 ){
				  y=sizeof(arg_string2)-1;
				  z=0;
				  while (1){
						buffer[z] = argv[x][y];
					   if(buffer[z] != '\0'){ y++; z++; }else{ break; }
				  }
				  printf ("Trying to open %s \n", buffer);
				  open_port(buffer);
			   }
		   }
		   for(z=0; z<sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
		   char arg_string3[] = "--pan=";
		   for (x=1; x < argc; x++){
			   if ( (strncmp(argv[x], arg_string3, (sizeof(arg_string3)-1))) == 0 ){
				  y=(sizeof(arg_string3)-1);
				  z=0;
				  while (1){
						buffer[z] = argv[x][y];
						if(buffer[z] != '\0'){ y++; z++; }else{ break; }
				  }
				  ant_tracker_pan_mode = atoi(buffer);
				  if (ant_tracker_pan_mode == 180 || ant_tracker_pan_mode == 360){
					 printf ("PAN mode set to %i %s \n", ant_tracker_pan_mode, "degrees");
					 if(ant_tracker_pan_mode == 360){ hfov = 360; vfov = 90; }else{ hfov = 180; vfov = 180; }

				  }else{
						  perror("ERROR: Pan mode can be either 180 or 360 degrees");
						  ant_tracker_pan_mode = 180;
						  hfov = 180;
						  vfov = 180;
						  printf ("PAN servo set to %i %s \n", ant_tracker_pan_mode, "degrees");
					  }
			   }
		   }

			  for(z=0; z < sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
			  char arg_string4[] = "--pan_epa=";
			  for (x=1; x < argc; x++){
				  if ( (strncmp(argv[x], arg_string4, (sizeof(arg_string4)-1))) == 0 ){
					 y=(sizeof(arg_string4)-1);
					 z=0;
					 while (1){
						   buffer[z] = argv[x][y];
						   if(buffer[z] != '\0'){ y++; z++; }else{ break; }
					 }
					 theta_servo_pw_span = atoi(buffer);
					 printf ("THETA servo EPA set to  %i \n", atoi(buffer));
					 if (abs(theta_servo_pw_span) > 1000){
						printf ("REMEMBER TO SET THE MIN/MAX SERVO LIMITS WITH THE POLOLU SETUP PROGRAM \n");
						printf ("OTHERWISE THE MAX SERVO MOVEMENT WILL BE RESTRAINED TO 1000 MICROSECONDS \n");
					 }
				  }
			  }

			  for(z=0; z < sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
			  char arg_string5[] = "--tilt_epa=";
			  for (x=1; x < argc; x++){
				  if ( (strncmp(argv[x], arg_string5, (sizeof(arg_string5)-1))) == 0 ){
					 y=(sizeof(arg_string5)-1);
					 z=0;
					 while (1){
						   buffer[z] = argv[x][y];
						   if(buffer[z] != '\0'){ y++; z++; }else{ break; }
					 }
					 psi_servo_pw_span = atoi(buffer);
					 printf ("PSI servo EPA set to  %i \n", atoi(buffer));
					 if (abs(psi_servo_pw_span) > 1000){
						printf ("REMEMBER TO SET THE MIN/MAX SERVO LIMITS WITH THE POLOLU SETUP PROGRAM \n");
						printf ("OTHERWISE THE MAX SERVO MOVEMENT WILL BE RESTRAINED TO 1000 MICROSECONDS \n");
					 }
				  }
			   }
			  for(z=0; z < sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
			  char arg_string8[] = "--id=";
			  for (x=1; x < argc; x++){
				  if ( (strncmp(argv[x], arg_string8, (sizeof(arg_string8)-1))) == 0 ){
					 y=(sizeof(arg_string8)-1);
					 z=0;
					 while (1){
						   buffer[z] = argv[x][y];
						   if(buffer[z] != '\0'){ y++; z++; }else{ break; }
					 }
					 pololu_board_id = (char)atoi(buffer);
					 printf ("Pololu Board id set to  %i \n", atoi(buffer));
				  }
			   }
			  for(z=0; z < sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
			  char arg_string9[] = "--servo_acc=";
			  for (x=1; x < argc; x++){
				  if ( (strncmp(argv[x], arg_string9, (sizeof(arg_string9)-1))) == 0 ){
					 y=(sizeof(arg_string9)-1);
					 z=0;
					 while (1){
						   buffer[z] = argv[x][y];
						   if(buffer[z] != '\0'){ y++; z++; }else{ break; }
					 }
					 servo_acceleration = (char)atoi(buffer);
					 printf ("Servo acceleration set to  %i \n", atoi(buffer));
				  }
			   }
		   for(z=0; z < sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
		   char arg_string10[] = "--pan_servo=";
		   for (x=1; x < argc; x++){
			   if ( (strncmp(argv[x], arg_string10, (sizeof(arg_string10)-1))) == 0 ){
				  y=(sizeof(arg_string10)-1);
				  z=0;
				  while (1){
						buffer[z] = argv[x][y];
						if(buffer[z] != '\0'){ y++; z++; }else{ break; }
				  }
				  theta_servo_address = (char)atoi(buffer);
				  printf ("Pan (Theta) servo number set to  %i \n", atoi(buffer));
			   }
		   }

		   for(z=0; z < sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
		   char arg_string11[] = "--tilt_servo=";
		   for (x=1; x < argc; x++){
			   if ( (strncmp(argv[x], arg_string11, (sizeof(arg_string11)-1))) == 0 ){
				  y=(sizeof(arg_string11)-1);
				  z=0;
				  while (1){
						buffer[z] = argv[x][y];
						if(buffer[z] != '\0'){ y++; z++; }else{ break; }
				  }
				  psi_servo_address = (char)atoi(buffer);
				  printf ("Tilt (Psi) servo number set to  %i \n", atoi(buffer));
			   }
		   }

		   for(z=0; z < sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
		   char arg_string12[] = "--zero_angle=";
		   for (x=1; x < argc; x++){
			   if ( (strncmp(argv[x], arg_string12, (sizeof(arg_string12)-1))) == 0 ){
				  y=(sizeof(arg_string12)-1);
				  z=0;
				  while (1){
						buffer[z] = argv[x][y];
						if(buffer[z] != '\0'){ y++; z++; }else{ break; }
				  }
				  hnp = (double)atoi(buffer);
				  printf ("Zero angle is set to %i %s \n", atoi(buffer), "degrees");
			   }
		   }
		   for(z=0; z < sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
		   char arg_string13[] = "--tilt_servo_center_pw=";
		   for (x=1; x < argc; x++){
			   if ( (strncmp(argv[x], arg_string13, (sizeof(arg_string13)-1))) == 0 ){
				  y=(sizeof(arg_string13)-1);
				  z=0;
				  while (1){
						buffer[z] = argv[x][y];
						if(buffer[z] != '\0'){ y++; z++; }else{ break; }
				  }
				  psi_servo_center_pw = atoi(buffer);
				  printf ("PSI servo center pulse width set to  %i \n", atoi(buffer));
			   }
		   }
		   for(z=0; z < sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
		   char arg_string14[] = "--pan_servo_center_pw=";
		   for (x=1; x < argc; x++){
			   if ( (strncmp(argv[x], arg_string14, (sizeof(arg_string14)-1))) == 0 ){
				  y=(sizeof(arg_string14)-1);
				  z=0;
				  while (1){
						buffer[z] = argv[x][y];
						if(buffer[z] != '\0'){ y++; z++; }else{ break; }
				  }
				  theta_servo_center_pw = atoi(buffer);
				  printf ("THETA servo center pulse width set to  %i \n", atoi(buffer));
			   }
		   }
		   for(z=0; z<sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
		   char arg_string15[] = "--hfov=";
		   for (x=1; x < argc; x++){
			   if ( (strncmp(argv[x], arg_string15, (sizeof(arg_string15)-1))) == 0 ){
				  y=(sizeof(arg_string15)-1);
				  z=0;
				  while (1){
						buffer[z] = argv[x][y];
						if(buffer[z] != '\0'){ y++; z++; }else{ break; }
				  }
				  hfov = atoi(buffer);
				  printf ("Horizontal field of view set to %i %s \n", (int)hfov, "degrees");
			   }
		   }
		   for(z=0; z<sizeof(buffer); z++){ buffer[z] = '\0'; } //Reset the buffer.
		   char arg_string16[] = "--vfov=";
		   for (x=1; x < argc; x++){
			   if ( (strncmp(argv[x], arg_string16, (sizeof(arg_string16)-1))) == 0 ){
				  y=(sizeof(arg_string16)-1);
				  z=0;
				  while (1){
						buffer[z] = argv[x][y];
						if(buffer[z] != '\0'){ y++; z++; }else{ break; }
				  }
				  vfov = atoi(buffer);
				  printf ("Vertical field of view set to %i %s \n", (int)vfov, "degrees");
			   }
		   }
		}

		if(serial_open == 0){ printf ("Trying to open /dev/ttyACM0 \n");  open_port("/dev/ttyACM0"); }

	GtkWidget* window = build_gui();
	gtk_widget_show_all(window);

		if (mode == MANUAL) {
		   ant_azim = gtk_range_get_value(GTK_RANGE (azim_scale));
		   ant_elev = gtk_range_get_value(GTK_RANGE (elev_scale));
		   set_servos();
		}

	IvyInit("AntennaTracker", "AntennaTracker READY", NULL, NULL, NULL, NULL);
	IvyBindMsg(
			on_GPS_STATUS,
			NULL,
			"^\\S* GPS (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
	IvyBindMsg(on_NAV_STATUS, NULL,
			"^\\S* NAVIGATION (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
	IvyStart("127.255.255.255");
	gtk_main();

	return 0;
}
