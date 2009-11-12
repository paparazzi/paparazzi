/*
 * Paparazzi $Id: ant_track.c 2009-10-30 12:14:20Z griffin $
 *
 * Copyright (C) 2009 - Pascal Brisset, Antoine Drouin
 *
 * Modified by: Mark Griffin and Todd Sandercock
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

#define MANUAL 0
#define AUTO 1

static double gps_pos_x;
static double gps_pos_y;
static double gps_alt;
static double home_alt;
static double ant_azim;
static double ant_elev;
static int mode;
static int home_found;

int fd; /* File descriptor for the port */

double hfov = 170., vfov = 170.;
double hnp = 270., vnp = 30.;
double hlim1 = 30, hlim2 = 200;
double vlim1 = 30, vlim2 = 200;

unsigned char startByte = 0x80, deviceId = 0x01, command = 0x03, commandinit =
		0x00, psiServo = 0x01, thetaServo = 0x02, speed = 0x00, speedCmd = 0x01;
unsigned char datainit = 0x40;
unsigned char data1 = 19, data2 = 68;

GtkWidget *azim_scale;
GtkWidget *elev_scale;

void on_mode_changed(GtkRadioButton *radiobutton, gpointer user_data) {
	mode
			= gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(radiobutton)) ? MANUAL
					: AUTO;
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
			GTK_ADJUSTMENT (gtk_adjustment_new (144.7, 0, 360, 1, 1, 1)));
	gtk_widget_show(azim_scale);
	gtk_table_attach(GTK_TABLE (table1), azim_scale, 1, 3, 1, 2,
			(GtkAttachOptions) (GTK_EXPAND | GTK_FILL),
			(GtkAttachOptions) (GTK_FILL), 0, 0);
	gtk_range_set_update_policy(GTK_RANGE (azim_scale), GTK_UPDATE_DELAYED);

	elev_scale = gtk_hscale_new(
			GTK_ADJUSTMENT (gtk_adjustment_new (32.3, 0, 90, 1, 1, 1)));
	gtk_widget_show(elev_scale);
	gtk_table_attach(GTK_TABLE (table1), elev_scale, 1, 3, 2, 3,
			(GtkAttachOptions) (GTK_FILL), (GtkAttachOptions) (GTK_FILL), 0, 0);

	entry1 = gtk_entry_new();
	gtk_widget_show(entry1);
	gtk_table_attach(GTK_TABLE (table1), entry1, 1, 3, 3, 4,
			(GtkAttachOptions) (GTK_EXPAND | GTK_FILL), (GtkAttachOptions) (0),
			0, 0);

	g_signal_connect ((gpointer) radiobutton1, "toggled",
			G_CALLBACK (on_mode_changed),
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

void send_pos(int vert, int hori) {
	// Split the value into two 7 bit numbers
	data1 = vert / 128;
	data2 = vert % 128;

	// Command buffer for "Set Position, 8-bit (2 data bytes)" from Pololu servo controller data sheet
	// Send vertical servo
	char buffer1[] = { startByte, deviceId, command, psiServo, data1, data2 };

	write(fd, buffer1, 6);

	// Split the value into two 7 bit numbers
	data1 = hori / 128;
	data2 = hori % 128;

	// Command buffer for "Set Position, 8-bit (2 data bytes)" from Pololu servo controller data sheet
	// Send horizontal servo
	char buffer2[] = { startByte, deviceId, command, thetaServo, data1, data2 };

	write(fd, buffer2, 6);
}

/* jump here when a NAVIGATION message is received */
void on_NAV_STATUS(IvyClientPtr app, void *user_data, int argc, char *argv[]) {
	double hpos, vpos;
	double hservo, vservo;

	if (mode == AUTO) {
		gps_pos_x = atof(argv[2]);
		gps_pos_y = atof(argv[3]);
		/* calculate azimuth */
		ant_azim = atan2(gps_pos_x, gps_pos_y) * 180. / M_PI;

		if (ant_azim < 0)
			ant_azim += 360.;

		/* calculate elevation */
		ant_elev = atan2((gps_alt - home_alt), sqrt(atof(argv[5]))) * 180.
				/ M_PI;
		if (ant_elev < 0)
			ant_elev = 0.;

		gtk_range_set_value(azim_scale, ant_azim);
		gtk_range_set_value(elev_scale, ant_elev);
	}

	// The magic is done here

	// First take the horizontal angle relative to the neutral point "hnp"
	hpos = ant_azim - hnp;

	// Keep the range between (-180,180). this is done so that it consistently swaps sides
	if (hpos < -180) {
		hpos += 360;
	} else if (hpos > 180) {
		hpos -= 360;
	}

	// keep the range within the field of view "hfov"
	if (hpos > (hfov / 2)) {
		hpos = hfov / 2;
	} else if (-hpos > (hfov / 2)) {
		hpos = -hfov / 2;
	}

	// Take the vertical angle relative to the neutral point "vnp"
	vpos = ant_elev - vnp;

	// keep within the field of view "vfov"
	if (vpos > (vfov / 2)) {
		vpos = vfov / 2;
	} else if (-vpos > (vfov / 2)) {
		vpos = -vfov / 2;
	}

	// make outputs relative to limits for the Pololu board

	vservo = (((vpos + (vfov / 2)) / vfov) * (vlim2 - vlim1)) + vlim1;
	hservo = (((hpos + (hfov / 2)) / hfov) * (hlim2 - hlim1)) + hlim1;

	/*g_message("home_alt %f gps_alt %f azim %f elev %f", home_alt, gps_alt, ant_azim, ant_elev); */

	// Send servo position.
	send_pos(vservo, hservo);

	g_message("vservo %f hservo %f", vservo, hservo);
}

int open_port(void) {
	struct termios options;

	// would probably be good to set the port up as an arg.
	fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		perror("open_port: Unable to open /dev/ttyUSB0 - ");
	} else
		fcntl(fd, F_SETFL, 0);

	tcgetattr(fd, &options);

	// Set the baud rates to 19200. This can be between 2,000 to 40,000

	cfsetispeed(&options, B19200);
	cfsetospeed(&options, B19200);

	options.c_cflag |= (CLOCAL | CREAD);

	tcsetattr(fd, TCSANOW, &options);

	// Send initialisation to the pololu board. By changing "speed" the speed of the servo can be changed
	// if "speed" is nonzero then 1 is the slowest 127 is the fastest. 0 = no speed restriction
	char buffer[] = { startByte, deviceId, speedCmd, psiServo, speed,
			startByte, deviceId, speedCmd, thetaServo, speed};

	write(fd, buffer, 10);

	return (fd);
}

int main(int argc, char** argv) {

	gtk_init(&argc, &argv);

	GtkWidget* window = build_gui();
	gtk_widget_show_all(window);

	open_port();

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

