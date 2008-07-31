/*
 * File: river_track.c
 * Author: mgalgs
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <gtk-2.0/gtk/gtk.h>
/*#include <gtk/gtk.h> is what the previous line should read when compiling
with pkg-config --cflags --libs gtk+-2.0 */
#include "nextwp.c"
#include "waypoint.h"



/* Some global variables */
float END_LAT = 0.0; //latitude of the "end" waypoint (the fifth waypoint defined in flight plan)
float END_LON = 0.0; //longitude of the "end" waypoint (the fifth waypoint defined in flight plan)
int CONTINUE = 1; //When this variable is set to 0 river_track will stop moving the waypoint around
float ADD = 0; //just to move the waypoint around for fun



/* callback associated to "Hello" messages */
void textCallback(IvyClientPtr app, void *data, int argc, char **argv)
{
    const char* arg = (argc < 1) ? "" : argv[0];
    IvySendMsg("Alo ai%s", arg);
}


/* Sets the global variables END_LAT & END_LON to the GPS coordinates
 * of the end waypoint (the fifth waypoint defined in the flight plan).
 */
void set_end(IvyClientPtr app, void *data, int argc, char **argv)
{
    const char *arg = (argc < 1) ? "" : argv[0];

    char *current;
    current = strtok(arg, " ");
    current = strtok(NULL, " ");
    current = strtok(NULL, " ");
    current = strtok(NULL, " ");
    END_LAT = atof(current);
    current = strtok(NULL, " ");
    END_LON = atof(current);
//    fprintf(stderr, "END_LAT=%f. END_LON=%f.", END_LAT, END_LON);
}

/* Called by clicking a button */
void init_river_tracking( GtkWidget *widget, gpointer data )
{
    fprintf(stderr, "initializing river tracking...\n");
    /* Call function get_next_waypoint (defined in nexcwp.c)
     * which returns a waypoint. */
    Waypoint new_wp = get_next_waypoint();
    IvySendMsg("gcs MOVE_WAYPOINT %d %d %f %f %f", \
            new_wp.ac_id, 4, new_wp.lat, new_wp.lon, new_wp.alt);
}


/* Called when aircraft reaches "Block 1" (the second 
 * block defined in the flight plan)
 */
void start_track(IvyClientPtr app, void *data, int argc, char **argv)
{
    fprintf(stderr, ".");
    /* Call function get_next_waypoint (defined in nexcwp.c)
     * which returns a waypoint. */
    Waypoint new_wp = get_next_waypoint();
    
    if(CONTINUE) {
        IvySendMsg("gcs MOVE_WAYPOINT %d %d %f %f %f", \
                new_wp.ac_id, 4, new_wp.lat, new_wp.lon, new_wp.alt); //Always move waypoint 4
    }
    /* else we've reached the end (we've seen the "end" waypoint in the
     * picture), so don't send any message.
     */
}


/* Destroys the window */
static void destroy( GtkWidget *widget, gpointer data )
{
    gtk_main_quit ();
}


int main( int   argc, char *argv[] )
{
    GtkWidget *window;
    GtkWidget *button;
    GtkWidget *box1;
    
    char *bus=getenv("IVYBUS");
    
    gtk_init(&argc, &argv);
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_container_set_border_width(GTK_CONTAINER(window), 10);
    gtk_window_set_title (GTK_WINDOW (window), "River Tracking");
    
    box1 = gtk_hbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (window), box1);
    
    //first button...
    button = gtk_button_new_with_label("(Re)define region of interest");
    g_signal_connect(G_OBJECT(button), "clicked", G_CALLBACK(init_river_tracking), 0);
    gtk_box_pack_start(GTK_BOX(box1), button, TRUE, TRUE, 0);
    gtk_widget_show (button);
            
    //second button...
    button = gtk_button_new_with_label("Quit");
    g_signal_connect(G_OBJECT(button), "clicked", G_CALLBACK(destroy), 0);
    gtk_box_pack_start(GTK_BOX (box1), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_widget_show(box1);
    gtk_widget_show(window);

    IvyInit("river_track", "river_track READY", NULL, NULL, NULL, NULL);
    IvyBindMsg(textCallback,0,"^river_track hello(.*)");
    IvyBindMsg(start_track,0,"(NAV_STATUS 1 1 +.*)");
    IvyBindMsg(set_end,0,"(WAYPOINT_MOVED 1 5 +.*)");
    IvyStart(bus);
    
    gtk_main();
    return 0;
}
