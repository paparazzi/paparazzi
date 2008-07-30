/*
 * File: river_track.c
 * Author: mgalgs
 */

#include <stdlib.h>
#include <stdio.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <gtk-2.0/gtk/gtk.h>
//#include <gtk/gtk.h> is what the previous line should read when compiling
//with pkg-config --cflags --libs gtk+-2.0
#include "nextwp.c"
#include "waypoint.h"


/* callback associated to "Hello" messages */
void textCallback(IvyClientPtr app, void *data, int argc, char **argv)
{
    const char* arg = (argc < 1) ? "" : argv[0];
    IvySendMsg("Alo ai%s", arg);
}


void start_track( GtkWidget *widget, gpointer data )
{
    fprintf(stderr, "starting river tracking...\n");
    /* Call function get_next_waypoint (defined in nexcwp.c)
     * which returns a waypoint. */
    Waypoint new_wp = get_next_waypoint();
    IvySendMsg("gcs MOVE_WAYPOINT %d %d %f %f %f", \
            new_wp.ac_id, new_wp.wp, new_wp.lat, new_wp.lon, new_wp.alt);
}


/* Destroys the window */
static void destroy( GtkWidget *widget, gpointer data )
{
    gtk_main_quit ();
}


int main( int   argc, char *argv[] ) {
    GtkWidget *window;
    GtkWidget *button;
    GtkWidget *box1;
    
    char *bus=getenv("IVYBUS");
    char *tosend="foo";
    
    gtk_init(&argc, &argv);
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_container_set_border_width(GTK_CONTAINER(window), 10);
    gtk_window_set_title (GTK_WINDOW (window), "River Tracking");
    
    box1 = gtk_hbox_new (FALSE, 0);
    gtk_container_add (GTK_CONTAINER (window), box1);
    
    //first button...
    button = gtk_button_new_with_label("start river tracking");
    g_signal_connect(G_OBJECT(button), "clicked", G_CALLBACK(start_track), &tosend);
    gtk_box_pack_start(GTK_BOX(box1), button, TRUE, TRUE, 0);
    gtk_widget_show (button);
            
    //second button...
    button = gtk_button_new_with_label("Quit");
    g_signal_connect(G_OBJECT(button), "clicked", G_CALLBACK(destroy), &tosend);
    gtk_box_pack_start(GTK_BOX (box1), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_widget_show(box1);
    gtk_widget_show(window);

    IvyInit("river_track", "river_track READY", NULL, NULL, NULL, NULL);
    IvyBindMsg(textCallback,&tosend,"^river_track hello(.*)");
    IvyStart(bus);
    
    gtk_main();
    return 0;
}
