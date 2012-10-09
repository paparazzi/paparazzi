#include <glib.h>
#include <gtk/gtk.h>
#include <stdio.h>
#include <stdlib.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>


static void on_button_clicked (GtkWidget *widget, gpointer data) {
  g_message("hello world");
  int foo = 42;
  IvySendMsg("ME HELLO_WORLD 1234 5678 %d", foo);
}


int main ( int argc, char** argv) {

  gtk_init(&argc, &argv);

  IvyInit ("Example1", "Example1 READY", NULL, NULL, NULL, NULL);
  IvyStart("127.255.255.255");


  GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window), "HelloWorld");

  GtkWidget* button = gtk_toggle_button_new_with_label( "ClickMe" );
  gtk_container_add (GTK_CONTAINER (window), button);
  g_signal_connect (G_OBJECT (button), "clicked", G_CALLBACK (on_button_clicked), NULL);

  gtk_widget_show_all(window);

  gtk_main();
  return 0;

}

