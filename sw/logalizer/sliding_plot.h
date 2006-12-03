#ifndef SLIDING_PLOT_H
#define SLIDING_PLOT_H

#include <gtk/gtk.h>

extern GtkWidget* sliding_plot_new(guint nb_plot);
extern void sliding_plot_update(GtkWidget* plot, float* values);

#endif /* SLIDING_PLOT_H */
