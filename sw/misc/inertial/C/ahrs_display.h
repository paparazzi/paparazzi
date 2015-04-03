#ifndef AHRS_DISPLAY_H
#define AHRS_DISPLAY_H

#include <gtk/gtk.h>

#include "ahrs_data.h"

extern GtkWidget* ahrs_display(struct ahrs_data* td);

#endif /* AHRS_DISPLAY_H */
