#ifndef SUBSYSTEMS_SETTINGS_H
#define SUBSYSTEMS_SETTINGS_H

#include "std.h"

extern void settings_init(void);
extern void settings_store(void);

extern bool_t settings_store_now;

#define settings_StoreSettings(_v) { \
    if (_v) {			     \
      settings_store();		     \
      settings_store_now = FALSE;    \
    }				     \
  }

#include "generated/settings.h"


#endif /* SUBSYSTEMS_SETTINGS_H */
