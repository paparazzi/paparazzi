#ifndef SUBSYSTEMS_SETTINGS_H
#define SUBSYSTEMS_SETTINGS_H

#include "std.h"

extern void settings_init(void);
extern void settings_store(void);

extern bool_t settings_store_now;

#define settings_StoreSettings(_v) { settings_store_now = _v; settings_store(); }

#include "generated/settings.h"

/* implemented in arch dependant code */
int32_t persistent_write(uint32_t ptr, uint32_t size);
int32_t persistent_read(uint32_t ptr, uint32_t size);


#endif /* SUBSYSTEMS_SETTINGS_H */
