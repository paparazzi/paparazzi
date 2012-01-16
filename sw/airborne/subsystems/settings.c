#include "subsystems/settings.h"


struct PersistentSettings pers_settings;
bool_t settings_store_now;


void settings_init(void) {
#if USE_PERSISTENT_SETTINGS
  if (persistent_read((uint32_t)&pers_settings, sizeof(struct PersistentSettings)))
    return; // return -1 ?
  persistent_settings_load();
#endif
}


void settings_store(void) {
#if USE_PERSISTENT_SETTINGS
  persistent_settings_store();
  persistent_write((uint32_t)&pers_settings, sizeof(struct PersistentSettings));
#endif
}
