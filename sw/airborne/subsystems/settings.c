#include "subsystems/settings.h"


struct PersistentSettings pers_settings;
bool_t settings_store_now;

void settings_init(void) {
  // READ SETTINGS FROM FLASH
  persitent_settings_load();
}


void settings_store(void) {
  persitent_settings_store();
  // WRITE SETTINGS TO FLASH
}

