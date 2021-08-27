#ifndef SETTINGS_H
#define SETTINGS_H

#define NB_SETTING 0
#define DlSetting(_idx, _value) {}
#define PeriodicSendDlValue(_trans, _dev) {}
static inline float settings_get_value(uint8_t i __attribute__((unused))) { return 0.f; }

/* Persistent Settings */
struct PersistentSettings {
  int dummy;
};

extern struct PersistentSettings pers_settings;

static inline void persistent_settings_store( void ) {}

static inline void persistent_settings_load( void ) {}

#endif
