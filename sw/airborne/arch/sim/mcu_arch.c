
#include "mcu_arch.h"
#include "mcu.h"

#include <stdio.h>
#include <string.h>

void mcu_arch_init(void) {}

/**
 * Identify the PC running the simulation by its machine ID (128 bit),
 * empty if not available (e.g. on macOS)
 */
uint8_t mcu_get_uid(uint8_t *uid, uint8_t max_len)
{
  static const char *paths[] = { "/etc/machine-id", "/var/lib/dbus/machine-id" };
  for (unsigned int p = 0; p < sizeof(paths) / sizeof(paths[0]); p++) {
    char line[40];
    FILE *f = fopen(paths[p], "r");
    if (f == NULL) {
      continue;
    }
    char *ok = fgets(line, sizeof(line), f);
    fclose(f);
    if (ok == NULL || strspn(line, "0123456789abcdef") != 32) {
      continue;
    }
    uint8_t len = Min(16, max_len);
    for (uint8_t i = 0; i < len; i++) {
      unsigned int b;
      sscanf(&line[2 * i], "%2x", &b);
      uid[i] = b;
    }
    return len;
  }
  return 0;
}
