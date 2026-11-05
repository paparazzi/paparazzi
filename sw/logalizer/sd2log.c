/*
 * Copyright (C) 2026 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file "sw/logalizer/sd2log.c"
 * @author F. van Tienen
 * Extract the .data, .log and (optionally) .tlm files from an airborne SD log
 *
 * Usage: sd2log <SD log> [<output directory>] [--tlm]
 *
 * The SD log (PPRZLOG frames) is decoded with the message definitions the
 * firmware was built with, found in var/conf through the configuration md5 of
 * the ALIVE message, or else with var/messages.xml. The output files are named
 * after the GPS time of the flight.
 */

#define _GNU_SOURCE // nftw, fmemopen, mkdtemp

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <ftw.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <zlib.h>

#define STX_LOG 0x99
#define IO_BUF_SZ (1 << 20)
#define MAX_PAYLOAD 255
#define MAX_MSGS 512
#define MAX_FIELDS 160
#define MAX_NAME 64

typedef enum {
  FT_UNKNOWN = 0,
  FT_UINT8,
  FT_INT8,
  FT_UINT16,
  FT_INT16,
  FT_UINT32,
  FT_INT32,
  FT_UINT64,
  FT_INT64,
  FT_FLOAT,
  FT_DOUBLE,
  FT_CHAR
} field_type_t;

typedef struct {
  char name[MAX_NAME];
  field_type_t base_type;
  bool is_array;
  int fixed_len; /* 0 scalar, -1 variable array, >0 fixed array */
  char format[16];
} field_def_t;

typedef struct {
  int id;
  char name[MAX_NAME];
  int num_fields;
  field_def_t fields[MAX_FIELDS];
} msg_def_t;

typedef struct {
  msg_def_t defs[MAX_MSGS];
  msg_def_t *by_id[256];
  int num_defs;
} msg_class_t;

typedef struct {
  msg_class_t telemetry;
  msg_class_t datalink;
  int gps_id;
  int gps_int_id;
  int alive_id;
} msg_db_t;

typedef struct {
  bool have_start_unix_time;
  double start_unix_time;
  bool have_first_tow;
  int first_tow_s;
  double first_tow_log_t;
  bool have_gps_week;
  int gps_week;
  bool have_md5;
  char md5[33];
  int single_ac_id;
  uint64_t parsed;
  uint64_t skipped;
  uint64_t checksum_errors;
} conv_state_t;

typedef struct {
  bool write_tlm;
  bool debug_detect;
} convert_options_t;

typedef struct {
  uint64_t total_bytes;
  uint64_t processed_bytes;
  uint64_t last_progress_ms;
  bool show_progress;
  bool announced_tow;
  bool announced_week;
  bool announced_md5;
  bool announced_time;
  bool checked_existing_log;
} ui_state_t;

typedef struct {
  char base[160];
  char data_name[200];
  char log_name[200];
  char tlm_name[200];
  char data_path[PATH_MAX];
  char log_path[PATH_MAX];
  char tlm_path[PATH_MAX];
} output_files_t;

static const double gps_epoch_unix = 315964800.0;
static const double gps_leaps[] = {
  46828800.0, 78364801.0, 109900802.0, 173059203.0, 252028804.0,
  315187205.0, 346723206.0, 393984007.0, 425520008.0, 457056009.0,
  504489610.0, 551750411.0, 599184012.0, 820108813.0, 914803214.0,
  1025136015.0, 1119744016.0, 1167264017.0
};

static void build_output_basename(double start_time, const char *suffix, char *out, size_t out_sz);

static int file_exists(const char *p)
{
  struct stat st;
  return stat(p, &st) == 0 && S_ISREG(st.st_mode);
}

static int dir_exists(const char *p)
{
  struct stat st;
  return stat(p, &st) == 0 && S_ISDIR(st.st_mode);
}

/* Ask only when run interactively; otherwise (e.g. from the Paparazzi Center)
 * overwrite like the OCaml version did. */
static bool prompt_yes_no(const char *question)
{
  if (!isatty(STDIN_FILENO) || !isatty(STDERR_FILENO)) {
    fprintf(stderr, "%s no terminal, overwriting\n", question);
    return true;
  }

  fprintf(stderr, "%s [y/N] ", question);
  fflush(stderr);

  char answer[16];
  bool ok = false;
  if (fgets(answer, sizeof(answer), stdin)) {
    ok = (answer[0] == 'y' || answer[0] == 'Y');
  }
  return ok;
}

static bool env_flag_enabled(const char *name)
{
  const char *value = getenv(name);
  return value && value[0] && strcmp(value, "0") != 0;
}

static bool abort_during_parse(uint8_t *buf, FILE *in, FILE *out, const char *tmp_data, msg_db_t *db)
{
  if (buf) free(buf);
  if (in) fclose(in);
  if (out) fclose(out);
  if (tmp_data) unlink(tmp_data);
  free(db);
  return false;
}

static bool abort_after_parse(const char *tmp_data, msg_db_t *db)
{
  if (tmp_data) unlink(tmp_data);
  free(db);
  return false;
}

static void path_join(char *dst, size_t dst_sz, const char *a, const char *b)
{
  if (a == NULL || b == NULL || dst_sz == 0) {
    if (dst_sz > 0) dst[0] = '\0';
    return;
  }
  size_t n = strlen(a);
  if (n > 0 && a[n - 1] == '/') {
    snprintf(dst, dst_sz, "%s%s", a, b);
  } else {
    snprintf(dst, dst_sz, "%s/%s", a, b);
  }
}

static const char *skip_ws(const char *s)
{
  while (*s && isspace((unsigned char)*s)) {
    s++;
  }
  return s;
}

/*
 * Minimal XML reading: the tags of a document one by one, and their attributes.
 * Comments, processing instructions and DOCTYPE are skipped, a '>' inside a quoted
 * attribute value does not end the tag, attributes may use ' or " and spaces around '='.
 */
typedef struct {
  const char *start;  /* first character after '<' */
  size_t len;         /* up to (not including) '>' */
} xml_tag_t;

static bool next_tag(const char **pos, xml_tag_t *tag)
{
  const char *p = *pos;
  while ((p = strchr(p, '<')) != NULL) {
    if (!strncmp(p, "<!--", 4)) {
      const char *e = strstr(p + 4, "-->");
      if (!e) return false;
      p = e + 3;
      continue;
    }
    if (p[1] == '?' || p[1] == '!') {
      const char *e = strchr(p, '>');
      if (!e) return false;
      p = e + 1;
      continue;
    }
    const char *q = p + 1;
    char quote = 0;
    while (*q && (quote || *q != '>')) {
      if (quote) {
        if (*q == quote) quote = 0;
      } else if (*q == '"' || *q == '\'') {
        quote = *q;
      }
      q++;
    }
    if (!*q) return false;
    tag->start = p + 1;
    tag->len = (size_t)(q - p - 1);
    *pos = q + 1;
    return true;
  }
  return false;
}

/* True when the tag is an element with this name (e.g. "message" or "/message") */
static bool tag_is(const xml_tag_t *tag, const char *name)
{
  size_t n = strlen(name);
  return tag->len >= n && !strncmp(tag->start, name, n)
         && (tag->len == n || isspace((unsigned char)tag->start[n]) || tag->start[n] == '/');
}

static bool get_attr(const xml_tag_t *tag, const char *key, char *out, size_t out_sz)
{
  const char *p = tag->start;
  const char *end = tag->start + tag->len;
  size_t key_len = strlen(key);
  /* skip the element name */
  while (p < end && !isspace((unsigned char)*p) && *p != '/') p++;
  while (p < end) {
    while (p < end && (isspace((unsigned char)*p) || *p == '/')) p++;
    const char *name = p;
    while (p < end && !isspace((unsigned char)*p) && *p != '=' && *p != '/') p++;
    size_t name_len = (size_t)(p - name);
    while (p < end && isspace((unsigned char)*p)) p++;
    if (p >= end || *p != '=') {
      if (name_len == 0) break;
      continue; /* attribute without value */
    }
    p++;
    while (p < end && isspace((unsigned char)*p)) p++;
    if (p >= end || (*p != '"' && *p != '\'')) break;
    char quote = *p++;
    const char *v = p;
    while (p < end && *p != quote) p++;
    if (p >= end) break;
    if (name_len == key_len && !strncmp(name, key, key_len)) {
      size_t n = (size_t)(p - v);
      if (n >= out_sz) n = out_sz - 1;
      memcpy(out, v, n);
      out[n] = '\0';
      return true;
    }
    p++;
  }
  return false;
}

/* Keep a format attribute only when it is exactly one floating point conversion
 * (e.g. "%.2f"), so it can be given to fprintf with a double and gives one value */
static void check_float_format(char *format)
{
  const char *p = format;
  if (*p++ != '%') { format[0] = '\0'; return; }
  while (*p && strchr("-+ #0", *p)) p++;
  while (isdigit((unsigned char)*p)) p++;
  if (*p == '.') {
    p++;
    while (isdigit((unsigned char)*p)) p++;
  }
  if (!*p || !strchr("eEfFgGaA", *p) || p[1] != '\0') {
    format[0] = '\0';
  }
}

static field_type_t parse_base_type(const char *t)
{
  if (!strcmp(t, "uint8") || !strcmp(t, "bool")) return FT_UINT8;
  if (!strcmp(t, "int8")) return FT_INT8;
  if (!strcmp(t, "uint16")) return FT_UINT16;
  if (!strcmp(t, "int16")) return FT_INT16;
  if (!strcmp(t, "uint32") || !strcmp(t, "timestamp")) return FT_UINT32;
  if (!strcmp(t, "int32")) return FT_INT32;
  if (!strcmp(t, "uint64")) return FT_UINT64;
  if (!strcmp(t, "int64")) return FT_INT64;
  if (!strcmp(t, "float")) return FT_FLOAT;
  if (!strcmp(t, "double")) return FT_DOUBLE;
  if (!strcmp(t, "char")) return FT_CHAR;
  return FT_UNKNOWN;
}

static void parse_field_type(const char *type_str, field_def_t *f)
{
  f->is_array = false;
  f->fixed_len = 0;
  f->base_type = FT_UNKNOWN;
  f->format[0] = '\0';

  char tmp[64];
  snprintf(tmp, sizeof(tmp), "%s", type_str);
  char *lb = strchr(tmp, '[');
  if (lb) {
    f->is_array = true;
    char *rb = strchr(lb, ']');
    if (rb) {
      *lb = '\0';
      *rb = '\0';
      f->fixed_len = lb[1] ? atoi(lb + 1) : -1;
    }
  }
  f->base_type = parse_base_type(tmp);
}

static msg_def_t *add_msg(msg_class_t *c, int id, const char *name)
{
  if (id < 0 || id > 255 || c->num_defs >= MAX_MSGS) return NULL;
  msg_def_t *m = &c->defs[c->num_defs++];
  m->id = id;
  m->num_fields = 0;
  snprintf(m->name, sizeof(m->name), "%s", name);
  c->by_id[id] = m;
  return m;
}

static bool read_conf_maybe_gz(const char *path, char **out);

/* Load the telemetry and datalink definitions from a messages.xml, or from the
 * <protocol> part of a saved aircraft configuration (var/conf/...conf.gz) */
static bool load_messages_xml(const char *xml_path, msg_db_t *db)
{
  memset(db, 0, sizeof(*db));
  db->gps_id = -1;
  db->gps_int_id = -1;
  db->alive_id = -1;

  char *text = NULL;
  if (!read_conf_maybe_gz(xml_path, &text)) return false;

  msg_class_t *klass = NULL;
  msg_def_t *msg = NULL;

  const char *pos = text;
  xml_tag_t tag;
  while (next_tag(&pos, &tag)) {
    if (tag_is(&tag, "msg_class")) {
      char cls[MAX_NAME] = {0};
      klass = NULL;
      if (get_attr(&tag, "name", cls, sizeof(cls))) {
        if (!strcmp(cls, "telemetry")) klass = &db->telemetry;
        else if (!strcmp(cls, "datalink")) klass = &db->datalink;
      }
      msg = NULL;
    } else if (tag_is(&tag, "/msg_class")) {
      klass = NULL;
      msg = NULL;
    } else if (klass && tag_is(&tag, "message")) {
      char name[MAX_NAME] = {0};
      char id_str[32] = {0};
      msg = NULL;
      if (get_attr(&tag, "name", name, sizeof(name)) && get_attr(&tag, "id", id_str, sizeof(id_str))) {
        int id = atoi(id_str);
        msg = add_msg(klass, id, name);
        if (klass == &db->telemetry) {
          if (!strcmp(name, "GPS")) db->gps_id = id;
          else if (!strcmp(name, "GPS_INT")) db->gps_int_id = id;
          else if (!strcmp(name, "ALIVE")) db->alive_id = id;
        }
      }
    } else if (tag_is(&tag, "/message")) {
      msg = NULL;
    } else if (msg && tag_is(&tag, "field")) {
      if (msg->num_fields >= MAX_FIELDS) continue;
      char fn[MAX_NAME] = {0};
      char ft[64] = {0};
      if (get_attr(&tag, "name", fn, sizeof(fn)) && get_attr(&tag, "type", ft, sizeof(ft))) {
        field_def_t *fd = &msg->fields[msg->num_fields++];
        snprintf(fd->name, sizeof(fd->name), "%s", fn);
        parse_field_type(ft, fd);
        get_attr(&tag, "format", fd->format, sizeof(fd->format));
        check_float_format(fd->format);
      }
    }
  }

  free(text);
  return db->telemetry.num_defs > 0;
}

static msg_def_t *find_msg(msg_db_t *db, uint8_t source, uint8_t msg_id)
{
  if (source == 0) return db->telemetry.by_id[msg_id];
  if (source == 1) return db->datalink.by_id[msg_id];
  return NULL;
}

static int gps_count_leaps(double gps_time)
{
  int c = 0;
  for (size_t i = 0; i < sizeof(gps_leaps) / sizeof(gps_leaps[0]); i++) {
    if (gps_time >= gps_leaps[i]) c++;
    else break;
  }
  return c;
}

static double unix_time_of_tow_week(int tow_s, int week)
{
  double gps_seconds = gps_epoch_unix + (double)week * 7.0 * 24.0 * 3600.0 + (double)tow_s;
  return gps_seconds - (double)gps_count_leaps(gps_seconds);
}

static int gps_tow_of_utc(int wday, int hour, int min, int sec)
{
  /* Keep parity with sw/lib/ocaml/latlong.ml leap_seconds constant. */
  const int leap_seconds = 16;
  return ((wday * 24 + hour) * 60 + min) * 60 + sec + leap_seconds;
}

static int host_gps_tow_now(void)
{
  time_t now = time(NULL);
  struct tm utc;
  gmtime_r(&now, &utc);
  return gps_tow_of_utc(utc.tm_wday, utc.tm_hour, utc.tm_min, utc.tm_sec);
}

static double unix_time_of_tow_no_week(int tow_s)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  int host_tow = host_gps_tow_now();
  double now = (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
  double guess = now + (double)(tow_s - host_tow);

  /* When we have no GPS week, prefer the most recent occurrence of this TOW. */
  while (guess > now) {
    guess -= 7.0 * 24.0 * 3600.0;
  }

  return guess;
}

static bool rd_u16(const uint8_t *b, size_t n, size_t *o, uint16_t *v)
{
  if (*o + 2 > n) return false;
  *v = (uint16_t)b[*o] | ((uint16_t)b[*o + 1] << 8);
  *o += 2;
  return true;
}

static bool rd_u32(const uint8_t *b, size_t n, size_t *o, uint32_t *v)
{
  if (*o + 4 > n) return false;
  *v = (uint32_t)b[*o] | ((uint32_t)b[*o + 1] << 8) | ((uint32_t)b[*o + 2] << 16) | ((uint32_t)b[*o + 3] << 24);
  *o += 4;
  return true;
}

static bool print_scalar(FILE *out, field_type_t t, const uint8_t *buf, size_t len, size_t *off,
                         int64_t *i64, uint64_t *u64, bool leading_space, const char *format)
{
  *i64 = 0;
  *u64 = 0;
  const char *prefix = leading_space ? " " : "";

  switch (t) {
    case FT_UINT8: {
      if (*off + 1 > len) return false;
      uint8_t v = buf[(*off)++];
      *u64 = v; *i64 = v;
      fprintf(out, "%s%u", prefix, (unsigned)v);
      return true;
    }
    case FT_INT8: {
      if (*off + 1 > len) return false;
      int8_t v = (int8_t)buf[(*off)++];
      *i64 = v;
      fprintf(out, "%s%d", prefix, (int)v);
      return true;
    }
    case FT_CHAR: {
      if (*off + 1 > len) return false;
      unsigned char v = buf[(*off)++];
      if (isprint(v) && v != ' ') fprintf(out, "%s%c", prefix, v);
      else fprintf(out, "%s0x%02x", prefix, v);
      *u64 = v; *i64 = v;
      return true;
    }
    case FT_UINT16: {
      uint16_t v;
      if (!rd_u16(buf, len, off, &v)) return false;
      *u64 = v; *i64 = v;
      fprintf(out, "%s%u", prefix, (unsigned)v);
      return true;
    }
    case FT_INT16: {
      uint16_t u;
      if (!rd_u16(buf, len, off, &u)) return false;
      int16_t v;
      memcpy(&v, &u, sizeof(v));
      *i64 = v;
      fprintf(out, "%s%d", prefix, (int)v);
      return true;
    }
    case FT_UINT32: {
      uint32_t v;
      if (!rd_u32(buf, len, off, &v)) return false;
      *u64 = v; *i64 = (int64_t)v;
      fprintf(out, "%s%" PRIu32, prefix, v);
      return true;
    }
    case FT_INT32: {
      uint32_t u;
      if (!rd_u32(buf, len, off, &u)) return false;
      int32_t v;
      memcpy(&v, &u, sizeof(v));
      *i64 = v;
      fprintf(out, "%s%" PRId32, prefix, v);
      return true;
    }
    case FT_FLOAT: {
      uint32_t u;
      if (!rd_u32(buf, len, off, &u)) return false;
      float v;
      memcpy(&v, &u, sizeof(v));
      fprintf(out, "%s", prefix);
      fprintf(out, format && format[0] ? format : "%.6f", (double)v);
      return true;
    }
    case FT_UINT64:
    case FT_INT64:
    case FT_DOUBLE: {
      uint32_t lo, hi;
      if (!rd_u32(buf, len, off, &lo) || !rd_u32(buf, len, off, &hi)) return false;
      uint64_t u = (uint64_t)lo | ((uint64_t)hi << 32);
      fprintf(out, "%s", prefix);
      if (t == FT_UINT64) {
        *u64 = u; *i64 = (int64_t)u;
        fprintf(out, "%" PRIu64, u);
      } else if (t == FT_INT64) {
        int64_t v;
        memcpy(&v, &u, sizeof(v));
        *i64 = v;
        fprintf(out, "%" PRId64, v);
      } else {
        double v;
        memcpy(&v, &u, sizeof(v));
        fprintf(out, format && format[0] ? format : "%.6f", v);
      }
      return true;
    }
    case FT_UNKNOWN:
    default:
      return false;
  }
}

/* Empty value for a field that is missing from the payload */
static void print_default(FILE *out, const field_def_t *fd)
{
  if (fd->is_array && (fd->base_type == FT_CHAR || fd->fixed_len <= 0)) {
    fprintf(out, " \"\"");
    return;
  }
  const char *zero = (fd->base_type == FT_FLOAT || fd->base_type == FT_DOUBLE) ? "0.000000" : "0";
  int n = fd->is_array ? fd->fixed_len : 1;
  for (int j = 0; j < n; j++) {
    fprintf(out, "%s%s", j == 0 ? " " : ",", zero);
  }
}

/* Decode the fields of a message. A payload that ends exactly between two fields
 * (firmware built with fewer fields at the end) gets empty values for the
 * missing fields; extra bytes at the end (firmware with more fields) are ignored.
 * Returns false when the payload ends inside a field. */
static bool decode_payload(FILE *out, const msg_def_t *msg, const uint8_t *payload, size_t payload_len,
                           int *gps_mode, int *gps_week, int *gps_itow,
                           int *gps_fix, int *gps_tow,
                           char md5_hex[33])
{
  size_t off = 0;
  if (md5_hex) md5_hex[0] = '\0';

  for (int i = 0; i < msg->num_fields; i++) {
    const field_def_t *fd = &msg->fields[i];
    int n = 1;

    if (off == payload_len && i > 0) {
      print_default(out, fd);
      continue;
    }

    if (fd->is_array) {
      if (fd->fixed_len > 0) n = fd->fixed_len;
      else {
        if (off >= payload_len) return false;
        n = payload[off++];
      }
    }

    if (fd->is_array && n == 0) {
      /* Keep the field position for an empty array */
      fprintf(out, " \"\"");
      continue;
    }

    if (fd->is_array && fd->base_type == FT_CHAR) {
      fprintf(out, " \"");
      /* raw characters as the OCaml sd2log, up to the first NUL (padding);
       * quotes and control characters would break the line, they become '?' */
      bool ended = false;
      for (int j = 0; j < n; j++) {
        if (off >= payload_len) return false;
        unsigned char c = payload[off++];
        if (c == '\0') ended = true;
        if (ended) continue;
        fputc((c >= 0x20 && c != 0x7f && c != '"') ? c : '?', out);
      }
      fprintf(out, "\"");
      continue;
    }

    if (fd->is_array) {
      for (int j = 0; j < n; j++) {
        int64_t i64;
        uint64_t u64;
        size_t prev = off;
        if (!print_scalar(out, fd->base_type, payload, payload_len, &off, &i64, &u64, j == 0, fd->format)) {
          off = prev;
          return false;
        }
        if (j + 1 < n) fprintf(out, ",");

        if (md5_hex && !strcmp(fd->name, "md5sum") && fd->base_type == FT_UINT8 && j < 16) {
          snprintf(&md5_hex[j * 2], 3, "%02x", (unsigned)u64);
        }
      }
      continue;
    }

    int64_t i64;
    uint64_t u64;
    if (!print_scalar(out, fd->base_type, payload, payload_len, &off, &i64, &u64, true, fd->format)) {
      return false;
    }

    if (gps_mode && !strcmp(fd->name, "mode")) *gps_mode = (int)u64;
    if (gps_week && !strcmp(fd->name, "week")) *gps_week = (int)u64;
    if (gps_itow && !strcmp(fd->name, "itow")) *gps_itow = (int)u64;
    if (gps_fix && !strcmp(fd->name, "fix")) *gps_fix = (int)u64;
    if (gps_tow && !strcmp(fd->name, "tow")) *gps_tow = (int)u64;
  }

  return true;
}

static bool copy_file_fast(const char *src, const char *dst)
{
  FILE *in = fopen(src, "rb");
  if (!in) return false;
  FILE *out = fopen(dst, "wb");
  if (!out) {
    fclose(in);
    return false;
  }
  setvbuf(in, NULL, _IOFBF, IO_BUF_SZ);
  setvbuf(out, NULL, _IOFBF, IO_BUF_SZ);

  uint8_t *buf = malloc(IO_BUF_SZ);
  if (!buf) {
    fclose(in);
    fclose(out);
    return false;
  }

  bool ok = true;
  while (!feof(in)) {
    size_t n = fread(buf, 1, IO_BUF_SZ, in);
    if (n > 0 && fwrite(buf, 1, n, out) != n) {
      ok = false;
      break;
    }
    if (ferror(in)) {
      ok = false;
      break;
    }
  }

  free(buf);
  fclose(in);
  if (fclose(out) != 0) ok = false;
  return ok;
}

static uint64_t monotonic_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static void print_progress_bar(uint64_t done, uint64_t total, uint64_t frames, bool finish_line)
{
  if (total == 0) return;

  const int width = 36;
  if (done > total) done = total;
  double pct = 100.0 * (double)done / (double)total;
  int filled = (int)((double)done * (double)width / (double)total);
  if (filled > width) filled = width;

  fputc('\r', stderr);
  fputc('[', stderr);
  for (int i = 0; i < width; i++) {
    fputc(i < filled ? '#' : '-', stderr);
  }
  fprintf(stderr, "] %6.2f%%  %" PRIu64 " frames", pct, frames);
  if (finish_line) {
    fputc('\n', stderr);
  }
  fflush(stderr);
}

static void ui_state_init(ui_state_t *ui, FILE *in)
{
  memset(ui, 0, sizeof(*ui));

  struct stat in_st;
  if (fstat(fileno(in), &in_st) == 0 && in_st.st_size > 0 && isatty(STDERR_FILENO)) {
    ui->total_bytes = (uint64_t)in_st.st_size;
    ui->show_progress = true;
    ui->last_progress_ms = monotonic_ms();
    print_progress_bar(0, ui->total_bytes, 0, false);
  }
}

static void ui_state_update_progress(ui_state_t *ui, uint64_t bytes_read, uint64_t frames)
{
  if (!ui->show_progress || bytes_read == 0) {
    return;
  }

  ui->processed_bytes += bytes_read;
  uint64_t now_ms = monotonic_ms();
  if (now_ms - ui->last_progress_ms >= 200 || ui->processed_bytes >= ui->total_bytes) {
    print_progress_bar(ui->processed_bytes, ui->total_bytes, frames, false);
    ui->last_progress_ms = now_ms;
  }
}

static void ui_state_finish_progress(const ui_state_t *ui, uint64_t frames)
{
  if (ui->show_progress) {
    print_progress_bar(ui->total_bytes, ui->total_bytes, frames, true);
  }
}

static void ui_state_redraw_progress(const ui_state_t *ui, uint64_t frames)
{
  if (ui->show_progress && ui->processed_bytes < ui->total_bytes) {
    print_progress_bar(ui->processed_bytes, ui->total_bytes, frames, false);
  }
}

static void build_output_files(const char *logs, double start_time, const char *suffix, output_files_t *files)
{
  build_output_basename(start_time, suffix, files->base, sizeof(files->base));
  snprintf(files->data_name, sizeof(files->data_name), "%s.data", files->base);
  snprintf(files->log_name, sizeof(files->log_name), "%s.log", files->base);
  snprintf(files->tlm_name, sizeof(files->tlm_name), "%s.tlm", files->base);
  path_join(files->data_path, sizeof(files->data_path), logs, files->data_name);
  path_join(files->log_path, sizeof(files->log_path), logs, files->log_name);
  path_join(files->tlm_path, sizeof(files->tlm_path), logs, files->tlm_name);
}

static bool maybe_prompt_existing_log(const char *logs, const char *base, bool show_progress)
{
  char log_name[200];
  char log_path[PATH_MAX];
  char question[PATH_MAX + 320];

  snprintf(log_name, sizeof(log_name), "%s.log", base);
  path_join(log_path, sizeof(log_path), logs, log_name);
  if (!file_exists(log_path)) {
    return true;
  }

  if (show_progress) {
    fputc('\n', stderr);
  }
  snprintf(question, sizeof(question), "Output %s.log already exists. Continue and overwrite?", base);
  if (prompt_yes_no(question)) {
    return true;
  }

  fprintf(stderr, "Aborted: existing outputs were left untouched\n");
  return false;
}

static bool maybe_prompt_existing_log_for_time(double start_unix_time, const char *logs, bool show_progress)
{
  char base[160];
  build_output_basename(start_unix_time, "", base, sizeof(base));
  return maybe_prompt_existing_log(logs, base, show_progress);
}

static bool maybe_announce_detected_time(const convert_options_t *options, ui_state_t *ui,
                                         const conv_state_t *st, const char *logs,
                                         uint8_t *buf, FILE *in, FILE *out,
                                         const char *tmp_data, msg_db_t *db)
{
  char preview[160];
  bool printed_detect = false;

  if (!st->have_start_unix_time) {
    return true;
  }

  build_output_basename(st->start_unix_time, "", preview, sizeof(preview));
  if (options->debug_detect && !ui->announced_time) {
    if (ui->show_progress) fputc('\n', stderr);
    fprintf(stderr, "[detect] start_time=%.2f -> output=%s.[data,log%s]\n",
            st->start_unix_time, preview, options->write_tlm ? ",tlm" : "");
    ui->announced_time = true;
    printed_detect = true;
  }

  if (!ui->checked_existing_log) {
    ui->checked_existing_log = true;
    if (!maybe_prompt_existing_log(logs, preview, ui->show_progress && !printed_detect)) {
      return abort_during_parse(buf, in, out, tmp_data, db);
    }
  }

  if (printed_detect) {
    ui_state_redraw_progress(ui, st->parsed);
  }
  return true;
}

static void maybe_announce_detect_messages(const convert_options_t *options, ui_state_t *ui,
                                           const conv_state_t *st)
{
  if (!options->debug_detect) {
    return;
  }

  bool printed_detect = false;

  if (!ui->announced_tow && st->have_first_tow) {
    if (ui->show_progress) fputc('\n', stderr);
    fprintf(stderr, "[detect] first_tow=%ds at log_t=%.4f\n", st->first_tow_s, st->first_tow_log_t);
    ui->announced_tow = true;
    printed_detect = true;
  }
  if (!ui->announced_week && st->have_gps_week) {
    if (ui->show_progress) fputc('\n', stderr);
    fprintf(stderr, "[detect] gps_week=%d\n", st->gps_week);
    ui->announced_week = true;
    printed_detect = true;
  }
  if (!ui->announced_md5 && st->have_md5) {
    if (ui->show_progress) fputc('\n', stderr);
    fprintf(stderr, "[detect] md5=%s\n", st->md5);
    ui->announced_md5 = true;
    printed_detect = true;
  }
  if (printed_detect) {
    ui_state_redraw_progress(ui, st->parsed);
  }
}

static bool maybe_set_fallback_start_time(const convert_options_t *options, ui_state_t *ui,
                                          conv_state_t *st, const char *logs,
                                          const char *tmp_data, msg_db_t *db)
{
  if (st->have_start_unix_time || !st->have_first_tow) {
    return true;
  }

  st->start_unix_time = unix_time_of_tow_no_week(st->first_tow_s) - st->first_tow_log_t;
  st->have_start_unix_time = true;

  if (options->debug_detect && !ui->announced_time) {
    char preview[160];
    build_output_basename(st->start_unix_time, "", preview, sizeof(preview));
    fprintf(stderr, "[detect] start_time=%.2f (no_week) -> output=%s.[data,log%s]\n",
            st->start_unix_time, preview, options->write_tlm ? ",tlm" : "");
    ui->announced_time = true;
  }

  if (!ui->checked_existing_log) {
    ui->checked_existing_log = true;
    if (!maybe_prompt_existing_log_for_time(st->start_unix_time, logs, false)) {
      return abort_after_parse(tmp_data, db);
    }
  }

  return true;
}

static void build_output_basename(double start_time, const char *suffix, char *out, size_t out_sz)
{
  time_t tt = (time_t)start_time;
  struct tm tmv;
  localtime_r(&tt, &tmv);
  snprintf(out, out_sz, "%02d_%02d_%02d__%02d_%02d_%02d_SD%s",
           (tmv.tm_year + 1900) % 100, tmv.tm_mon + 1, tmv.tm_mday,
           tmv.tm_hour, tmv.tm_min, tmv.tm_sec, suffix ? suffix : "");
}

static bool read_conf_maybe_gz(const char *path, char **out)
{
  *out = NULL;
  size_t n = strlen(path);

  if (n >= 3 && !strcmp(path + n - 3, ".gz")) {
    gzFile z = gzopen(path, "rb");
    if (!z) return false;

    size_t cap = 1 << 16;
    size_t len = 0;
    char *buf = malloc(cap);
    if (!buf) {
      gzclose(z);
      return false;
    }

    while (1) {
      if (len + 4096 >= cap) {
        cap *= 2;
        char *tmp = realloc(buf, cap);
        if (!tmp) {
          free(buf);
          gzclose(z);
          return false;
        }
        buf = tmp;
      }
      int r = gzread(z, buf + len, (unsigned int)(cap - len - 1));
      if (r < 0) {
        free(buf);
        gzclose(z);
        return false;
      }
      if (r == 0) break;
      len += (size_t)r;
    }

    gzclose(z);
    buf[len] = '\0';
    *out = buf;
    return true;
  }

  FILE *f = fopen(path, "rb");
  if (!f) return false;
  fseek(f, 0, SEEK_END);
  long sz = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (sz <= 0) {
    fclose(f);
    return false;
  }
  char *buf = malloc((size_t)sz + 1);
  if (!buf) {
    fclose(f);
    return false;
  }
  if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) {
    free(buf);
    fclose(f);
    return false;
  }
  fclose(f);
  buf[sz] = '\0';
  *out = buf;
  return true;
}

/* Get an attribute (e.g. "name" or "airframe") of the aircraft with ac_id in a conf.xml */
static bool lookup_aircraft_attr(const char *conf_xml_path, int ac_id, const char *attr, char *out, size_t out_sz)
{
  char *xml = NULL;
  if (!read_conf_maybe_gz(conf_xml_path, &xml)) {
    return false;
  }

  bool found = false;
  const char *pos = xml;
  xml_tag_t tag;
  while (!found && next_tag(&pos, &tag)) {
    char id_str[32];
    if (tag_is(&tag, "aircraft") && get_attr(&tag, "ac_id", id_str, sizeof(id_str)) && atoi(id_str) == ac_id) {
      found = get_attr(&tag, attr, out, out_sz);
      break;
    }
  }

  free(xml);
  return found;
}

/* Temporary directory of the generated aircraft configuration, removed at exit */
static char gen_tmp_dir[PATH_MAX - 64];

static int remove_entry(const char *path, const struct stat *sb, int flag, struct FTW *ftw)
{
  (void)sb; (void)flag; (void)ftw;
  remove(path);
  return 0;
}

static void remove_gen_tmp_dir(void)
{
  if (gen_tmp_dir[0]) {
    nftw(gen_tmp_dir, remove_entry, 16, FTW_DEPTH | FTW_PHYS);
    gen_tmp_dir[0] = '\0';
  }
}

/* Generate the configuration of an aircraft of the current conf.xml with the OCaml generator,
 * in a temporary directory (the build configuration in var/aircrafts is not changed) */
static bool generate_conf_aircraft_with_ocaml(const char *home, const char *conf_xml_path,
                                               const char *aircraft_name,
                                               char *out_conf_path, size_t out_sz)
{
  if (!home || !home[0] || !conf_xml_path || !conf_xml_path[0] || !aircraft_name || !aircraft_name[0]) {
    return false;
  }

  const char *tmp = getenv("TMPDIR");
  snprintf(gen_tmp_dir, sizeof(gen_tmp_dir), "%s/sd2log_XXXXXX", tmp && tmp[0] ? tmp : "/tmp");
  if (!mkdtemp(gen_tmp_dir)) {
    gen_tmp_dir[0] = '\0';
    return false;
  }
  atexit(remove_gen_tmp_dir);

  char generator[PATH_MAX];
  snprintf(generator, sizeof(generator), "%s/sw/tools/generators/gen_aircraft.out", home);
  char *args[] = { generator, "-ac_conf", "-all", "-name", (char *)aircraft_name, "-target", "ap",
                   "-conf", (char *)conf_xml_path, "-conf_dir", gen_tmp_dir, "-ac_dir", gen_tmp_dir, NULL };

  fflush(NULL);
  pid_t pid = fork();
  if (pid < 0) {
    return false;
  }
  if (pid == 0) {
    setenv("PAPARAZZI_HOME", home, 1);
    setenv("PAPARAZZI_SRC", home, 1);
    int devnull = open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      dup2(devnull, STDOUT_FILENO);
      dup2(devnull, STDERR_FILENO);
    }
    execv(generator, args);
    _exit(127);
  }
  int status = 0;
  if (waitpid(pid, &status, 0) < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    return false;
  }

  char gen_path[PATH_MAX];
  snprintf(gen_path, sizeof(gen_path), "%s/conf/conf_aircraft.xml", gen_tmp_dir);
  if (!file_exists(gen_path)) {
    return false;
  }

  snprintf(out_conf_path, out_sz, "%s", gen_path);
  return true;
}

static void fputs_indented(FILE *out, const char *text, const char *indent)
{
  bool at_line_start = true;
  size_t indent_len = strlen(indent);
  for (const char *p = text; *p; p++) {
    if (at_line_start) {
      fwrite(indent, 1, indent_len, out);
      at_line_start = false;
    }
    fputc(*p, out);
    if (*p == '\n') {
      at_line_start = true;
    }
  }
}

/* Skip the XML declaration, DOCTYPE and leading comments, so a file can be embedded */
static const char *skip_prolog(const char *xml)
{
  const char *p = xml;
  while (1) {
    p = skip_ws(p);
    const char *end = NULL;
    if (!strncmp(p, "<?", 2)) end = strstr(p, "?>");
    else if (!strncmp(p, "<!--", 4)) end = strstr(p, "-->");
    else if (!strncmp(p, "<!", 2)) end = strchr(p, '>');
    else return p;
    if (!end) return p;
    p = (*end == '>') ? end + 1 : end + (end[1] == '>' ? 2 : 3);
  }
}

/* Write a file embedded in the .log, indented and without its prolog */
static void write_embedded(FILE *out, const char *xml, const char *indent)
{
  const char *body = skip_prolog(xml);
  fputs_indented(out, body, indent);
  size_t n = strlen(body);
  if (n > 0 && body[n - 1] != '\n') {
    fputc('\n', out);
  }
}

/* Message definitions, as the OCaml version always included them in the .log */
static void write_messages(FILE *out, const char *msg_xml_path)
{
  char *xml = NULL;
  if (msg_xml_path && read_conf_maybe_gz(msg_xml_path, &xml)) {
    write_embedded(out, xml, "  ");
    free(xml);
  }
}

static bool write_log_from_conf(const char *log_path, const char *conf_path, const char *airframe_path,
                                const char *msg_xml_path, double start_time, const char *data_name)
{
  FILE *out = fopen(log_path, "wb");
  if (!out) return false;

  char conf_xml_path[PATH_MAX];
  const char *home = getenv("PAPARAZZI_HOME");
  if (!home || !home[0]) home = ".";
  snprintf(conf_xml_path, sizeof(conf_xml_path), "%s/conf/conf.xml", home);

  if (conf_path) {
    char *xml = NULL;
    if (read_conf_maybe_gz(conf_path, &xml)) {
      const char *cfg = strstr(xml, "<configuration");
      if (cfg) {
        const char *gt = strchr(cfg, '>');
        if (gt) {
          size_t pre = (size_t)(cfg - xml);
          fwrite(xml, 1, pre, out);

          size_t tag_len = (size_t)(gt - cfg + 1);
          if (tag_len >= 2 && cfg[tag_len - 2] == '/') {
            fwrite(cfg, 1, tag_len - 2, out);
            fprintf(out, " data_file=\"%s\" time_of_day=\"%.2f\"/>", data_name, start_time);
          } else {
            fwrite(cfg, 1, tag_len - 1, out);
            fprintf(out, " data_file=\"%s\" time_of_day=\"%.2f\">", data_name, start_time);
          }

          fputs(gt + 1, out);
          free(xml);
          fclose(out);
          return true;
        }
      }
      fputs(xml, out);
      free(xml);
      fclose(out);
      return true;
    }
  }

  if (airframe_path) {
    char *xml = NULL;
    if (read_conf_maybe_gz(airframe_path, &xml)) {
      fprintf(out, "<configuration data_file=\"%s\" time_of_day=\"%.2f\">\n", data_name, start_time);
      fputs("  <conf>\n", out);
      write_embedded(out, xml, "    ");
      fputs("  </conf>\n", out);
      write_messages(out, msg_xml_path);
      fputs("</configuration>\n", out);
      free(xml);
      fclose(out);
      return true;
    }
  }

  if (file_exists(conf_xml_path)) {
    char *xml = NULL;
    if (read_conf_maybe_gz(conf_xml_path, &xml)) {
      fprintf(out, "<configuration data_file=\"%s\" time_of_day=\"%.2f\">\n", data_name, start_time);
      write_embedded(out, xml, "  ");
      write_messages(out, msg_xml_path);
      fputs("</configuration>\n", out);
      free(xml);
      fclose(out);
      return true;
    }
  }

  fprintf(out,
      "<configuration data_file=\"%s\" time_of_day=\"%.2f\">\n"
          "  <description>Generated by C sd2log fallback</description>\n",
      data_name, start_time);
  write_messages(out, msg_xml_path);
  fputs("</configuration>\n", out);
  fclose(out);
  return true;
}

static bool find_conf_by_md5(const char *conf_dir, const char *md5, char *out_path, size_t out_sz)
{
  DIR *d = opendir(conf_dir);
  if (!d) return false;

  const size_t md5_ofs = 3 * 6 + 1;
  const size_t md5_len = 32;
  bool found = false;

  struct dirent *ent;
  while ((ent = readdir(d)) != NULL) {
    size_t n = strlen(ent->d_name);
    if (n > md5_ofs + md5_len && !strncmp(ent->d_name + md5_ofs, md5, md5_len)) {
      path_join(out_path, out_sz, conf_dir, ent->d_name);
      found = true;
      break;
    }
  }

  closedir(d);
  return found;
}

static void handle_message(FILE *out, msg_db_t *db, conv_state_t *st,
                           uint8_t source, uint32_t ts100us,
                           const uint8_t *payload, uint8_t len)
{
  st->parsed++;

  if (source > 1 || len < 4) {
    st->skipped++;
    return;
  }

  uint8_t ac_id = payload[0];
  uint8_t msg_id = payload[3];
  const uint8_t *msg_payload = payload + 4;
  size_t msg_len = (size_t)(len - 4);

  if (source == 0 && st->single_ac_id < 0) {
    st->single_ac_id = (int)ac_id;
  }
  if (source == 0 && st->single_ac_id >= 0 && st->single_ac_id != (int)ac_id) {
    st->skipped++;
    return;
  }

  msg_def_t *m = find_msg(db, source, msg_id);
  bool is_tm = (source == 0);
  bool is_gps = is_tm && msg_id == db->gps_id;
  bool is_gps_int = is_tm && msg_id == db->gps_int_id;
  bool is_alive = is_tm && msg_id == db->alive_id;

  char name[96];
  if (m) snprintf(name, sizeof(name), "%s", m->name);
  else snprintf(name, sizeof(name), "%s_%u", source == 0 ? "TM" : "DL", (unsigned)msg_id);

  double t = (double)ts100us / 10000.0;

  /* Decode into a line buffer first, so a message that doesn't match its
   * definition is skipped as a whole instead of being written half decoded. */
  static char line_buf[16384];
  static FILE *line = NULL;
  if (!line) {
    line = fmemopen(line_buf, sizeof(line_buf), "w");
    if (!line) {
      st->skipped++;
      return;
    }
  }
  rewind(line);
  fprintf(line, "%.4f %u %s", t, (unsigned)ac_id, name);

  int gps_mode = -1, gps_week = -1, gps_itow = -1, gps_fix = -1, gps_tow = -1;
  char md5_hex[33] = {0};

  /* A message without definition can't be described in the .log, it is skipped
   * (its raw bytes are still in the .tlm with --tlm) */
  bool ok = m && decode_payload(line, m, msg_payload, msg_len,
                                &gps_mode, &gps_week, &gps_itow,
                                &gps_fix, &gps_tow,
                                md5_hex);
  fflush(line);
  long line_len = ftell(line);
  if (!ok || line_len < 0 || (size_t)line_len >= sizeof(line_buf) - 1) {
    st->skipped++;
    return;
  }
  fwrite(line_buf, 1, (size_t)line_len, out);
  fputc('\n', out);

  if (is_tm && !st->have_start_unix_time) {
    /* Keep the first valid TOW as the anchor (GPS or GPS_INT). */
    if (!st->have_first_tow) {
      if (is_gps && (gps_mode == 3 || gps_week > 0) && gps_itow >= 0) {
        st->first_tow_s = gps_itow / 1000;
        st->first_tow_log_t = t;
        st->have_first_tow = true;
      } else if (is_gps_int && gps_fix >= 3 && gps_tow >= 0) {
        st->first_tow_s = gps_tow / 1000;
        st->first_tow_log_t = t;
        st->have_first_tow = true;
      }
    }

    /* Accept GPS week when it becomes available, even after first TOW. */
    if (!st->have_gps_week && is_gps && (gps_mode == 3 || gps_week > 0) && gps_week >= 0) {
      st->gps_week = gps_week;
      st->have_gps_week = true;
    }

    /* As soon as we have both first TOW and GPS week, compute absolute start time. */
    if (st->have_first_tow && st->have_gps_week) {
      st->start_unix_time = unix_time_of_tow_week(st->first_tow_s, st->gps_week) - st->first_tow_log_t;
      st->have_start_unix_time = true;
    }
  }

  if (is_alive && !st->have_md5 && md5_hex[0]) {
    snprintf(st->md5, sizeof(st->md5), "%s", md5_hex);
    st->have_md5 = true;
  }
}

/* PPRZLOG frame: STX, len, source, timestamp (4), pprz payload (len), checksum */
typedef struct {
  enum { S_WAIT_STX, S_LEN, S_SOURCE, S_TS, S_PAYLOAD, S_CK } state;
  uint8_t len;
  uint8_t source;
  uint32_t ts;
  uint8_t cks;
  uint8_t got;
  uint8_t payload[MAX_PAYLOAD];
} frame_parser_t;

/* Feed one byte; returns 1 for a complete frame, -1 for a checksum error, else 0 */
static int frame_parser_feed(frame_parser_t *fp, uint8_t b)
{
  switch (fp->state) {
    case S_WAIT_STX:
      if (b == STX_LOG) fp->state = S_LEN;
      return 0;
    case S_LEN:
      fp->len = b;
      fp->cks = b;
      fp->got = 0;
      fp->ts = 0;
      fp->state = S_SOURCE;
      return 0;
    case S_SOURCE:
      fp->source = b;
      fp->cks = (uint8_t)(fp->cks + b);
      fp->state = S_TS;
      return 0;
    case S_TS:
      fp->ts |= (uint32_t)b << (8 * fp->got);
      fp->cks = (uint8_t)(fp->cks + b);
      if (++fp->got == 4) {
        fp->got = 0;
        fp->state = (fp->len == 0) ? S_CK : S_PAYLOAD;
      }
      return 0;
    case S_PAYLOAD:
      fp->payload[fp->got++] = b;
      fp->cks = (uint8_t)(fp->cks + b);
      if (fp->got >= fp->len) fp->state = S_CK;
      return 0;
    case S_CK:
    default:
      fp->state = S_WAIT_STX;
      return (b == fp->cks) ? 1 : -1;
  }
}

/* Find the configuration md5 in the first ALIVE message of the log */
static bool find_md5_in_log(const char *input, const msg_db_t *db, char md5[33])
{
  FILE *in = fopen(input, "rb");
  uint8_t *buf = malloc(IO_BUF_SZ);
  frame_parser_t fp = { .state = S_WAIT_STX };
  bool found = false;

  while (in && buf && !found) {
    size_t n = fread(buf, 1, IO_BUF_SZ, in);
    if (n == 0) break;
    for (size_t i = 0; i < n && !found; i++) {
      if (frame_parser_feed(&fp, buf[i]) != 1) continue;
      /* pprz payload: sender, receiver, class/component, msg id, md5sum length, md5sum */
      if (fp.source == 0 && fp.len >= 4 + 1 + 16 && fp.payload[3] == db->alive_id && fp.payload[4] >= 16) {
        for (int j = 0; j < 16; j++) {
          snprintf(&md5[j * 2], 3, "%02x", fp.payload[5 + j]);
        }
        found = true;
      }
    }
  }

  free(buf);
  if (in) fclose(in);
  return found;
}

static bool convert_file(const char *input, const char *out_dir, bool write_tlm)
{
  uint64_t start_ms = monotonic_ms();
  convert_options_t options = {
    .write_tlm = write_tlm,
    .debug_detect = env_flag_enabled("SD2LOG_DEBUG")
  };
  char var_dir[PATH_MAX];
  char logs_default[PATH_MAX];

  const char *home = getenv("PAPARAZZI_HOME");
  if (!home || !home[0]) home = ".";
  snprintf(var_dir, sizeof(var_dir), "%s/var", home);
  path_join(logs_default, sizeof(logs_default), var_dir, "logs");

  const char *logs = out_dir ? out_dir : logs_default;
  if (!dir_exists(logs)) {
    fprintf(stderr, "Output directory not found: %s\n", logs);
    return false;
  }

  char msg_xml[PATH_MAX];
  path_join(msg_xml, sizeof(msg_xml), var_dir, "messages.xml");

  msg_db_t *db = calloc(1, sizeof(*db));
  if (!db) {
    fprintf(stderr, "Failed to allocate message database\n");
    return false;
  }

  if (!load_messages_xml(msg_xml, db)) {
    fprintf(stderr, "Failed to load messages definition: %s\n", msg_xml);
    free(db);
    return false;
  }

  /* Decode with the message definitions the firmware was built with, saved in
   * var/conf with the configuration md5, so logs of older or newer firmware decode
   * correctly. Without that file, var/messages.xml is used. */
  char build_md5[33] = {0};
  char build_conf[PATH_MAX] = {0};
  char build_conf_dir[PATH_MAX];
  path_join(build_conf_dir, sizeof(build_conf_dir), var_dir, "conf");
  if (find_md5_in_log(input, db, build_md5)
      && find_conf_by_md5(build_conf_dir, build_md5, build_conf, sizeof(build_conf))) {
    msg_db_t *build_db = calloc(1, sizeof(*build_db));
    if (build_db && load_messages_xml(build_conf, build_db)) {
      free(db);
      db = build_db;
      fprintf(stderr, "[msgs] using the message definitions of the build (%s)\n", build_md5);
    } else {
      free(build_db);
      fprintf(stderr, "[msgs] no message definitions in %s, using %s\n", build_conf, msg_xml);
    }
  } else {
    fprintf(stderr, "[msgs] build configuration not found, using %s\n", msg_xml);
  }

  FILE *in = fopen(input, "rb");
  if (!in) {
    fprintf(stderr, "Cannot open input '%s': %s\n", input, strerror(errno));
    free(db);
    return false;
  }

  char tmp_data[PATH_MAX];
  char tmp_name[64];
  snprintf(tmp_name, sizeof(tmp_name), ".sd2log_tmp_%d.data", (int)getpid());
  path_join(tmp_data, sizeof(tmp_data), logs, tmp_name);
  FILE *out = fopen(tmp_data, "wb");
  if (!out) {
    fprintf(stderr, "Cannot create temp data file '%s': %s\n", tmp_data, strerror(errno));
    fclose(in);
    free(db);
    return false;
  }

  setvbuf(in, NULL, _IOFBF, IO_BUF_SZ);
  setvbuf(out, NULL, _IOFBF, IO_BUF_SZ);

  ui_state_t ui;
  ui_state_init(&ui, in);

  conv_state_t st;
  memset(&st, 0, sizeof(st));
  st.single_ac_id = -1;

  frame_parser_t fp = { .state = S_WAIT_STX };

  uint8_t *buf = malloc(IO_BUF_SZ);
  if (!buf) {
    fclose(in);
    fclose(out);
    unlink(tmp_data);
    free(db);
    return false;
  }

  while (!feof(in)) {
    size_t n = fread(buf, 1, IO_BUF_SZ, in);
    if (ferror(in)) {
      if (ui.show_progress) fputc('\n', stderr);
      fprintf(stderr, "Read error on '%s': %s\n", input, strerror(errno));
      return abort_during_parse(buf, in, out, tmp_data, db);
    }
    for (size_t i = 0; i < n; i++) {
      int r = frame_parser_feed(&fp, buf[i]);
      if (r > 0) {
        handle_message(out, db, &st, fp.source, fp.ts, fp.payload, fp.len);
      } else if (r < 0) {
        st.checksum_errors++;
      }
    }

    ui_state_update_progress(&ui, (uint64_t)n, st.parsed);
    maybe_announce_detect_messages(&options, &ui, &st);
    if (!maybe_announce_detected_time(&options, &ui, &st, logs, buf, in, out, tmp_data, db)) {
      return false;
    }
  }

  ui_state_finish_progress(&ui, st.parsed);

  free(buf);
  fclose(in);
  fclose(out);

  if (!maybe_set_fallback_start_time(&options, &ui, &st, logs, tmp_data, db)) {
    return false;
  }

  double start_time;
  const char *suffix;
  if (st.have_start_unix_time) {
    start_time = st.start_unix_time;
    suffix = "";
  } else {
    start_time = (double)time(NULL);
    suffix = "_no_GPS";
    fprintf(stderr, "Warning: no GPS date found, using current local time\n");
  }

  output_files_t outputs;
  build_output_files(logs, start_time, suffix, &outputs);

  if (rename(tmp_data, outputs.data_path) != 0) {
    fprintf(stderr, "Failed to move data file to '%s': %s\n", outputs.data_path, strerror(errno));
    unlink(tmp_data);
    free(db);
    return false;
  }

  char conf_dir[PATH_MAX];
  char conf_path[PATH_MAX] = {0};
  char airframe_path[PATH_MAX] = {0};
  char conf_xml_path[PATH_MAX];
  bool conf_from_md5 = false;
  const char *conf_choice = "unknown";
  path_join(conf_xml_path, sizeof(conf_xml_path), home, "conf/conf.xml");
  path_join(conf_dir, sizeof(conf_dir), var_dir, "conf");
  if (st.have_md5 && dir_exists(conf_dir)) {
    if (find_conf_by_md5(conf_dir, st.md5, conf_path, sizeof(conf_path))) {
      conf_from_md5 = true;
    }
  }

  if (!conf_path[0] && st.single_ac_id >= 0) {
    char ac_name[MAX_NAME] = {0};
    if (file_exists(conf_xml_path)
        && lookup_aircraft_attr(conf_xml_path, st.single_ac_id, "name", ac_name, sizeof(ac_name))) {
      generate_conf_aircraft_with_ocaml(home, conf_xml_path, ac_name, conf_path, sizeof(conf_path));
    }
  }

  if (!conf_path[0] && st.single_ac_id >= 0) {
    char replay_conf_xml[PATH_MAX];
    path_join(replay_conf_xml, sizeof(replay_conf_xml), var_dir, "replay/conf/conf.xml");
    char airframe_rel[PATH_MAX] = {0};
    if (file_exists(replay_conf_xml) && lookup_aircraft_attr(replay_conf_xml, st.single_ac_id, "airframe", airframe_rel, sizeof(airframe_rel))) {
      char replay_conf_dir[PATH_MAX];
      path_join(replay_conf_dir, sizeof(replay_conf_dir), var_dir, "replay/conf");
      path_join(airframe_path, sizeof(airframe_path), replay_conf_dir, airframe_rel);
    }
  }

  if (conf_path[0]) {
    conf_choice = conf_from_md5 ? "md5-conf" : "rebuild-conf";
  } else if (airframe_path[0]) {
    conf_choice = "replay-airframe";
  } else if (file_exists(conf_xml_path)) {
    conf_choice = "conf.xml-fallback";
  } else {
    conf_choice = "minimal-fallback";
  }

  fprintf(stderr, "[conf] md5=%s ac_id=%d choice=%s\n", st.have_md5 ? st.md5 : "none", st.single_ac_id, conf_choice);

  if (write_log_from_conf(outputs.log_path,
                          conf_path[0] ? conf_path : NULL,
                          airframe_path[0] ? airframe_path : NULL,
                          msg_xml,
                          start_time,
                          outputs.data_name)) {
  } else {
    fprintf(stderr, "No .log produced\n");
  }

  if (options.write_tlm) {
    if (!copy_file_fast(input, outputs.tlm_path)) {
      fprintf(stderr, "Failed to copy .tlm to '%s'\n", outputs.tlm_path);
      free(db);
      return false;
    }
  }
  double processing_s = (double)(monotonic_ms() - start_ms) / 1000.0;
  fprintf(stderr,
          "Generated %s.[data,log%s] | time=%.2f | conf=%s | frames=%" PRIu64 " skipped=%" PRIu64 " cksum=%" PRIu64 " | proc=%.2fs\n",
          outputs.base, options.write_tlm ? ",tlm" : "", start_time,
          conf_choice, st.parsed, st.skipped, st.checksum_errors, processing_s);

  free(db);
  return true;
}

int main(int argc, char **argv)
{
  if (argc < 2 || argc > 4) {
    fprintf(stderr,
            "Usage: %s <telemetry airborne file> [<output directory>] [--tlm]\n",
            argv[0]);
    return 1;
  }

  const char *out_dir = NULL;
  bool write_tlm = false;
  for (int i = 2; i < argc; i++) {
    if (strcmp(argv[i], "--tlm") == 0) {
      write_tlm = true;
    } else if (!out_dir) {
      out_dir = argv[i];
    } else {
      fprintf(stderr, "Unknown argument: %s\n", argv[i]);
      fprintf(stderr,
              "Usage: %s <telemetry airborne file> [<output directory>] [--tlm]\n",
              argv[0]);
      return 1;
    }
  }

  if (!convert_file(argv[1], out_dir, write_tlm)) {
    return 1;
  }
  return 0;
}
