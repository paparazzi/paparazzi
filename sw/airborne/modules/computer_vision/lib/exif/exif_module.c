/*
 * Copyright (C) 2015
 *
 * This file is part of paparazzi.
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



#include "exif_module.h"
#include "state.h"
#include "subsystems/gps.h"


//////////////////////////////////////////////////////////////
// Multithreaded part

volatile int32_t lat_em7deg = 0;
volatile int32_t lon_em7deg = 0;
volatile int32_t alt_mm = 0;

//////////////////////////////////////////////////////////////
// Paparazzi part

void push_gps_to_vision(void)
{
  struct LlaCoor_i *c = stateGetPositionLla_i();
  lat_em7deg = c->lat;
  lon_em7deg = c->lon;
  // alt_mm = c->alt;
  alt_mm = gps.hmsl;
}

/////////////////////////////////////////////////////////////
// Vision part

#include <libexif/exif-data.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Get an existing tag, or create one if it doesn't exist */
static ExifEntry *init_tag(ExifData *exif, ExifIfd ifd, ExifTag tag)
{
  ExifEntry *entry;
  /* Return an existing tag if one exists */
  if (!((entry = exif_content_get_entry(exif->ifd[ifd], tag)))) {
    /* Allocate a new entry */
    entry = exif_entry_new();
    //assert(entry != NULL); /* catch an out of memory condition */
    entry->tag = tag; /* tag must be set before calling
         exif_content_add_entry */

    /* Attach the ExifEntry to an IFD */
    exif_content_add_entry(exif->ifd[ifd], entry);

    /* Allocate memory for the entry and fill with default data */
    exif_entry_initialize(entry, tag);

    /* Ownership of the ExifEntry has now been passed to the IFD.
     * One must be very careful in accessing a structure after
     * unref'ing it; in this case, we know "entry" won't be freed
     * because the reference count was bumped when it was added to
     * the IFD.
     */
    exif_entry_unref(entry);
  }
  return entry;
}

/* Create a brand-new tag with a data field of the given length, in the
 * given IFD. This is needed when exif_entry_initialize() isn't able to create
 * this type of tag itself, or the default data length it creates isn't the
 * correct length.
 */
static ExifEntry *create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag, size_t len)
{
  void *buf;
  ExifEntry *entry;

  /* Create a memory allocator to manage this ExifEntry */
  ExifMem *mem = exif_mem_new_default();
  //assert(mem != NULL); /* catch an out of memory condition */

  /* Create a new ExifEntry using our allocator */
  entry = exif_entry_new_mem(mem);
  //assert(entry != NULL);

  /* Allocate memory to use for holding the tag data */
  buf = exif_mem_alloc(mem, len);
  //assert(buf != NULL);

  /* Fill in the entry */
  entry->data = buf;
  entry->size = len;
  entry->tag = tag;
  entry->components = len;
  entry->format = EXIF_FORMAT_UNDEFINED;

  /* Attach the ExifEntry to an IFD */
  exif_content_add_entry(exif->ifd[ifd], entry);

  /* The ExifMem and ExifEntry are now owned elsewhere */
  exif_mem_unref(mem);
  exif_entry_unref(entry);

  return entry;
}

/* start of JPEG image data section */
static const unsigned int image_data_offset = 2;
#define image_data_len (image_jpg_len - image_data_offset)

/* raw EXIF header data */
static const unsigned char exif_header[] = {
  0xff, 0xd8, 0xff, 0xe1
};
/* length of data in exif_header */
static const unsigned int exif_header_len = sizeof(exif_header);

/* byte order to use in the EXIF block */
#define FILE_BYTE_ORDER EXIF_BYTE_ORDER_INTEL

/* comment to write into the EXIF block */
#define FILE_COMMENT "Paparazzi autopilot"

/* special header required for EXIF_TAG_USER_COMMENT */
#define ASCII_COMMENT "ASCII\0\0\0"




int write_exif_jpeg(char *filename, const unsigned char *image_jpg, const unsigned int image_jpg_len,
                    const unsigned int image_jpg_x, const unsigned int image_jpg_y)
{
  int rc = 1;
  FILE *f;
  unsigned char *exif_data;
  unsigned int exif_data_len;
  ExifEntry *entry;
  ExifData *exif = exif_data_new();
  if (!exif) {
    fprintf(stderr, "Out of memory\n");
    return 2;
  }

  /* Set the image options */
  exif_data_set_option(exif, EXIF_DATA_OPTION_FOLLOW_SPECIFICATION);
  exif_data_set_data_type(exif, EXIF_DATA_TYPE_COMPRESSED);
  exif_data_set_byte_order(exif, FILE_BYTE_ORDER);

  /* Create the mandatory EXIF fields with default data */
  exif_data_fix(exif);

  /* All these tags are created with default values by exif_data_fix() */
  /* Change the data to the correct values for this image. */
  entry = init_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_PIXEL_X_DIMENSION);
  exif_set_long(entry->data, FILE_BYTE_ORDER, image_jpg_x);

  entry = init_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_PIXEL_Y_DIMENSION);
  exif_set_long(entry->data, FILE_BYTE_ORDER, image_jpg_y);

  entry = init_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_COLOR_SPACE);
  exif_set_short(entry->data, FILE_BYTE_ORDER, 1);

  /* Create a EXIF_TAG_USER_COMMENT tag. This one must be handled
   * differently because that tag isn't automatically created and
   * allocated by exif_data_fix(), nor can it be created using
   * exif_entry_initialize() so it must be explicitly allocated here.
   */
  entry = create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_USER_COMMENT,
                     sizeof(ASCII_COMMENT) + sizeof(FILE_COMMENT) - 2);
  /* Write the special header needed for a comment tag */
  memcpy(entry->data, ASCII_COMMENT, sizeof(ASCII_COMMENT) - 1);
  /* Write the actual comment text, without the trailing NUL character */
  memcpy(entry->data + 8, FILE_COMMENT, sizeof(FILE_COMMENT) - 1);
  /* create_tag() happens to set the format and components correctly for
   * EXIF_TAG_USER_COMMENT, so there is nothing more to do. */

  /* Create a EXIF_TAG_SUBJECT_AREA tag */
  entry = create_tag(exif, EXIF_IFD_EXIF, EXIF_TAG_SUBJECT_AREA,
                     4 * exif_format_get_size(EXIF_FORMAT_SHORT));
  entry->format = EXIF_FORMAT_SHORT;
  entry->components = 4;
  exif_set_short(entry->data, FILE_BYTE_ORDER, image_jpg_x / 2);
  exif_set_short(entry->data + 2, FILE_BYTE_ORDER, image_jpg_y / 2);
  exif_set_short(entry->data + 4, FILE_BYTE_ORDER, image_jpg_x);
  exif_set_short(entry->data + 6, FILE_BYTE_ORDER, image_jpg_y);

  entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_LATITUDE_REF, 2);
  entry->format = EXIF_FORMAT_ASCII;
  entry->components = 1;
  entry->data[1] = 0;
  if (lat_em7deg < 0) {
    entry->data[0] = 'S';
    // from now on: go positive only
    lat_em7deg = -lat_em7deg;
  }
  else {
    entry->data[0] = 'N';
  }

  entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_LONGITUDE_REF, 2);
  entry->format = EXIF_FORMAT_ASCII;
  entry->components = 1;
  entry->data[1] = 0;
  if (lon_em7deg < 0) {
    entry->data[0] = 'W';
    // from now on: go positive only
    lon_em7deg = -lon_em7deg;
  }
  else {
    entry->data[0] = 'E';
  }



  entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_LATITUDE, 24);
  // Set the field's format and number of components, this is very important!
  entry->format = EXIF_FORMAT_RATIONAL;
  entry->components = 3;
  // Degrees
  uint32_t lat = lat_em7deg;
  uint32_t lati = lat / 1e7;
  ExifRational loc;
  loc.numerator = lati;
  loc.denominator = 1;
  exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, loc);
  lat -= lati * 1e7;
  lat *= 60.0;
  lati = lat / 1e7;
  loc.numerator = lati;
  exif_set_rational(entry->data + 8, EXIF_BYTE_ORDER_INTEL, loc);
  lat -= lati * 1e7;
  lat *= 60.0;
  lati = lat / 1e4;
  loc.numerator = lati;
  loc.denominator = 1000;
  exif_set_rational(entry->data + 16, EXIF_BYTE_ORDER_INTEL, loc);

  entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_LONGITUDE, 24);
  // Set the field's format and number of components, this is very important!
  entry->format = EXIF_FORMAT_RATIONAL;
  entry->components = 3;
  // Degrees
  // Degrees
  uint32_t lon = lon_em7deg;
  uint32_t loni = lon / 1e7;
  loc.numerator = loni;
  loc.denominator = 1;
  exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, loc);
  lon -= loni * 1e7;
  lon *= 60.0;
  loni = lon / 1e7;
  loc.numerator = loni;
  loc.denominator = 1;
  exif_set_rational(entry->data + 8, EXIF_BYTE_ORDER_INTEL, loc);
  lon -= loni * 1e7;
  lon *= 60.0;
  loni = lon / 1e4;
  loc.numerator = loni;
  loc.denominator = 1000;
  exif_set_rational(entry->data + 16, EXIF_BYTE_ORDER_INTEL, loc);

  entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_ALTITUDE_REF, 1);
  entry->format = EXIF_FORMAT_BYTE;
  entry->components = 1;
  if (alt_mm < 0) {
    entry->data[0] = 1; // Below MSL
    // from now on: go positive only
    alt_mm = -alt_mm;
  }
  else {
    entry->data[0] = 0; // Above MSL
  }

  entry = create_tag(exif, EXIF_IFD_GPS, EXIF_TAG_GPS_ALTITUDE, 8);
  // Set the field's format and number of components, this is very important!
  entry->format = EXIF_FORMAT_RATIONAL;
  entry->components = 1;
  // Height
  ExifRational alt;
  alt.numerator = alt_mm;
  alt.denominator = 1000;
  exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, alt);


  /* Get a pointer to the EXIF data block we just created */
  exif_data_save_data(exif, &exif_data, &exif_data_len);
//  assert(exif_data != NULL);

  f = fopen(filename, "wb");
  if (!f) {
    fprintf(stderr, "Error creating file %s\n", filename);
    exif_data_unref(exif);
    return rc;
  }
  /* Write EXIF header */
  if (fwrite(exif_header, exif_header_len, 1, f) != 1) {
    fprintf(stderr, "Error writing to file %s\n", filename);
    goto errout;
  }
  /* Write EXIF block length in big-endian order */
  if (fputc((exif_data_len + 2) >> 8, f) < 0) {
    fprintf(stderr, "Error writing to file %s\n", filename);
    goto errout;
  }
  if (fputc((exif_data_len + 2) & 0xff, f) < 0) {
    fprintf(stderr, "Error writing to file %s\n", filename);
    goto errout;
  }
  /* Write EXIF data block */
  if (fwrite(exif_data, exif_data_len, 1, f) != 1) {
    fprintf(stderr, "Error writing to file %s\n", filename);
    goto errout;
  }
  /* Write JPEG image data, skipping the non-EXIF header */
  if (fwrite(image_jpg + image_data_offset, image_data_len, 1, f) != 1) {
    fprintf(stderr, "Error writing to file %s\n", filename);
    goto errout;
  }
  printf("Wrote file %s\n", filename);
  rc = 0;

errout:
  if (fclose(f)) {
    fprintf(stderr, "Error writing to file %s\n", filename);
    rc = 1;
  }
  /* The allocator we're using for ExifData is the standard one, so use
   * it directly to free this pointer.
   */
  free(exif_data);
  exif_data_unref(exif);

  return rc;
}


