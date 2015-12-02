/*
 * Copyright (C) 2015 Gautier Hattenberger, Alexandre Bustico
 *
 * This file is part of paparazzi

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
 *
 */

/**
 * @file modules/meteo/meteo_stick_calib.c
 *
 * Parser for calibration data coming from 25AA256 EEPROM
 * of the Meteo Stick
 *
 */

#include "modules/meteo/meteo_stick_calib.h"
#include <math.h>

typedef struct {
  float ref;
  float value;
  float rawValue;
} CalibrationPoint;

// local functions declaration
static bool mtostk_populate_sensor_from_buffer(Sensors_params *sp, uint32_t **eeprom_buffer);
static bool mtostk_populate_float_array_from_buffer(float *ar,
    uint16_t *num_of_array, uint16_t *num_of_elem,
    uint32_t **eeprom_buffer);
static bool mtostk_populate_uuid_from_buffer(char *str, uint32_t **eeprom_buffer);
static bool mtostk_populate_uint_from_buffer(uint32_t *sca, uint32_t **eeprom_buffer);
static bool mtostk_seek_array_buffer(size_t dataSize, uint32_t **eeprom_buffer);
static bool mtostk_seek_scalar_buffer(size_t dataSize, uint32_t **eeprom_buffer);
static uint32_t fletcher32(uint32_t const *_data, size_t words);
static float mtostk_apply_polynomial(float *coeffs, uint16_t num_coef, float value);
static float mtostk_apply_polynomial_temp(Sensors_params *params, float temp, float value);

// Populate calib struct
bool mtostk_populate_cal_from_buffer(Calibration_params *cp,  uint8_t *_eeprom_buffer)
{
  uint32_t *eeprom_buffer_ptr = (uint32_t *) _eeprom_buffer;

  if (mtostk_populate_uuid_from_buffer(cp->uuid, &eeprom_buffer_ptr) == false) {
    return false;
  }

  for (Mtostk_sensors ms = MTOSTK_TEMP;  ms < MTOSTK_NUM_SENSORS; ms++) {
    if (mtostk_populate_sensor_from_buffer(&cp->params[ms], &eeprom_buffer_ptr) == false) {
      return false;
    }
  }

  return true;
}

// Get calibrated data
float mtostk_get_calibrated_value(Calibration_params *cp, Mtostk_sensors type, float uncal, float temp)
{
  if (type >= MTOSTK_NUM_SENSORS) { return 0; } // Invalid number

  Sensors_params *params = &(cp->params[type]);
  if (type == MTOSTK_TEMP) {
    // sanity check
    if (params->num_temp == 1 && params->timestamp != 0) {
      return mtostk_apply_polynomial(params->coeffs[0], params->num_coeff, uncal);
    } else {
      return uncal;
    }
  } else {
    return mtostk_apply_polynomial_temp(params, temp, uncal);
  }

}

static bool mtostk_populate_sensor_from_buffer(Sensors_params *sp,  uint32_t **eeprom_buffer)
{
  uint32_t storeChksum = 0, recordSize;
  const uint32_t *initialBufPtr = *eeprom_buffer;

  mtostk_populate_uint_from_buffer(&recordSize, eeprom_buffer);
  mtostk_populate_uint_from_buffer((uint32_t *) & (sp->timestamp), eeprom_buffer);

  mtostk_seek_scalar_buffer(sizeof(float), eeprom_buffer);  // we don't need maxStdDev
  mtostk_populate_float_array_from_buffer((float *) sp->temps,  NULL, NULL, eeprom_buffer);
  mtostk_populate_float_array_from_buffer((float *) sp->coeffs,  &sp->num_temp, &sp->num_coeff, eeprom_buffer);
  mtostk_seek_array_buffer(sizeof(CalibrationPoint),  eeprom_buffer);   // we don't need calibrationPoints

  const size_t chksumBufLen = *eeprom_buffer - initialBufPtr;
  const uint32_t calcChksum = fletcher32(initialBufPtr, chksumBufLen);
  mtostk_populate_uint_from_buffer(&storeChksum, eeprom_buffer);

  if (calcChksum != storeChksum) {
    return false;
  }

  return true;
}

static bool mtostk_populate_uint_from_buffer(uint32_t *sca,  uint32_t **eeprom_buffer)
{
  if (*eeprom_buffer == NULL) {
    return false;
  }

  *sca =  *(*eeprom_buffer)++;
  return true;
}

static bool mtostk_populate_float_array_from_buffer(float *ar,
    uint16_t *num_of_array, uint16_t *num_of_elem,
    uint32_t **eeprom_buffer)
{
  if (*eeprom_buffer == NULL) {
    return false;
  }
  uint16_t noa = 0, noe = 0;

  if (num_of_array == NULL) {
    num_of_array = &noa;
  }

  if (num_of_elem == NULL) {
    num_of_elem = &noe;
  } else {
    *num_of_elem = 0;
  }

  float **eeprom_buffer_float = (float **) eeprom_buffer;

  _Static_assert(sizeof(uint32_t) == sizeof(time_t),
                 "sizeof (uint32_t) differ from sizeof (time_t)");
  _Static_assert(sizeof(uint32_t) == sizeof(float),
                 "sizeof (uint32_t) differ from sizeof (float)");

  *num_of_array = *(*eeprom_buffer)++;
  if (*num_of_array > MTOSTK_MAX_TEMP_ARRAY_SIZE) {
    return false;
  }

  for (size_t idx = 0; idx < *num_of_array; idx++) {
    const size_t this_num_of_elem = *(*eeprom_buffer)++;
    if (*num_of_elem == 0) {
      *num_of_elem =  this_num_of_elem;
    } else {
      if (*num_of_elem != this_num_of_elem) {
        return false;
      }
    }

    if (*num_of_elem > MTOSTK_MAX_POLY_ARRAY_SIZE) {
      return false;
    }

    for (int32_t jdx = 0; jdx < *num_of_elem; jdx++) {
      ar[(idx * MTOSTK_MAX_POLY_ARRAY_SIZE) + jdx] = *(*eeprom_buffer_float)++;
    }
  }

  return true;
}

static bool mtostk_populate_uuid_from_buffer(char *str,  uint32_t **eeprom_buffer)
{
  uint8_t **eeprom_buffer_byte = (uint8_t **) eeprom_buffer;
  if (*eeprom_buffer == NULL) {
    return false;
  }

  const size_t numOfArrays = *(*eeprom_buffer)++;
  if (numOfArrays > 1) {
    return false;
  }

  const size_t numOfElems = *(*eeprom_buffer)++;
  if (numOfElems > UUID_LEN) {
    return false;
  }

  for (uint32_t idx = 0; idx < numOfArrays * numOfElems; idx++) {
    *str++ = *(*eeprom_buffer_byte)++;
  }

  return true;
}

static bool mtostk_seek_array_buffer(size_t dataSize,  uint32_t **eeprom_buffer)
{
  uint8_t **eeprom_buffer_byte = (uint8_t **) eeprom_buffer;

  if (*eeprom_buffer == NULL) {
    return false;
  }

  const size_t numOfArrays = *(*eeprom_buffer)++;
  if (numOfArrays > MTOSTK_MAX_TEMP_ARRAY_SIZE) {
    return false;
  }

  for (size_t i = 0; i < numOfArrays; i++) {
    const size_t numOfElems =  *(*eeprom_buffer)++;
    if (numOfElems > MTOSTK_MAX_SEEK_ARRAY_SIZE) {
      return false;
    }
    *eeprom_buffer_byte += numOfElems * dataSize;
    // *eeprom_buffer = (uint32_t *) *eeprom_buffer_byte;
  }

  return true;
}

static bool mtostk_seek_scalar_buffer(size_t dataSize,  uint32_t **eeprom_buffer)
{
  uint8_t **eeprom_buffer_byte = (uint8_t **) eeprom_buffer;
  *eeprom_buffer_byte += dataSize;
  return true;
}


static uint32_t fletcher32(uint32_t const *_data, size_t words)
{
  words += words;
  uint16_t const *data = (uint16_t *) _data;

  uint32_t sum1 = 0xffff, sum2 = 0xffff;

  while (words) {
    unsigned tlen = words > 359 ? 359 : words;
    words -= tlen;
    do {
      sum2 += sum1 += *data++;
    } while (--tlen);
    sum1 = (sum1 & 0xffff) + (sum1 >> 16);
    sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  }
  /* Second reduction step to reduce sums to 16 bits */
  sum1 = (sum1 & 0xffff) + (sum1 >> 16);
  sum2 = (sum2 & 0xffff) + (sum2 >> 16);
  return sum2 << 16 | sum1;
}

float mtostk_apply_polynomial(float *coeffs, uint16_t num_coef, float value)
{
  int i;
  float y = coeffs[0];
  for (i = 1; i < num_coef; i++) {
    y += coeffs[i] * powf(value, (float) i);
  }
  return y;
}

float mtostk_apply_polynomial_temp(Sensors_params *params, float temp, float value)
{
  /* if not temperature compensated, just apply polynomial if any
   * if temperature compensated
   *   get current corrected temp
   *   if out of bound, use closest bound
   *   else
   *     find correct interval
   *     compute the two values based on bounding polynomials
   *     make a linear interpolation of these two value
   */
  if (params->num_temp == 0 || params->timestamp == 0) {
    return value; // No calibration available
  } else if (params->num_temp == 1) {
    return mtostk_apply_polynomial(params->coeffs[0], params->num_coeff, value); // No interpolation
  } else {
    if (temp <= params->temps[0]) {
      return mtostk_apply_polynomial(params->coeffs[0], params->num_coeff, value);
    } else if (temp >= params->temps[params->num_temp - 1]) {
      return mtostk_apply_polynomial(params->coeffs[params->num_temp - 1], params->num_coeff, value);
    } else {
      int i;
      for (i = 0; i <= params->num_temp - 2; i++) {
        const float t1 = params->temps[i];
        const float t2 = params->temps[i + 1];
        if (temp > t1 && temp <= t2) {
          const float v1 = mtostk_apply_polynomial(params->coeffs[i], params->num_coeff, value);
          const float v2 = mtostk_apply_polynomial(params->coeffs[i + 1], params->num_coeff, value);
          // Linear interpolation
          const float alpha = (t2 - temp) / (t2 - t1);
          return (alpha * v1 + (1.0f - alpha) * v2);
        }
      }
      return value; // This should never append
    }
  }

}

