#include "booz_sensors_model_utils.h"

#include "6dof.h"
#include <math.h>

void UpdateSensorLatency(double time, VEC* cur_reading, GSList* history, 
			 double latency, VEC* sensor_reading) {
  /* add new reading */
  struct BoozDatedSensor* cur_read = g_new(struct BoozDatedSensor, 1);
  cur_read->time = time;
  cur_read->value = v_get(AXIS_NB);
  v_copy(cur_reading, cur_read->value);
  history = g_slist_prepend(history, cur_read);
  /* remove old readings */
  GSList* last =  g_slist_last(history);
  while (last && 
	 ((struct BoozDatedSensor*)last->data)->time < time - latency) {
    history = g_slist_remove_link(history, last);
    v_free(((struct BoozDatedSensor*)last->data)->value);
    g_free((struct BoozDatedSensor*)last->data);
    g_slist_free(last);
    last =  g_slist_last(history);
  }
  /* update sensor        */
  v_copy(((struct BoozDatedSensor*)last->data)->value, sensor_reading);
}


VEC* v_update_random_walk(VEC* in, VEC* std_dev, double dt, VEC* out) {
  static VEC *tmp = VNULL;
  tmp = v_resize(tmp, AXIS_NB);
  tmp = sv_mlt(dt, std_dev, tmp);
  out =  v_add_gaussian_noise(in, tmp, out);
  return out;
}

VEC* v_add_gaussian_noise(VEC* in, VEC* std_dev, VEC* out) {
  static VEC *tmp = VNULL;
  tmp = v_resize(tmp, in->dim);
  unsigned int i;
  for (i=0; i<in->dim; i++)
    tmp->ve[i] = get_gaussian_noise() * std_dev->ve[i];
  v_add(in, tmp, out);
  return out;
}


/* 
   http://www.taygeta.com/random/gaussian.html 
*/

#include "booz_r250.h"

double get_gaussian_noise(void) {
 
  double x1;
  static int nb_call = 0;
  static double x2, w;
  if (nb_call==0) r250_init(0);
  nb_call++;
  if (nb_call%2) {
    do {
      x1 = 2.0 * dr250() - 1.0;
      x2 = 2.0 * dr250() - 1.0;
      w = x1 * x1 + x2 * x2;
    } while ( w >= 1.0 );
    
    w = sqrt( (-2.0 * log( w ) ) / w );
    return x1 * w;
  }
  else
    return x2 * w;
}





