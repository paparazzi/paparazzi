#include "booz_sensors_model_utils.h"

#include "6dof.h"

void UpdateSensorLatency(double time, VEC* cur_reading, GSList* history, 
			 double latency, VEC* sensor_reading) {
  /* add new reading */
  struct BoozDatedSensor* cur_read = g_new(struct BoozDatedSensor, 1);
  cur_read->time = time;
  cur_read->value = v_get(AXIS_NB);
  CopyVect(cur_read->value, cur_reading);
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
  CopyVect(sensor_reading, ((struct BoozDatedSensor*)last->data)->value);
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
  tmp = v_resize(tmp, AXIS_NB);
  tmp = v_rand(tmp);
  static VEC *one = VNULL;
  one = v_resize(one, AXIS_NB);
  one = v_ones(one);
  tmp = v_mltadd(one, tmp, -2., tmp); 
  tmp = v_star(tmp, std_dev, tmp);
  out = v_add(in, tmp, out);
  return out;
}
