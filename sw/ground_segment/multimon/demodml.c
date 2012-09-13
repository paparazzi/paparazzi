#include <caml/mlvalues.h>
#include <caml/fail.h>
#include <caml/alloc.h>
#include <caml/memory.h>

#include "pprzlib.h"

value
ml_demod_init(value dev) {
#ifndef __APPLE__
  int fd = pprz_demod_init(String_val(dev));
  return Val_int(fd);
#else
  failwith("Not supported under OSX");
  return Val_int(0);
#endif

}

value
ml_demod_get_data(value unit) {
#ifndef __APPLE__
  struct data *data = pprz_demod_read_data();
  if (data) {
    int i;

    CAMLparam0();
    CAMLlocal3 (result,l, r);
    result = alloc(2, 0);
    l = alloc_string(data->len_left);
    for(i = 0; i < data->len_left; i++) Byte(l, i) = data->data_left[i];
    r = alloc_string(data->len_right);
    for(i = 0; i < data->len_right; i++) Byte(r, i) = data->data_right[i];

    Store_field(result, 0, l);
    Store_field(result, 1, r);
    CAMLreturn (result);
  } else {
    failwith("End of file");
  }
#else
  CAMLparam0();
  CAMLlocal3 (result,l, r);
  result = alloc(2, 0);
  failwith("Not supported under OSX");
  CAMLreturn (result);
#endif
}
