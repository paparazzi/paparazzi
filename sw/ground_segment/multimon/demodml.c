#include <caml/mlvalues.h>
#include <caml/fail.h>
#include <caml/alloc.h>
#include <caml/memory.h>

#include "pprzlib.h"

value
ml_demod_init(value dev) {
  int fd = pprz_demod_init(String_val(dev));
  return Val_int(fd);
}

value
ml_demod_get_data(value unit) {
  char **data = pprz_demod_read_data();

  CAMLparam0();
  CAMLlocal1 (result);
  result = alloc(2, 0);
  Store_field(result, 0, copy_string(data[0]));
  Store_field(result, 1, copy_string(data[1]));
  CAMLreturn (result);
}
