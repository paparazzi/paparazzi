/** Values boxing for Flight Gear */

#include <string.h>
#include <caml/alloc.h>
#include <caml/mlvalues.h>
#include <caml/memory.h>

struct fg {
  float x;
  float y;
  float z;
  float phi;
};

value fg_msg(value x, value y, value z, value phi) {
  CAMLparam4(x, y, z, phi);
  CAMLlocal1(s);

  struct fg msg;
  msg.x = Double_val(x);
  msg.y = Double_val(x);
  msg.z = Double_val(x);
  msg.phi = Double_val(x);

  s = alloc_string(sizeof(struct fg));
  strncpy(String_val(s), (char*)&msg, sizeof(struct fg));

  CAMLreturn (s);
}
