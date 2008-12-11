#include <stdio.h>
#include <string.h>
#include <caml/mlvalues.h>
#include <caml/callback.h>
#include <caml/memory.h>
#include <caml/alloc.h>
#include <caml/custom.h>


void print_ascii_to_binary(int n, char* msg)
{
  static value * caml_to_binary_closure = NULL;
  if (caml_to_binary_closure == NULL)
    caml_to_binary_closure = caml_named_value("to_binary");
  
  value s = caml_callback2(*caml_to_binary_closure, Val_int(n), caml_copy_string(msg));

  int i;
  for(i = 0; i < string_length(s); i++)
    printf("%02x ", Byte_u(s, i));
  printf("\n");
}

int main(int argc, char** argv) {
  caml_startup(argv);
  
  print_ascii_to_binary(42, "ATTITUDE 7 22 33");
  
  return 0;
}
