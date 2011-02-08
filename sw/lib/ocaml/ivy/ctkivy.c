#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include <timer.h>
#include <caml/mlvalues.h>
#include <caml/fail.h>
#include <caml/callback.h>
#include <caml/memory.h>
#include <caml/alloc.h>
#include "ivytcl.h"

extern void cb_delete_channel(void *delete_read);
extern void cb_read_channel(Channel ch, IVY_HANDLE fd, void *closure);

value ivy_TclmainLoop(value unit)
{
  Tk_MainLoop();
  return Val_unit;
}


value ivy_TclchannelSetUp(value fd, value closure_name)
{
  Channel c;
  value * closure = caml_named_value(String_val(closure_name));

  c = IvyTclChannelSetUp((IVY_HANDLE)Int_val(fd), (void*)closure, cb_delete_channel, cb_read_channel);
  return Val_int(c);
}

value ivy_TclchannelClose(value ch)
{
  IvyTclChannelClose((Channel)Int_val(ch));
  return Val_unit;
}
