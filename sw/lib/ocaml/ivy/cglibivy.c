#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include <Ivy/timer.h>
#include <Ivy/ivychannel.h>
#include <Ivy/ivyglibloop.h>
#include <glib.h>
#include <caml/mlvalues.h>
#include <caml/fail.h>
#include <caml/callback.h>
#include <caml/memory.h>
#include <caml/alloc.h>

value ivy_GtkmainLoop(value unit)
{
  g_main_loop_run(g_main_loop_new(NULL, FALSE));
  return Val_unit;
}

extern void cb_delete_channel(void *delete_read);
extern void cb_read_channel(Channel ch, HANDLE fd, void *closure);

value ivy_GtkchannelSetUp(value fd, value closure_name)
{
  Channel c;
  value * closure = caml_named_value(String_val(closure_name));

  c = IvyChannelAdd((HANDLE)Int_val(fd), (void*)closure, cb_delete_channel, cb_read_channel);
  return Val_int(c);
}

value ivy_GtkchannelClose(value ch)
{
  IvyChannelRemove((Channel)Int_val(ch));
  return Val_unit;
}
