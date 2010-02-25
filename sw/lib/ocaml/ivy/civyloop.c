#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <Ivy/timer.h>
#include <caml/mlvalues.h>
#include <caml/fail.h>
#include <caml/callback.h>
#include <caml/memory.h>
#include <caml/alloc.h>

value ivy_mainLoop(value unit)
{
  IvyMainLoop (NULL, NULL);
  return Val_unit;
}

void timer_cb(TimerId id, void *data, unsigned long delta)
{
  value closure = *(value*)data;
  callback(closure, Val_int((int) id));
}

value ivy_timerRepeatafter(value nb_ticks,value delay, value closure_name)
{
  value * closure = caml_named_value(String_val(closure_name));
  TimerId id = TimerRepeatAfter(Int_val(nb_ticks), Int_val(delay), timer_cb, (void*)closure);
  return Val_int(id);
}

/* Data associated to Channel callbacks is the couple of delete and
read closures */

void cb_delete_channel(void *delete_read);
void cb_read_channel(Channel ch, HANDLE fd, void *closure);


value ivy_channelSetUp(value fd, value closure_name)
{
  Channel c;
  value * closure = caml_named_value(String_val(closure_name));

  c = IvyChannelAdd((HANDLE)Int_val(fd), (void*)closure, cb_delete_channel, cb_read_channel);
  return Val_int(c);
}

value ivy_timerRemove(value t)
{
  TimerRemove((TimerId)Int_val(t));
  return Val_unit;
}


value ivy_channelClose(value ch)
{
  IvyChannelRemove((Channel)Int_val(ch));
  return Val_unit;
}
