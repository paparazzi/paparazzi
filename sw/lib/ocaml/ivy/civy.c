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

value ivy_sendMsg(value msg)
{
  IvySendMsg(String_val(msg));
  return Val_unit;
}

value ivy_stop(value unit)
{
  IvyStop ();
  return Val_unit;
}


void app_cb(IvyClientPtr app, void *user_data, IvyApplicationEvent event )
{
  value closure = *(value*)user_data;
  callback2(closure, Val_int(app), Val_int(event));
}

value ivy_init(value vappName, value vready, value closure_name)
{
  value * closure = caml_named_value(String_val(closure_name));
  char * appName = malloc(strlen(String_val(vappName))+1); /* Memory leak */
  strcpy(appName, String_val(vappName));
  char * ready = malloc(strlen(String_val(vready))+1); /* Memory leak */
  strcpy(ready, String_val(vready));
  IvyInit(appName, ready, app_cb, (void*)closure, 0, 0); /* When the "die callback" is called ??? */
  return Val_unit;
}

value ivy_start(value bus)
{
  IvyStart(String_val(bus));
  return Val_unit;
}

void ClosureCallback(IvyClientPtr app, void *closure, int argc, char **argv)
{
  char* t[argc+1];
  int i;
  /* Copie de argv dans t avec ajout d'un pointeur nul a la fin */
  for(i=0; i < argc; i++) t[i] = argv[i];
  t[argc] = (char*)0L;
  callback2(*(value*)closure, Val_long(app), copy_string_array((char const **)t));
}

value ivy_bindMsg(value cb_name, value regexp)
{
  value * closure = caml_named_value(String_val(cb_name));
  MsgRcvPtr id = IvyBindMsg(ClosureCallback, (void*)closure, String_val(regexp));
  return Val_long(id);
}

value ivy_unbindMsg(value id)
{
  IvyUnbindMsg((MsgRcvPtr)Long_val(id));
  return Val_unit;
}

value ivy_name_of_client(value c)
{
  return copy_string(IvyGetApplicationName((IvyClientPtr)Long_val(c)));
}
value ivy_host_of_client(value c)
{
  return copy_string(IvyGetApplicationHost((IvyClientPtr)Long_val(c)));
}

void cb_delete_channel(void *delete_read)
{
}

void cb_write_channel(Channel ch, IVY_HANDLE fd, void *closure)
{
}

void cb_read_channel(Channel ch, IVY_HANDLE fd, void *closure)
{
  callback(*(value*)closure, Val_int(ch));
}
