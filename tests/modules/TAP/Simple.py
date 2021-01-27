
import TAP
import re

builder = TAP.Builder()

def plan(plan, plan_param=None):
  builder.set_plan(plan, plan_param)

ok = builder.ok

def diag(comment):
  print(("# %s" % re.compile("\n(.)").sub(comment, '\n#\1')))

def eq_ok(have, want, comment):
  okness = have == want
  ok(okness, comment)
  if not okness:
    diag("want: %s" % want)
    diag("have: %s" % have)
  return okness

def is_ok(have, want, comment):
  okness = have is want
  ok(okness, comment)
  if not okness:
    diag("want id: %s" % id(want))
    diag("have id: %s" % id(have))
  return okness

def isa_ok(object, cls, object_name="the object"):
  okness = isinstance(object, cls)
  ok(okness, object_name + " is a " + repr(cls))
  # if not okness:
  #   diag("want id: %s" % id(want))
  #   diag("have id: %s" % id(have))
  return okness
