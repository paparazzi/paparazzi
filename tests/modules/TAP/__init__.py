import sys
import atexit

# todo: make written-to stream passable

class Plan(object):
  def __init__(self, plan, param=None):
    self.counter = 0
    self.expected_tests = None
    self.ended = False

    if isinstance(plan, int):
      self.expected_tests = plan
      print(("1..%u" % self.expected_tests))
    elif plan == "no_plan" or plan == None: 1
    elif plan == "skip_all":
      print(("1..0 # skip %s" % param))
      raise SystemExit(0) # ??? this is what T::B does, but sucks
    else:
      raise TestBadPlan(plan)

  def increment_counter(self):
    self.counter += 1

  def __del__(self):
    if self.ended: return
    self.ended = True
    if self.expected_tests is None:
      print(("1..%u" % self.counter))
    elif self.counter != self.expected_tests:
      print(("# Looks like you planned %u tests but ran %u." \
        % (self.expected_tests, self.counter)))

class Builder(object):
  global_defaults = {
    "_plan":    None,
    "current":  1,
    "has_plan": False,
  }
  global_test_builder = global_defaults.copy()

  def __init__(self, plan=None, plan_param=None):
    self.__dict__ = self.global_test_builder
    if plan: self.set_plan(plan, plan_param)

  @classmethod
  def create(cls, plan=None, plan_param=None):
    return Builder(plan, plan_param)

  def set_plan(self, plan, plan_param=None):
    if self.get_plan(): raise TestPlannedAlready(plan, plan_param)
    self._plan = Plan(plan, plan_param)
    atexit.register(self._plan.__del__)

  def get_plan(self): return self._plan
 
  def ok(self, is_ok, desc=None, skip=None, todo=None):
    self.get_plan().increment_counter()

    if skip and todo: raise TestBadDirective(self)

    if is_ok: report = "ok" 
    else:     report = "not ok"

    sys.stdout.write("%s %u" % (report, self.current))

    if desc: sys.stdout.write(" - %s" % desc)
    if skip: sys.stdout.write(" # SKIP %s" % skip)
    if todo: sys.stdout.write(" # TODO %s" % todo)

    print("")

    self.current += 1

    return is_ok

  def reset(self):
    self.__dict__.clear()
    for key in list(self.global_defaults.keys()):
      self.__dict__[key] = self.global_defaults[key]

class TestPlannedAlready(Exception):
  def __init__(self, plan, param=None):
    self.plan  = plan
    self.param = param
  def __str__(self):
    return "tried to plan twice; second plan: %s, %s" % self.plan, self.param

class TestWithoutPlan(Exception):
  def __str__(self):
    return "tried running tests without a plan"

class TestBadPlan(Exception):
  def __init__(self, plan):
    self.plan = plan
  def __str__(self):
    return "didn't understand plan '%s'" % self.plan

class TestBadDirective(Exception):
  def __str__(self):
    return "tried running a test with more than one directive"
