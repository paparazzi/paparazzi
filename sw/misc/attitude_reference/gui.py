from gi.repository import Gtk, GObject

import threading

import pat.utils as pu


class Worker(GObject.GObject):
    """a worker thread that avoid blocking the UI during long tasks"""
    __gsignals__ = {
        "progress": (
            GObject.SIGNAL_RUN_LAST, GObject.TYPE_NONE, [GObject.TYPE_FLOAT]),
        "completed": (
            GObject.SIGNAL_RUN_LAST, GObject.TYPE_NONE, []),
        "aborted": (
            GObject.SIGNAL_RUN_LAST, GObject.TYPE_NONE, [])
    }

    def __init__(self, n_step=100):
        GObject.GObject.__init__(self)
        self.running = False
        self.canceled = False
        self.do_work = False
        self.n_step = n_step

    def start(self, args):
        if self.running:
            self.cancel()
        self.t = threading.Thread(target=self._work, args=args)
        self.t.daemon = True
        self.t.start()

    def cancel(self, wait=True):
        self.canceled = True
        self.do_work = False
        if wait:
            self.t.join()

    def _emit(self, *args):
        GObject.idle_add(GObject.GObject.emit, self, *args)

    def _work(self, args):
        self.running = True
        self.canceled = False
        self.do_work = False
        #print("starting new work thread with args %s\nnow calling work init" % args)
        self._work_init(args)
        for i in range(0, self.n_step):
            if self.canceled:
                self.running = False
                self._emit("aborted")
                return
            self._work_step(i, args)
            self._emit("progress", float(i) / self.n_step)
        self.running = False
        self._emit("completed")


class AttRefParamView(Gtk.Frame):
    """a graphical user interface for editing parameters of an attitude reference"""

    def __init__(self, txt=None, ref_classes=[], active_impl=None):
        Gtk.Frame.__init__(self)
        if txt is not None:
            lab = Gtk.Label()
            lab.set_markup(txt)
            self.set_label_widget(lab)
        self.ref_classes = ref_classes
        self.b = Gtk.Builder()
        self.b.add_from_file("ressources/att_ref_param_view.xml")
        self.b.get_object("main_grid").reparent(self)
        self.combo_type = self.b.get_object("comboboxtext_references")
        for c in self.ref_classes:
            self.combo_type.append_text(c.name)
        self.progress = self.b.get_object("progressbar")

        self.spin_cfg = {
            'omega': {'range': (0.2, 20., 0.1, 1., 0.), 'r2d': lambda x: x, 'd2r': lambda x: x},
            'xi': {'range': (0.1, 1.5, 0.05, 0.2, 0.), 'r2d': lambda x: x, 'd2r': lambda x: x},
            'sat_vel': {'range': (1., 500., 1., 5., 0.), 'r2d': pu.deg_of_rad, 'd2r': pu.rad_of_deg},
            'sat_accel': {'range': (10., 2000., 10., 20., 0.), 'r2d': pu.deg_of_rad, 'd2r': pu.rad_of_deg},
            'sat_jerk': {'range': (10., 7500., 10., 20., 0.), 'r2d': pu.deg_of_rad, 'd2r': pu.rad_of_deg}
        }
        for n, c in self.spin_cfg.iteritems():
            w = self.b.get_object("spin_{}".format(n))
            adj = Gtk.Adjustment(0., *c['range'])
            w.set_adjustment(adj)

        if active_impl is not None:
            self.update_view(active_impl)
        else:
            self.combo_type.set_active(0)

    def connect(self, cbk_type_changed, cbk_param_changed, *args):
        self.combo_type.connect("changed", cbk_type_changed, *args)
        for n in self.spin_cfg.keys():
            w = self.b.get_object("spin_{}".format(n))
            self.spin_cfg[n]['handler'] = w.connect("value-changed", cbk_param_changed, n, *args)

    def get_selected(self):
        return self.combo_type.get_active()

    def set_active(self, ref_class):
        self.combo_type.set_active(self.ref_classes.index(ref_class))

    def get_selected_ref_class(self):
        return self.ref_classes[self.get_selected()]

    def update_view(self, ref):
        """ update view according to ref params """
        self.set_active(ref.__class__)
        for n, c in self.spin_cfg.iteritems():
            w = self.b.get_object("spin_{}".format(n))
            try:
                w.set_value(c['r2d'](getattr(ref, n)[0]))
                w.set_sensitive(True)
            except AttributeError:
                w.set_sensitive(False)

    def update_ref_params(self, ref):
        """" upate ref params from view by emitting value-changed.
        if param was not sensitive, update view with current param from ref
        """
        for n, c in self.spin_cfg.iteritems():
            w = self.b.get_object("spin_{}".format(n))
            if hasattr(ref, n):
                if not w.is_sensitive():
                    w.set_value(c['r2d'](getattr(ref, n)[0]))
                w.emit('value-changed')
                w.set_sensitive(True)
            else:
                w.set_sensitive(False)
