#!/usr/bin/python3
import sys
import signal
import re
from os import path, getenv
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QAction, QToolButton, QMenu
import argparse
from ui_rt_plotter import Ui_RT_Plotter
from time import sleep
import numpy as np
import pyqtgraph as pg
from dataclasses import dataclass, Field, field
from typing import List, Dict, Optional

# if PAPARAZZI_SRC or PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PPRZ_HOME = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))
PPRZ_SRC = getenv("PAPARAZZI_SRC", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../../../')))

sys.path.append(PPRZ_SRC + "/sw/lib/python")
sys.path.append(PPRZ_HOME + "/var/lib/python")  # pprzlink

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage


@dataclass()
class Curve:
    field: str
    scale: float
    offset: float
    nb: int
    action: QAction
    time: np.ndarray = field(default=None)
    data: np.ndarray = field(default=None)
    last_data: Optional[float] = field(default=None)
    plot: pg.PlotDataItem = field(default=None)
    bind: Field = field(default=None)


@dataclass()
class Constant:
    value: float
    action: QAction
    plot: pg.PlotDataItem


class CurvesCollection:
    def __init__(self):
        self.data = {}  # type: Dict[str, Dict[str, Dict[str, Curve]]]  ## {ac_id:{ msg_name:{key:Curve}}}

    def curve_exists(self, ac_id, msg_name, key) -> bool:
        try:
            c = self.get_curve(ac_id, msg_name, key)
            return True
        except KeyError:
            return False

    def get_curve(self, ac_id, msg_name, key) -> Curve:
        """Throw KeyError if this curve does not exists"""
        return self.data[ac_id][msg_name][key]

    def remove_curve(self, ac_id, msg_name, key):
        """Throw KeyError if this curve does not exists"""
        del self.data[ac_id][msg_name][key]

    def add_curve(self, ac_id, msg_name, key, c):
        dam = self.data.setdefault(ac_id, {}).setdefault(msg_name, {})
        dam[key] = c

    def get_all_curves(self) -> List[Curve]:
        all_curves = []
        for ac_id in self.data:
            for msg_name in self.data[ac_id]:
                for key in self.data[ac_id][msg_name]:
                    c = self.data[ac_id][msg_name][key]
                    all_curves.append(c)
        return all_curves

    def curves(self, ac_id, msg_name) -> List[Curve]:
        return list(self.data.get(ac_id, {}).get(msg_name, {}).values())


class Plotter(QWidget, Ui_RT_Plotter):
    """
    Main plotter class
    """

    def __init__(self, parent, ivy):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        self.plot = pg.PlotWidget()
        self.plot.setAcceptDrops(True)
        self.plot.dragMoveEvent = self.dragMoveEvent
        self.plot.dragEnterEvent = self.dragEnterEvent
        self.plot.dragLeaveEvent = self.dragLeaveEvent
        self.plot.dropEvent = self.dropEvent
        self.plot.setRange(xRange=(-30.0, 0.))
        self.plot.disableAutoRange(pg.ViewBox.XAxis)
        self.plot.setMouseEnabled(x=False, y=True)
        self.plot.addLegend(offset=(1, 1))
        self.plot.showGrid(x=True, y=True)
        self.plot.setBackground('w')
        # self.plot.setLabel('bottom','time (s)')
        self.gridLayout.addWidget(self.plot, 1, 0, 1, 14)
        self.menu_button.setPopupMode(QToolButton.InstantPopup)
        self.menu = QMenu(self.menu_button)
        self.menu.addAction(self.action_new_plot)
        self.menu.addAction(self.action_reset)
        self.menu.addAction(self.action_suspend)
        self.menu.addAction(self.action_restart)
        self.menu.addAction(self.action_dark_background)
        self.menu.addSeparator()
        self.menu.addAction(self.action_close)
        self.menu.addAction(self.action_quit)
        self.menu.addSeparator()
        self.menu_button.setMenu(self.menu)
        self.action_new_plot.triggered.connect(self.open_new_plotter)
        self.action_reset.triggered.connect(self.reset_plotter)
        self.action_suspend.triggered.connect(self.suspend_plotter)
        self.action_restart.triggered.connect(self.restart_plotter)
        self.action_dark_background.triggered.connect(self.set_background)
        self.action_close.triggered.connect(self.close_window)
        self.action_quit.triggered.connect(self.quit_plotter)
        self.constant_input.returnPressed.connect(self.draw_constant)
        self.ivy = ivy
        self.pattern = re.compile("^([0-9]+):(\\w+):(\\w+):(\\w+):([0-9]+.[0-9]*).*$")
        self.data = CurvesCollection()
        self.constants = {}             # type: Dict[float, Constant]
        self.dt_slider.valueChanged.connect(self.set_dt)
        self.size_slider.valueChanged.connect(self.set_size)
        self.autoscale.clicked.connect(lambda: self.plot.enableAutoRange(x=False, y=True))
        self.timer = QtCore.QTimer()
        self.timer.setInterval(self.dt_slider.value() * 10)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()
        self.suspended = False
        self.nb_color = 8
        self.idx_color = 0
        # max display size (s) / min update time (s) (with rounding)
        self.max_size = int(self.size_slider.maximum() / (self.dt_slider.minimum() / 100) + 0.5)

    def get_dt(self):
        return self.dt_slider.value() / 100.

    def set_dt(self):
        self.dt_label.setText("{:1.2f} s".format(self.get_dt()))
        self.timer.setInterval(int(self.dt_slider.value() * 10))

    def set_size(self):
        size = self.size_slider.value()
        self.size_label.setText("{} s".format(size))
        self.plot.setRange(xRange=(-size, 0.))

    def update_plot_data(self):
        for curve in self.data.get_all_curves():
            x = curve.time - self.get_dt()
            y = curve.data
            if curve.last_data is not None:
                x = np.roll(x, -1)
                x[-1] = 0.
                y = np.roll(y, -1)
                y[-1] = curve.scale * curve.last_data + curve.offset
                curve.nb = min(curve.nb + 1, self.max_size)
                curve.last_data = None
                curve.data = y
            curve.time = x
            if not self.suspended:
                curve.plot.setData(x[-curve.nb:], y[-curve.nb:])

    def closing(self):
        """ shutdown Ivy and window """
        pass

    def dragLeaveEvent(self, e):
        pass

    def dragMoveEvent(self, e):
        e.accept()

    def dragEnterEvent(self, e):
        if e.mimeData().hasFormat('text/plain'):
            e.accept()
        else:
            e.ignore()

    def dropEvent(self, event):
        event.accept()
        match = self.pattern.fullmatch(event.mimeData().text())
        if match is None:
            return
        ac_id, class_name, msg_name, field, sc = match.group(1, 2, 3, 4, 5)
        scale = float(sc) * self.scale_spin.value()
        offset = self.offset_spin.value()
        key = '{}:{}+{}'.format(field, scale, offset)

        if self.data.curve_exists(ac_id, msg_name, key):
            return

        menu_item = QAction("remove {}".format(match.group(0)))
        plot = self.plot.plot([], [], pen=(self.idx_color, self.nb_color),
                              name='{}:{}:{}:{}'.format(ac_id, class_name, msg_name, key))
        bind = ivy.subscribe(self.msg_callback, '^({} {} .*)'.format(ac_id, msg_name))

        c = Curve(field=field,
                  data=np.zeros(self.max_size),
                  last_data=None,
                  time=np.zeros(self.max_size),
                  scale=scale,
                  offset=offset,
                  nb=0,
                  action=menu_item,
                  plot=plot,
                  bind=bind)

        self.data.add_curve(ac_id, msg_name, key, c)

        self.idx_color = (self.idx_color + 1) % self.nb_color
        self.menu.addAction(menu_item)
        menu_item.triggered.connect(lambda: self.remove_curve(ac_id, msg_name, key))

    def msg_callback(self, ac_id, msg):
        for c in self.data.curves(str(ac_id), msg.name):
            c.last_data = float(msg[c.field])

    def remove_curve(self, ac_id, msg_name, key):
        curve = self.data.get_curve(ac_id, msg_name, key)
        self.ivy.unbind(curve.bind)
        plot_id = curve.plot
        self.data.remove_curve(ac_id, msg_name, key)
        self.plot.removeItem(plot_id)

    def open_new_plotter(self):
        self.parent().open_new_window()

    def reset_plotter(self):
        for c in self.data.get_all_curves():
            c.nb = 0

    def suspend_plotter(self):
        self.suspended = True

    def restart_plotter(self):
        self.suspended = False

    def close_window(self):
        self.parent().close()

    def quit_plotter(self):
        QApplication.exit()

    def set_background(self, dark):
        if dark:
            self.plot.setBackground('k')
        else:
            self.plot.setBackground('w')

    def draw_constant(self):
        try:
            value = float(self.constant_input.text())
            if value not in self.constants:
                action = QAction('remove {}'.format(value))
                self.menu.addAction(action)
                plot = self.plot.plot([-self.size_slider.maximum(), 0], [value, value],
                                      name='constant {}'.format(value), pen='b')
                c = Constant(value=value, action=action, plot=plot)
                action.triggered.connect(lambda: self.remove_constant(c))
                self.constants[value] = c

        except ValueError:
            print("Input constant value is not a number")

    def remove_constant(self, c: Constant):
        self.plot.removeItem(c.plot)
        self.menu.removeAction(c.action)
        del self.constants[c.value]



class MainWindow(QMainWindow):

    def __init__(self, ivy, new_window):
        super().__init__()
        icon = QtGui.QIcon(path.join(PPRZ_HOME, "data", "pictures", "penguin_icon_rtp.png"))
        self.setWindowIcon(icon)
        self.setGeometry(0, 0, 900, 300)
        self.setMinimumSize(600, 200)
        self.ivy = ivy
        self.new_window = new_window
        self.plotter = Plotter(self, ivy)
        self.setCentralWidget(self.plotter)

    def close_plotter(self):
        self.plotter.closing()

    def open_new_window(self):
        self.new_window()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Real-time plotter")

    try:
        app = QtWidgets.QApplication(sys.argv)
        ivy = IvyMessagesInterface("natnet2ivy")
        windows = []


        def make_new_window():
            w = MainWindow(ivy, make_new_window)
            windows.append(w)
            w.show()


        def closing():
            ivy.shutdown()
            for w in windows:
                w.close_plotter()


        def sigint_handler(*args):
            """Handler for the SIGINT signal."""
            app.quit()


        app.aboutToQuit.connect(closing)
        signal.signal(signal.SIGINT, sigint_handler)
        make_new_window()
        sys.exit(app.exec_())

    except Exception as e:
        print('exit on exception:', e)
        ivy.shutdown()
