import sys
import time

import numpy as np
from PyQt5 import QtCore, QtWidgets
import utils

try:
    import pyqtgraph as pg

    sys.path.append(utils.PAPARAZZI_SRC + "/sw/lib/python")
    sys.path.append(utils.PAPARAZZI_SRC + "/sw/tools/calibration")
    sys.path.append(utils.PAPARAZZI_HOME + "/var/lib/python")
    from pprzlink.message import PprzMessage
except ImportError:
    pg = None


class AccelCalib(QtWidgets.QWidget):
    """Collect raw accelerometer data and calibrate stationary samples."""

    accel_sig = QtCore.pyqtSignal(tuple)
    NOISE_WINDOW = 20

    def __init__(self, parent=None):
        super().__init__(parent)
        self.connect = None
        self.raw_bind_id = None
        self.samples = []
        self.times = []
        self.start_time = None
        self.dirty = False
        layout = QtWidgets.QVBoxLayout(self)
        if pg is None:
            layout.addWidget(QtWidgets.QLabel(
                "Module not found!\nInstall pyqtgraph and pprzlink to use accelerometer calibration."))
            return

        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)
        controls.addWidget(QtWidgets.QLabel("Accelerometers:"))
        self.accel_combo = QtWidgets.QComboBox(self)
        self.accel_combo.addItem("--- Select sensor in list ---", None)
        controls.addWidget(self.accel_combo, 1)
        self.ivy_sub_button = QtWidgets.QPushButton("Start listening", self)
        self.ivy_sub_button.setEnabled(False)
        self.ivy_sub_button.clicked.connect(self.sub_unsub)
        controls.addWidget(self.ivy_sub_button)
        clear_button = QtWidgets.QPushButton("Clear data", self)
        clear_button.clicked.connect(self.reset_data)
        controls.addWidget(clear_button)
        self.calibrate_button = QtWidgets.QPushButton("calibrate", self)
        self.calibrate_button.clicked.connect(self.calibrate)
        controls.addWidget(self.calibrate_button)

        noise_controls = QtWidgets.QHBoxLayout()
        layout.addLayout(noise_controls)
        noise_controls.addWidget(QtWidgets.QLabel("Noise threshold:"))
        self.noise_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
        self.noise_slider.setRange(1, 10000)
        self.noise_slider.setValue(100)
        self.noise_slider.setToolTip(
            "Accept samples when the norm of the per-axis standard deviations "
            "over 40 neighboring samples is below this threshold.")
        noise_controls.addWidget(self.noise_slider, 1)
        self.noise_label = QtWidgets.QLabel(self)
        noise_controls.addWidget(self.noise_label)
        self.noise_slider.valueChanged.connect(self.threshold_changed)

        self.plot_widget = pg.PlotWidget(self)
        self.plot_widget.setLabel("bottom", "Collection time", units="s")
        self.plot_widget.setLabel("left", "Raw acceleration")
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.addLegend()
        layout.addWidget(self.plot_widget)
        self.curves = []
        self.accepted_curves = []
        for axis, color in zip(("ax", "ay", "az"), ("r", "g", "b")):
            curve = self.plot_widget.plot(pen=pg.mkPen(color, width=1), name=axis)
            curve.setDownsampling(auto=True, method="peak")
            self.curves.append(curve)
            self.accepted_curves.append(self.plot_widget.plot(
                pen=None, symbol="o", symbolSize=4, symbolBrush=color,
                symbolPen=None))
        self.status_label = QtWidgets.QLabel(self)
        layout.addWidget(self.status_label)
        layout.addWidget(QtWidgets.QLabel(
            "Hold the sensor still in several orientations covering both directions "
            "of each axis. Dots mark samples accepted for calibration."))

        self.accel_combo.currentIndexChanged.connect(self.reset_data)
        self.accel_sig.connect(self.update_accel)
        self.plot_timer = QtCore.QTimer(self)
        self.plot_timer.setInterval(100)
        self.plot_timer.timeout.connect(self.refresh_plot)
        self.plot_timer.start()
        self.threshold_changed()
        self.refresh_plot()

    def set_connection(self, connection):
        self.connect = connection
        if pg is not None:
            self.ivy_sub_button.setEnabled(connection is not None)

    def sub_unsub(self):
        if self.raw_bind_id is None:
            self.start_time = None

            def callback(sender, msg):
                self.accel_sig.emit((str(sender), int(msg['id']),
                                     time.monotonic(),
                                     (float(msg['ax']), float(msg['ay']), float(msg['az']))))

            self.raw_bind_id = self.connect.ivy.subscribe(
                callback, PprzMessage("telemetry", "IMU_ACCEL_RAW"))
            self.ivy_sub_button.setText("Stop listening")
        else:
            self.connect.ivy.unsubscribe(self.raw_bind_id)
            self.raw_bind_id = None
            self.ivy_sub_button.setText("Start listening")

    def update_accel(self, data):
        ac_id, sensor_id, timestamp, sample = data
        if self.raw_bind_id is None:
            return
        key = (ac_id, sensor_id)
        index = next((i for i in range(self.accel_combo.count())
                      if self.accel_combo.itemData(i) == key), -1)
        if index == -1:
            conf = self.connect.conf_by_id().get(ac_id)
            aircraft = conf.name if conf is not None else "Aircraft " + ac_id
            self.accel_combo.addItem(f"{aircraft}: {sensor_id}", key)
        if self.accel_combo.currentData() != key or not np.isfinite(sample).all():
            return
        if self.start_time is None:
            # Continue from the last sample, excluding the gap between sessions.
            self.start_time = timestamp - (self.times[-1] if self.times else 0)
        self.times.append(timestamp - self.start_time)
        self.samples.append(sample)
        self.dirty = True

    def reset_data(self):
        self.samples.clear()
        self.times.clear()
        self.start_time = None
        self.dirty = True
        self.refresh_plot()

        self.plot_widget.setRange(xRange=(0, 1), yRange=(-1, 1), padding=0)
        self.plot_widget.enableAutoRange(x=True, y=True)

    def threshold_changed(self):
        self.noise_label.setText(str(self.noise_slider.value()))
        self.dirty = True

    def accepted_indices(self, samples):
        """Match calibration_utils.filter_meas without importing plotting dependencies."""
        window = self.NOISE_WINDOW
        if len(samples) <= 2 * window:
            return np.empty(0, dtype=int)
        windows = np.lib.stride_tricks.sliding_window_view(
            samples, 2 * window, axis=0)[:-1]
        noise = np.linalg.norm(windows.std(axis=2), axis=1)
        return np.flatnonzero(noise < self.noise_slider.value()) + window

    def refresh_plot(self):
        if not self.dirty:
            return
        samples = np.asarray(self.samples, dtype=float).reshape(-1, 3)
        times = np.asarray(self.times)
        indices = self.accepted_indices(samples)
        for axis, (curve, accepted) in enumerate(zip(self.curves, self.accepted_curves)):
            curve.setData(times, samples[:, axis])
            accepted.setData(times[indices], samples[indices, axis])
        self.status_label.setText(
            f"{len(samples)} samples collected; {len(indices)} accepted for calibration")
        self.calibrate_button.setEnabled(len(indices) >= 6)
        self.dirty = False

    def calibrate(self):
        try:
            import calibration_utils

            samples = np.asarray(self.samples, dtype=float).reshape(-1, 3)
            accepted = samples[self.accepted_indices(samples)]
            if len(accepted) < 6 or np.any(np.ptp(accepted, axis=0) == 0):
                raise ValueError("Collect more stationary samples in different orientations.")
            p0 = calibration_utils.get_min_max_guess(accepted, 9.81)
            p1, message, success = calibration_utils.optimize_calibration(accepted, 9.81, p0)
            if success not in (1, 2, 3, 4) or not np.isfinite(p1).all():
                raise ValueError("Calibration failed: " + message)
            text = calibration_utils.format_xml(
                p1, "ACCEL", self.accel_combo.currentData()[1], 10)
        except Exception as error:
            text = "Calibration error\n" + str(error)
        self.calibration_dialog = QtWidgets.QMessageBox(
            QtWidgets.QMessageBox.Information, "Accelerometer calibration", text,
            QtWidgets.QMessageBox.Ok, self)
        self.calibration_dialog.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.calibration_dialog.open()

    def stop(self):
        if self.connect is not None and pg is not None:
            self.plot_timer.stop()
            if self.raw_bind_id is not None:
                self.connect.ivy.unsubscribe(self.raw_bind_id)
                self.raw_bind_id = None
            self.ivy_sub_button.setText("Start listening")
