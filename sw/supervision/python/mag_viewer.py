import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore
import utils
from PyQt5.QtWidgets import QSizePolicy as QSP

try:
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl

    #raise ModuleNotFoundError()

    sys.path.append(utils.PAPARAZZI_SRC + "/sw/lib/python")
    sys.path.append(utils.PAPARAZZI_SRC + "/sw/tools/calibration")
    sys.path.append(utils.PAPARAZZI_HOME + "/var/lib/python") # pprzlink

    from pprzlink.message import PprzMessage


    class MagViewer(QtWidgets.QWidget):

        mag_sig = QtCore.pyqtSignal(tuple)

        def __init__(self, parent=None):
            super().__init__(parent)

            self.all_data = {}    # raw samples of every magnetometer, per (ac_id, mag_id)
            self.mag_data = np.empty((0,3))   # samples of the magnetometer shown
            self.positions = {}   # GNSS fixes (lat, lon) per aircraft, for the field strength

            self.vlay = QtWidgets.QVBoxLayout(self)

            self.cmds_lay = QtWidgets.QHBoxLayout()
            self.vlay.addLayout(self.cmds_lay)
            
            self.setup_cmds()

            self.gl_widget = gl.GLViewWidget()
            self.gl_widget.installEventFilter(self)
            self.vlay.addWidget(self.gl_widget)
            self.gl_widget.setCameraPosition(distance=20)

            # Example axis
            self.axis = gl.GLAxisItem()
            self.axis.setSize(2, 2, 2)
            self.gl_widget.addItem(self.axis)
            # Axis Texts
            self.txt_x = gl.GLTextItem(text='X', pos=[2, 0, 0])
            self.gl_widget.addItem(self.txt_x)
            self.txt_y = gl.GLTextItem(text='Y', pos=[0, 2, 0])
            self.gl_widget.addItem(self.txt_y)
            self.txt_z = gl.GLTextItem(text='Z', pos=[0, 0, 2])
            self.gl_widget.addItem(self.txt_z)
            # XY grid
            self.xy_grid = gl.GLGridItem()
            self.xy_grid.setSize(2, 2)
            self.xy_grid.setSpacing(0.2, 0.2)
            self.xy_grid.rotate(0, 1, 0, 0)
            self.gl_widget.addItem(self.xy_grid)

            shaft = np.array([[0,0,0], [1,0,0]])
            self.arrow = gl.GLLinePlotItem(pos=shaft, width=2)
            self.gl_widget.addItem(self.arrow)

            # Placeholder for scatter plot
            self.scatter = None
            self.setMinimumSize(100, 100)
            #self.gl_widget.setBackgroundColor((20, 20, 20, 255))

            self.mean_norm = 1

            self.connect = None
            self.mag_sig.connect(self.update_mag)

            self.raw_bind_id = None
            self.gps_bind_id = None
            self.ivy_sub_button.setEnabled(False)
            self.refresh_filter()

        def set_connection(self, connection):
            self.connect = connection
            self.ivy_sub_button.setEnabled(connection is not None)

        def eventFilter(self, watched, event):
            if watched is self.gl_widget:
                if event.type() == QtCore.QEvent.Wheel or (
                    event.type() == QtCore.QEvent.MouseMove
                    and event.buttons() & (QtCore.Qt.LeftButton | QtCore.Qt.MiddleButton)
                ):
                    self.distance_auto_chk.setChecked(False)
            return super().eventFilter(watched, event)
        
        def setup_cmds(self):
            mag_lbl = QtWidgets.QLabel("mags:", self)
            mag_lbl.setSizePolicy(QSP.Policy.Fixed, QSP.Policy.Fixed)
            self.cmds_lay.addWidget(mag_lbl)

            self.mag_combo = QtWidgets.QComboBox(self)
            self.mag_combo.setToolTip("Magnetometer shown. The data of all of them is collected")
            self.cmds_lay.addWidget(self.mag_combo)
            self.mag_combo.addItem("--- Select sensor in list ---", None)
            self.mag_combo.currentIndexChanged.connect(self.show_selected)

            self.ivy_sub_button = QtWidgets.QPushButton("Start listening data", self)
            self.cmds_lay.addWidget(self.ivy_sub_button)
            self.ivy_sub_button.clicked.connect(self.sub_unsub)

            clear_button = QtWidgets.QPushButton("Clear all data")
            clear_button.setSizePolicy(QSP.Policy.Fixed, QSP.Policy.Fixed)
            self.cmds_lay.addWidget(clear_button)
            clear_button.clicked.connect(self.reset_data)

            self.distance_auto_chk = QtWidgets.QCheckBox("size auto", self)
            self.distance_auto_chk.setSizePolicy(QSP.Policy.Fixed, QSP.Policy.Fixed)
            self.cmds_lay.addWidget(self.distance_auto_chk)
            self.distance_auto_chk.setChecked(True)

            calibrate_button = QtWidgets.QPushButton("Calibrate all", self)
            calibrate_button.setToolTip("Calibrate all magnetometers of the aircraft shown")
            calibrate_button.setSizePolicy(QSP.Policy.Fixed, QSP.Policy.Fixed)
            self.cmds_lay.addWidget(calibrate_button)
            calibrate_button.clicked.connect(self.calibrate)

            noise_lay = QtWidgets.QHBoxLayout()
            self.vlay.addLayout(noise_lay)
            noise_lay.addWidget(QtWidgets.QLabel("Noise threshold:", self))
            self.noise_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal, self)
            self.noise_slider.setRange(0, 10000)
            self.noise_slider.setValue(0)
            self.auto_noise_threshold = True
            self.noise_slider.setToolTip(
                "Zero selects the script's automatic threshold. Otherwise accept "
                "samples when the norm of the per-axis standard deviations "
                "over 20 neighboring samples is below this threshold.")
            noise_lay.addWidget(self.noise_slider, 1)
            self.noise_label = QtWidgets.QLabel(self)
            noise_lay.addWidget(self.noise_label)
            self.noise_slider.valueChanged.connect(self.threshold_changed)
            self.filter_status = QtWidgets.QLabel(self)
            self.vlay.addWidget(self.filter_status)
            for index in range(noise_lay.count()):
                widget = noise_lay.itemAt(index).widget()
                policy = widget.sizePolicy()
                policy.setVerticalPolicy(QSP.Fixed)
                widget.setSizePolicy(policy)
            policy = self.filter_status.sizePolicy()
            policy.setVerticalPolicy(QSP.Fixed)
            self.filter_status.setSizePolicy(policy)
        
        def stop(self):
            if self.raw_bind_id is not None:
                self.connect.ivy.unsubscribe(self.raw_bind_id)
                self.raw_bind_id = None
            if self.gps_bind_id is not None:
                self.connect.ivy.unsubscribe(self.gps_bind_id)
                self.gps_bind_id = None
            self.ivy_sub_button.setText("Start listening data")
        
        def sub_unsub(self):
            if self.raw_bind_id is None:
                self.raw_bind_id = self.connect.ivy.subscribe(self.mag_raw_cb, PprzMessage("telemetry", "IMU_MAG_RAW"))
                self.gps_bind_id = self.connect.ivy.subscribe(self.gps_cb, PprzMessage("telemetry", "GPS_INT"))
                self.ivy_sub_button.setText("Stop listening data")
            else:
                self.stop()

        def gps_cb(self, sender, msg):
            # 3D fixes, for the field strength where the magnetometer is calibrated
            if int(msg['fix']) >= 3:
                self.positions.setdefault(str(sender), []).append((float(msg['lat']) * 1e-7, float(msg['lon']) * 1e-7))

        def reset_data(self):
            self.all_data = {}
            self.mag_data = np.empty((0,3))
            self.refresh_filter()
            self.mean_norm = 1

        def mag_raw_cb(self, sender, msg):
            # Ivy thread: handled in the GUI thread
            self.mag_sig.emit((str(sender), int(msg['id']),
                               np.array([float(msg['mx']), float(msg['my']), float(msg['mz'])])))

        def update_mag(self, mag_data):
            ac_id, mag_id, mag = mag_data
            key = (ac_id, mag_id)
            if key not in self.all_data:
                self.all_data[key] = np.empty((0,3))
                conf = self.connect.conf_by_id().get(ac_id) if self.connect is not None else None
                ac_name = conf.name if conf is not None else "Aircraft " + ac_id
                self.mag_combo.addItem(f"{ac_name}: {mag_id}", key)
                if self.mag_combo.currentData() is None:
                    self.mag_combo.setCurrentIndex(self.mag_combo.count() - 1)
            self.all_data[key] = np.vstack([self.all_data[key], mag])
            if self.mag_combo.currentData() != key:
                return
            self.mag_data = self.all_data[key]
            mag_unit = mag / np.linalg.norm(mag)
            self.refresh_filter()
            shaft = np.array([[0,0,0], mag_unit*self.mean_norm])
            self.arrow.setData(pos=shaft)

            if self.mag_data.shape[0] % 10 == 0:
                self.resize_grid()

        def show_selected(self):
            key = self.mag_combo.currentData()
            self.mag_data = self.all_data.get(key, np.empty((0,3)))
            self.refresh_filter()
            if len(self.mag_data):
                self.resize_grid()

        def resize_grid(self):
            norms = np.linalg.norm(self.mag_data, axis=1)
            self.mean_norm = np.mean(norms)
            if self.distance_auto_chk.isChecked():
                self.txt_x.setData(pos=[self.mean_norm, 0, 0])
                self.txt_y.setData(pos=[0, self.mean_norm, 0])
                self.txt_z.setData(pos=[0, 0, self.mean_norm])
                #print(mean_norm)
                self.axis.setSize(self.mean_norm, self.mean_norm, self.mean_norm)
                grid_size = 2*self.mean_norm
                grid_spacing = self.mean_norm / 10
                self.xy_grid.setSize(grid_size, grid_size, grid_size)
                self.xy_grid.setSpacing(grid_spacing, grid_spacing, grid_spacing)
                self.gl_widget.setCameraPosition(distance=self.mean_norm*5)
        
        def calibrate(self):
            """Calibrate all magnetometers of the aircraft shown, as one IMU_MAG_CALIB define"""
            import calibration_utils
            from calib_result_dialog import CalibResultDialog, keep_old_calibration
            key = self.mag_combo.currentData()
            if key is None:
                summary, xml = "No magnetometer data yet: start listening and turn the aircraft in all directions.", ""
            else:
                ac_id = key[0]
                new, comments, done = {}, [], []
                # field strength where it is calibrated, from the GNSS fixes of the aircraft
                strength, location = None, ""
                pos = self.positions.get(ac_id, [])
                if pos:
                    lat, lon = np.median(np.array(pos), axis=0)
                    strength = calibration_utils.get_mag_strength_gauss(lat, lon)
                    location = f"{lat:.4f} {lon:.4f} (GNSS)"
                for (ac, mag_id) in sorted(k for k in self.all_data if k[0] == ac_id):
                    data = self.all_data[(ac, mag_id)]
                    points = data[self.accepted_indices(data)]
                    try:
                        if len(points) < 6 or np.any(np.ptp(points, axis=0) == 0):
                            raise ValueError("not enough accepted samples in different orientations")
                        p0 = calibration_utils.get_min_max_guess(points, 1.)
                        p1, msg, success = calibration_utils.optimize_calibration(points, 1., p0)
                        if success not in [1, 2, 3, 4] or not np.isfinite(p1).all():
                            raise ValueError(msg)
                        new[str(mag_id)] = calibration_utils.format_xml_struct(p1, mag_id, 11, strength, location)
                        _, norms = calibration_utils.scale_measurements(points, p1)
                        done.append(f"mag {mag_id}: {len(points)} samples, norm {norms.mean():.3f} \u00b1 {norms.std():.3f}")
                    except Exception as e:
                        comments.append(f"mag {mag_id} not calibrated: {e}")
                conf = self.connect.conf_by_id().get(ac_id) if self.connect is not None else None
                xml, kept_note = keep_old_calibration("MAG", "mag", conf, new, comments)
                summary = "\n".join(done + comments + kept_note)
                if strength is None:
                    summary += "\nNo GNSS fix received: no field strength (calibrate.py --lat --lon can add it)"
            self.result_dialog = CalibResultDialog("Magnetometer calibration", summary, xml, self)
            self.result_dialog.open()

        def noise_threshold(self, data=None):
            data = self.mag_data if data is None else data
            if self.auto_noise_threshold:
                if not len(data):
                    return 0
                neutral = np.mean(data)
                return 0.6 * np.median(np.linalg.norm(
                    data - neutral, axis=1))
            return self.noise_slider.value()

        def threshold_changed(self, value):
            self.auto_noise_threshold = value == 0
            self.refresh_filter()

        def accepted_indices(self, data=None):
            # Match calibration_utils.filter_meas with MAG's window_size=10.
            data = self.mag_data if data is None else data
            window = 10
            if len(data) <= 2 * window:
                return np.empty(0, dtype=int)
            windows = np.lib.stride_tricks.sliding_window_view(
                data, 2 * window, axis=0)[:-1]
            noise = np.linalg.norm(windows.std(axis=2), axis=1)
            return np.flatnonzero(noise < self.noise_threshold(data)) + window

        def refresh_filter(self):
            threshold = self.noise_threshold()
            if self.auto_noise_threshold:
                with QtCore.QSignalBlocker(self.noise_slider):
                    value = int(round(threshold))
                    self.noise_slider.setMaximum(max(self.noise_slider.maximum(), value))
                    self.noise_slider.setValue(value)
            self.noise_label.setText(
                f"Auto ({threshold:.1f})" if self.auto_noise_threshold
                else str(self.noise_slider.value()))
            indices = self.accepted_indices()
            colors = np.tile((1., 0., 0., 1.), (len(self.mag_data), 1))
            colors[indices] = (0., 1., 0., 1.)
            self.set_points(self.mag_data, color=colors)
            self.filter_status.setText(
                f"{len(indices)} / {len(self.mag_data)} samples accepted "
                "(green); excluded samples are red")

        def set_points(self, points, color=(1, 0, 0, 1), size=5):
            """
            points: Nx3 numpy array
            color: tuple (r, g, b, a)
            size: point size
            """
            if self.scatter is None:
                self.scatter = gl.GLScatterPlotItem(pos=points, color=color, size=size)
                self.gl_widget.addItem(self.scatter)
            else:
                self.scatter.setData(pos=points, color=color, size=size)
except ImportError:
    
    class MagViewer(QtWidgets.QWidget):
        def __init__(self, parent=None):
            super().__init__(parent)
            self.vlay = QtWidgets.QVBoxLayout(self)
            self.error_label = QtWidgets.QLabel("Module not found!\nTry to install pyqtgraph")
            self.vlay.addWidget(self.error_label)
        
        def stop(self):
            pass

        def set_connection(self, connection):
            pass
