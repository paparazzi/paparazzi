"""
SD log extractor (Utilities tab).

Lists the logs of a mounted SD card (or a folder), extracts them one after another with
sw/logalizer/sd2log into the output folder, and remembers which logs were extracted in
$PAPARAZZI_HOME/var/sd_log_state.json. The options are saved in the Paparazzi Center settings.
"""

from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QProcess, Qt
import utils
from generated.ui_log_widget import Ui_LogWidget
from conf import *
import datetime
import hashlib
import json
import os
import atexit
import re
import shutil
import tempfile
import time

# Output of sd2log before it is moved to the output folder, private to this process
TMP_DIR = tempfile.mkdtemp(prefix="pprz_sd2log_")
atexit.register(shutil.rmtree, TMP_DIR, True)
STATE_FILE = os.path.join(utils.PAPARAZZI_HOME, "var", "sd_log_state.json")
DEFAULT_OUTPUT = "$PAPARAZZI_HOME/var/logs/$YY-$MM/$DD/$AIRCRAFT"

ENVS = [
    "PAPARAZZI_HOME",
    "PAPARAZZI_SRC",
    "HOME"
]

DATE_TAGS = ["YY", "MM", "DD", "hh", "mm", "ss"]
OTHER_TAGS = ["AIRCRAFT", "AC_ID"]

# Mounted SD cards: removable media mounted by the desktop with a FAT file system
MOUNT_DIRS = ("/media/", "/run/media/")
CARD_FS = ("vfat", "msdos", "exfat")
# FUSE file systems (exFAT on some systems, but also NTFS disks): only with SD logs on it
FUSE_FS = ("fuseblk",)
SCAN_INTERVAL_MS = 3000

COL_FILE, COL_SIZE, COL_DATE, COL_AIRCRAFT, COL_STATUS = range(5)


def natural_key(name):
    """Sort log names like the SD logger numbers them (fr_0009 before fr_0010)"""
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r'(\d+)', name)]


def log_key(path):
    """Identify a log by its size and the hash of its first MB (a renamed or copied log stays the same)"""
    h = hashlib.sha1()
    with open(path, "rb") as f:
        h.update(f.read(1 << 20))
    return f"{os.path.getsize(path)}:{h.hexdigest()}"


def human_size(size):
    for unit in ("B", "kB", "MB", "GB"):
        if size < 1000 or unit == "GB":
            return f"{size:.0f} {unit}" if unit == "B" else f"{size:.1f} {unit}"
        size /= 1000


def detect_cards():
    """Mount points of SD cards, from /proc/mounts"""
    cards = []
    try:
        with open("/proc/mounts") as mounts:
            for line in mounts:
                parts = line.split()
                if len(parts) < 3:
                    continue
                mount = parts[1].replace("\\040", " ")
                if mount.startswith(MOUNT_DIRS) and \
                        (parts[2] in CARD_FS or (parts[2] in FUSE_FS and find_logs(mount))):
                    cards.append(mount)
    except OSError:
        pass
    return sorted(cards)


def find_logs(folder):
    """The .LOG files in a folder and in its direct sub folders"""
    logs = []
    try:
        entries = list(os.scandir(folder))
    except OSError:
        return logs
    for entry in entries:
        try:
            if entry.is_file() and entry.name.upper().endswith(".LOG"):
                logs.append(entry.path)
            elif entry.is_dir() and not entry.name.startswith("."):
                for sub in os.scandir(entry.path):
                    if sub.is_file() and sub.name.upper().endswith(".LOG"):
                        logs.append(sub.path)
        except OSError:
            continue
    return sorted(logs, key=lambda p: natural_key(os.path.basename(p)))


class DeleteLogsDialog(QDialog):
    """Ask which logs to delete from the SD card: all of them, or all except the last ones"""

    def __init__(self, logs, extracted, keep_last, parent=None):
        QDialog.__init__(self, parent)
        self.setWindowTitle("Delete logs from SD card")
        self.logs = logs              # all logs, oldest first
        self.extracted = extracted    # logs which were extracted
        layout = QVBoxLayout(self)
        self.all_radio = QRadioButton("All logs")
        self.except_radio = QRadioButton("All logs except the last")
        self.keep_spin = QSpinBox()
        self.keep_spin.setRange(1, 999)
        self.keep_spin.setValue(keep_last)
        self.except_radio.setChecked(True)
        row = QHBoxLayout()
        row.addWidget(self.except_radio)
        row.addWidget(self.keep_spin)
        row.addStretch()
        layout.addWidget(self.all_radio)
        layout.addLayout(row)
        self.only_extracted = QCheckBox("Only logs which were extracted")
        self.only_extracted.setChecked(True)
        layout.addWidget(self.only_extracted)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        self.delete_button = self.buttons.addButton("Delete", QDialogButtonBox.DestructiveRole)
        self.delete_button.clicked.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        layout.addWidget(self.buttons)
        for w in (self.all_radio, self.except_radio, self.only_extracted):
            w.toggled.connect(self.update_summary)
        self.keep_spin.valueChanged.connect(self.update_summary)
        self.update_summary()

    def selected_logs(self):
        logs = self.logs
        if self.except_radio.isChecked():
            keep = self.keep_spin.value()
            logs = logs[:-keep] if keep < len(logs) else []
        if self.only_extracted.isChecked():
            logs = [p for p in logs if p in self.extracted]
        return logs

    def update_summary(self):
        self.keep_spin.setEnabled(self.except_radio.isChecked())
        logs = self.selected_logs()
        self.delete_button.setText(f"Delete {len(logs)} log{'s' if len(logs) != 1 else ''}")
        self.delete_button.setEnabled(len(logs) > 0)
        not_extracted = [p for p in logs if p not in self.extracted]
        text = f"{len(logs)} of {len(self.logs)} logs will be deleted from the card."
        if not_extracted:
            text += f" <b>{len(not_extracted)} of them were never extracted.</b>"
        self.summary.setText(text)


class LogWidget(QWidget, Ui_LogWidget):

    extracted = QtCore.pyqtSignal(str)  # path of the extracted .data file

    def __init__(self, parent=None, *args, **kwargs):
        QWidget.__init__(self, parent=parent, *args, **kwargs)
        self.setupUi(self)
        self.conf: Conf = None
        self.process = QProcess(self)
        self.queue = []            # logs waiting to be extracted
        self.queue_total = 0
        self.current = None        # log being extracted
        self.stderr = ""
        self.logs = []             # logs of the current source
        self.keys = {}             # (path, size, mtime) -> log key
        self.errors = {}           # log path -> last error
        self.state = self.load_state()

        header = self.log_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        header.setStretchLastSection(True)
        self.progress_bar.setVisible(False)

        self.load_settings()
        self.source_combo.currentIndexChanged.connect(self.refresh_logs)
        self.browse_button.clicked.connect(self.select_folder)
        self.delete_button.clicked.connect(self.delete_logs)
        self.help_button.clicked.connect(self.show_help)
        self.output_button.clicked.connect(self.select_output)
        self.reset_output_button.clicked.connect(self.reset_output)
        self.output_edit.editingFinished.connect(self.save_settings)
        self.output_edit.textChanged.connect(self.update_preview)
        for check in (self.tlm_check, self.upload_check, self.delete_after_check):
            check.toggled.connect(self.save_settings)
        self.log_table.itemSelectionChanged.connect(self.update_details)
        self.log_table.itemChanged.connect(self.update_buttons)
        self.plotter_button.clicked.connect(self.open_plotter)
        self.folder_button.clicked.connect(self.open_folder)
        self.extract_new_button.clicked.connect(self.extract_new)
        self.extract_selected_button.clicked.connect(self.extract_selected)
        self.cancel_button.clicked.connect(self.cancel)
        self.process.finished.connect(self.handle_finished)
        self.process.readyReadStandardError.connect(self.read_stderr)
        self.process.readyReadStandardOutput.connect(self.process.readAllStandardOutput)

        self.scan_timer = QtCore.QTimer(self)
        self.scan_timer.timeout.connect(self.scan_sources)
        self.scan_timer.start(SCAN_INTERVAL_MS)
        self.scan_sources()

    def set_conf(self, conf):
        self.conf = conf

    # --- settings and state

    def load_settings(self):
        settings = utils.get_settings()
        default = settings.value("default_log_path", DEFAULT_OUTPUT, str)
        self.output_edit.setText(settings.value("sd_log/output_path", default, str))
        self.tlm_check.setChecked(settings.value("sd_log/keep_tlm", False, bool))
        self.upload_check.setChecked(settings.value("sd_log/upload", True, bool))
        self.delete_after_check.setChecked(settings.value("sd_log/delete_after", False, bool))
        self.keep_last = settings.value("sd_log/keep_last", 3, int)
        self.folder = settings.value("sd_log/folder", "", str)

    def save_settings(self):
        settings = utils.get_settings()
        settings.setValue("sd_log/output_path", self.output_edit.text())
        settings.setValue("sd_log/keep_tlm", self.tlm_check.isChecked())
        settings.setValue("sd_log/upload", self.upload_check.isChecked())
        settings.setValue("sd_log/delete_after", self.delete_after_check.isChecked())
        settings.setValue("sd_log/keep_last", self.keep_last)
        settings.setValue("sd_log/folder", self.folder)
        settings.sync()

    @staticmethod
    def load_state():
        try:
            with open(STATE_FILE) as f:
                state = json.load(f)
            return state if isinstance(state.get("logs"), dict) else {"logs": {}}
        except (OSError, ValueError, AttributeError):
            return {"logs": {}}

    def save_state(self):
        os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.state, f, indent=1)
        os.replace(tmp, STATE_FILE)

    def key_of(self, path):
        """Log key, cached on path, size and modification time"""
        try:
            st = os.stat(path)
        except OSError:
            return None
        cache = (path, st.st_size, st.st_mtime)
        if cache not in self.keys:
            try:
                self.keys[cache] = log_key(path)
            except OSError:
                return None
        return self.keys[cache]

    def record_of(self, path):
        key = self.key_of(path)
        return self.state["logs"].get(key) if key else None

    # --- sources and log table

    def scan_sources(self):
        """Update the detected SD cards in the source list, and the logs of the current source"""
        current = self.source_combo.currentData()
        sources = [(f"SD card: {card}", card) for card in detect_cards()]
        if self.folder:
            sources.append((f"Folder: {self.folder}", self.folder))
        if sources != [(self.source_combo.itemText(i), self.source_combo.itemData(i))
                       for i in range(self.source_combo.count())]:
            self.source_combo.blockSignals(True)
            self.source_combo.clear()
            for text, path in sources:
                self.source_combo.addItem(text, path)
            # keep the selected source, else take a new SD card
            index = self.source_combo.findData(current)
            self.source_combo.setCurrentIndex(index if index >= 0 else 0)
            self.source_combo.blockSignals(False)
            self.refresh_logs()
        elif self.process.state() == QProcess.NotRunning:
            source = self.source_combo.currentData()
            if source and find_logs(source) != self.logs:
                self.refresh_logs()

    def refresh_logs(self):
        source = self.source_combo.currentData()
        self.logs = find_logs(source) if source else []
        checked = {self.log_table.item(r, COL_FILE).data(Qt.UserRole)
                   for r in range(self.log_table.rowCount())
                   if self.log_table.item(r, COL_FILE).checkState() == Qt.Checked}
        known = {self.log_table.item(r, COL_FILE).data(Qt.UserRole) for r in range(self.log_table.rowCount())}
        selected = self.selected_log()
        self.log_table.blockSignals(True)
        self.log_table.setRowCount(len(self.logs))
        new = 0
        for row, path in enumerate(self.logs):
            record = self.record_of(path)
            new += record is None
            item = QTableWidgetItem(os.path.basename(path))
            item.setData(Qt.UserRole, path)
            item.setToolTip(path)
            item.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsUserCheckable)
            # new logs are ticked, a choice made before is kept
            tick = path in checked if path in known else record is None
            item.setCheckState(Qt.Checked if tick else Qt.Unchecked)
            self.log_table.setItem(row, COL_FILE, item)
            try:
                size = human_size(os.path.getsize(path))
            except OSError:
                size = "-"
            self.log_table.setItem(row, COL_SIZE, QTableWidgetItem(size))
            self.log_table.setItem(row, COL_DATE, QTableWidgetItem(record.get("date", "-") if record else "-"))
            self.log_table.setItem(row, COL_AIRCRAFT, QTableWidgetItem(record.get("aircraft", "-") if record else "-"))
            self.set_status(row, path, record)
            if path == selected:
                self.log_table.selectRow(row)
        self.log_table.blockSignals(False)
        if self.logs:
            self.count_label.setText(f"{len(self.logs)} log{'s' if len(self.logs) != 1 else ''}, {new} new")
        else:
            self.count_label.setText("No logs" if self.source_combo.count() else "No SD card detected")
        self.update_details()
        self.update_buttons()

    def set_status(self, row, path, record=None):
        if path == self.current:
            text, color = "Extracting…", None
        elif path in self.queue:
            text, color = "Waiting", None
        elif path in self.errors:
            text, color = f"Error: {self.errors[path]}", QtGui.QColor("#b42318")
        elif record:
            when = record.get("extracted_at", "")[:10]
            text, color = f"Extracted on {when}", None
            if record.get("warning"):
                text += f" ({record['warning']})"
                color = QtGui.QColor("#b45309")
        else:
            text, color = "New", None
        item = QTableWidgetItem(text)
        item.setToolTip(text)
        if color:
            item.setForeground(color)
        self.log_table.setItem(row, COL_STATUS, item)

    def row_of(self, path):
        for row in range(self.log_table.rowCount()):
            if self.log_table.item(row, COL_FILE).data(Qt.UserRole) == path:
                return row
        return None

    def update_row(self, path):
        row = self.row_of(path)
        if row is not None:
            self.log_table.blockSignals(True)
            record = self.record_of(path)
            self.log_table.setItem(row, COL_DATE, QTableWidgetItem(record.get("date", "-") if record else "-"))
            self.log_table.setItem(row, COL_AIRCRAFT, QTableWidgetItem(record.get("aircraft", "-") if record else "-"))
            self.set_status(row, path, record)
            self.log_table.blockSignals(False)

    def selected_log(self):
        rows = self.log_table.selectionModel().selectedRows()
        if rows and self.log_table.item(rows[0].row(), COL_FILE):
            return self.log_table.item(rows[0].row(), COL_FILE).data(Qt.UserRole)
        return None

    def ticked_logs(self):
        return [self.log_table.item(r, COL_FILE).data(Qt.UserRole) for r in range(self.log_table.rowCount())
                if self.log_table.item(r, COL_FILE).checkState() == Qt.Checked]

    def new_logs(self):
        return [p for p in self.logs if self.record_of(p) is None]

    def update_buttons(self, *_):
        running = self.process.state() != QProcess.NotRunning or bool(self.queue)
        new = len(self.new_logs()) if not running else 0
        self.extract_new_button.setText(f"Extract new ({new})" if new else "Extract new")
        self.extract_new_button.setEnabled(not running and new > 0)
        self.extract_selected_button.setEnabled(not running and bool(self.ticked_logs()))
        self.cancel_button.setEnabled(running)
        self.delete_button.setEnabled(not running and bool(self.logs))
        self.browse_button.setEnabled(not running)
        self.source_combo.setEnabled(not running)

    def update_details(self):
        record = self.record_of(self.selected_log()) if self.selected_log() else None
        name = os.path.basename(self.selected_log()) if self.selected_log() else ""
        self.details_box.setTitle(f"Details: {name}" if name else "Details")
        if record:
            self.frames_label.setText(f"{record.get('frames', 0):,} ({record.get('skipped', 0)} skipped, "
                                      f"{record.get('cksum', 0)} checksum errors)")
            duration = record.get("duration")
            self.duration_label.setText(f"{int(duration // 60)} min {int(duration % 60)} s"
                                        if duration is not None else "-")
            self.definitions_label.setText(record.get("definitions", "-"))
            files = record.get("files", [])
            self.files_label.setText(", ".join(os.path.basename(f) for f in files) or "-")
            self.files_label.setToolTip("\n".join(files))
        else:
            for label in (self.frames_label, self.duration_label, self.definitions_label, self.files_label):
                label.setText("-")
        data = self.data_file(record)
        self.plotter_button.setEnabled(data is not None and os.path.exists(self.plotter_path()))
        self.folder_button.setEnabled(data is not None)
        self.update_preview()

    def update_preview(self, *_):
        """Show where the selected (or an example) log ends up"""
        record = self.record_of(self.selected_log()) if self.selected_log() else None
        if record:
            folder = self.output_folder(record["basename"], record.get("ac_id", 0), record.get("aircraft", ""))
            self.preview_label.setText(f"e.g. {folder}/{record['basename']}.data")
        else:
            example = self.output_folder("25_06_11__11_06_45_SD", 42, "MyAircraft")
            self.preview_label.setText(f"e.g. {example} (filled in from each log)")

    # --- user actions

    def select_folder(self):
        start = self.folder or os.path.expandvars("$HOME")
        path = QFileDialog().getExistingDirectory(self, "Select the folder with the SD logs", start,
                                                  QFileDialog.ShowDirsOnly)
        if path:
            self.folder = path
            self.save_settings()
            self.scan_sources()
            self.source_combo.setCurrentIndex(self.source_combo.findData(path))

    def select_output(self):
        start_path = os.path.expandvars(self.output_edit.text())
        path = QFileDialog().getExistingDirectory(self, "Select output folder", start_path, QFileDialog.ShowDirsOnly)
        if not path:
            return
        for env in ENVS:
            res = os.getenv(env)
            if res and res in path:
                path_sub = path.replace(res, f"${env}")
                if os.path.expandvars(path_sub) == path:
                    # check that the substitution is valid
                    path = path_sub
        self.output_edit.setText(path)
        self.save_settings()

    def reset_output(self):
        settings = utils.get_settings()
        self.output_edit.setText(settings.value("default_log_path", DEFAULT_OUTPUT, str))
        self.save_settings()

    def delete_logs(self):
        extracted = {p for p in self.logs if self.record_of(p)}
        dialog = DeleteLogsDialog(self.logs, extracted, self.keep_last, self)
        if dialog.exec_() != QDialog.Accepted:
            return
        self.keep_last = dialog.keep_spin.value()
        self.save_settings()
        failed = []
        for path in dialog.selected_logs():
            try:
                os.remove(path)
            except OSError as e:
                failed.append(f"{os.path.basename(path)}: {e.strerror}")
        os.sync()
        if failed:
            QMessageBox.warning(self, "Delete logs", "Not deleted:\n" + "\n".join(failed))
        self.status_label.setText(f"Deleted {len(dialog.selected_logs()) - len(failed)} logs")
        self.refresh_logs()

    def open_plotter(self):
        data = self.data_file(self.record_of(self.selected_log()))
        if data:
            QProcess.startDetached(self.plotter_path(), [data])

    def open_folder(self):
        data = self.data_file(self.record_of(self.selected_log()))
        if data:
            QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(os.path.dirname(data)))

    @staticmethod
    def plotter_path():
        return os.path.expandvars("$PAPARAZZI_SRC/sw/logalizer/logplotter")

    @staticmethod
    def data_file(record):
        if record:
            for f in record.get("files", []):
                if f.endswith(".data") and os.path.exists(f):
                    return f
        return None

    # --- extraction

    def extract_new(self):
        self.start_queue(self.new_logs())

    def extract_selected(self):
        self.start_queue(self.ticked_logs())

    def start_queue(self, logs):
        if not logs or self.process.state() != QProcess.NotRunning:
            return
        self.save_settings()
        self.queue = list(logs)
        self.queue_total = len(logs)
        for path in logs:
            self.errors.pop(path, None)
        self.progress_bar.setRange(0, self.queue_total)
        self.progress_bar.setValue(0)
        self.progress_bar.setVisible(True)
        self.extract_next()

    def cancel(self):
        """Stop after the log being extracted now"""
        waiting = self.queue
        self.queue = []
        for path in waiting:
            self.update_row(path)
        self.status_label.setText("Cancelled")
        self.update_buttons()

    def extract_next(self):
        if not self.queue:
            self.current = None
            self.progress_bar.setVisible(False)
            self.update_buttons()
            self.refresh_logs()
            return
        self.current = self.queue.pop(0)
        done = self.queue_total - len(self.queue)
        self.status_label.setText(f"Extracting {done} of {self.queue_total}: {os.path.basename(self.current)}")
        self.progress_bar.setValue(done - 1)
        for path in [self.current] + self.queue:
            self.update_row(path)
        self.update_buttons()
        self.stderr = ""
        program = os.path.expandvars("$PAPARAZZI_SRC/sw/logalizer/sd2log")
        args = [self.current, TMP_DIR]
        if self.tlm_check.isChecked():
            args.append("--tlm")
        self.process.start(program, args)

    def read_stderr(self):
        self.stderr += self.process.readAllStandardError().data().decode(errors="replace")

    def handle_finished(self, exit_code: int, exit_status: QProcess.ExitStatus):
        path = self.current
        self.read_stderr()
        self.errors.pop(path, None)  # result of a previous extraction
        try:
            if exit_status != QProcess.ExitStatus.NormalExit or exit_code != 0:
                lines = [line for line in self.stderr.splitlines() if line.strip()]
                raise RuntimeError(lines[-1] if lines else f"sd2log exited with code {exit_code}")
            record = self.store_result(path)
        except (RuntimeError, OSError, ValueError) as e:
            self.errors[path] = str(e)
        else:
            if self.upload_check.isChecked():
                self.extracted.emit(self.data_file(record))
            if self.delete_after_check.isChecked() and record.get("frames", 0) > 0:
                if record.get("conf") != "md5-conf" and not record.get("tlm"):
                    # decoded without the build definitions and no raw copy: keep the original
                    self.errors[path] = "extracted, but not deleted: build configuration not found " \
                                        "and no .tlm copy kept"
                else:
                    try:
                        os.remove(path)
                        os.sync()
                    except OSError as e:
                        self.errors[path] = f"extracted, but not deleted: {e.strerror}"
            self.status_label.setText(f"Extracted {os.path.basename(path)} in {os.path.dirname(self.data_file(record))}")
            row = self.row_of(path)
            if row is not None:
                self.log_table.item(row, COL_FILE).setCheckState(Qt.Unchecked)
        self.update_row(path)
        self.extract_next()

    def store_result(self, path):
        """Move the files produced by sd2log to the output folder and remember the log"""
        result = None
        definitions = "Build configuration found (md5 of the ALIVE message)"
        for line in self.stderr.splitlines():
            m = re.match(r'Generated (\S+)\.\[([a-z,]+)\] \| time=([\d.]+) \| conf=(\S+) \| '
                         r'frames=(\d+) skipped=(\d+) cksum=(\d+)', line)
            if m:
                result = m
            elif line.startswith("[msgs] build configuration not found") or \
                    line.startswith("[msgs] no message definitions"):
                definitions = "Build configuration not found, decoded with var/messages.xml"
        if result is None:
            raise RuntimeError("no output from sd2log")
        basename, extensions = result.group(1), result.group(2).split(",")
        data_tmp = os.path.join(TMP_DIR, basename + ".data")
        ac_id, duration = self.read_data(data_tmp)
        ac = self.conf[ac_id] if self.conf is not None and ac_id is not None else None
        aircraft = ac.name if ac else (f"ac_{ac_id}" if ac_id is not None else "unknown")
        out = self.output_folder(basename, ac_id if ac_id is not None else 0, aircraft)
        os.makedirs(out, exist_ok=True)
        files = []
        for ext in extensions:
            src = os.path.join(TMP_DIR, f"{basename}.{ext}")
            if os.path.exists(src):
                dst = os.path.join(out, f"{basename}.{ext}")
                shutil.move(src, dst)
                files.append(dst)
        gps_time = float(result.group(3))
        no_gps = "_no_GPS" in basename
        record = {
            "file": os.path.basename(path),
            "extracted_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "basename": basename,
            "date": "unknown (no GPS)" if no_gps else
                    time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime(gps_time)),
            "ac_id": ac_id,
            "aircraft": f"{aircraft} ({ac_id})" if ac else aircraft,
            "frames": int(result.group(5)),
            "skipped": int(result.group(6)),
            "cksum": int(result.group(7)),
            "conf": result.group(4),
            "definitions": definitions,
            "duration": duration,
            "files": files,
            "tlm": any(f.endswith(".tlm") for f in files),
            "warning": "no messages found" if int(result.group(5)) == 0 else ("no GPS date" if no_gps else ""),
        }
        self.state["logs"][self.key_of(path)] = record
        self.save_state()
        return record

    @staticmethod
    def read_data(data_path):
        """Aircraft id (first line) and duration (first and last time) of a .data file"""
        with open(data_path, "rb") as f:
            first = f.readline().split()
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 4096))
            lines = f.read().splitlines()
        last = lines[-1].split() if lines else []
        ac_id = int(first[1]) if len(first) > 1 else None
        duration = float(last[0]) - float(first[0]) if first and last else None
        return ac_id, duration

    def output_folder(self, basename, ac_id, aircraft):
        """Output folder with the tags filled in for a log"""
        out = os.path.expandvars(self.output_edit.text())
        m = re.match(r'(\d{2})_(\d{2})_(\d{2})__(\d{2})_(\d{2})_(\d{2})', basename)
        values = dict(zip(DATE_TAGS, m.groups())) if m else {tag: "00" for tag in DATE_TAGS}
        values["AIRCRAFT"] = re.sub(r' \(\d+\)$', "", aircraft)
        values["AC_ID"] = str(ac_id)
        for tag in DATE_TAGS + OTHER_TAGS:
            out = out.replace(f"${tag}", values[tag])
        return out

    def show_help(self):
        QMessageBox.information(self, "SD log extractor help",
                                "<h1>SD log extractor</h1>"
                                "Extract the logs of the flight recorder SD card with sd2log.<br/>"
                                "SD cards are detected when they are mounted; Browse reads the logs of a folder. "
                                "New logs are ticked; the extracted ones are remembered in "
                                "var/sd_log_state.json.<br/><br/>"
                                "You can use substitution variables in the output folder:<br/>"
                                "<b>$AIRCRAFT</b>: aircraft name<br/>"
                                "<b>$AC_ID</b>: aircraft id<br/>"
                                "<b>$YY</b>: year<br/>"
                                "<b>$MM</b>: month<br/>"
                                "<b>$DD</b>: day<br/>"
                                "<b>$hh</b>: hour<br/>"
                                "<b>$mm</b>: minutes<br/>"
                                "<b>$ss</b>: seconds<br/>")
