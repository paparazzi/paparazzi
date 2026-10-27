# Copyright (C) 2008-2022 The Paparazzi Team
# released under GNU GPLv2 or later. See COPYING file.
"""Upload new .data/.log pairs from the logs folder to a log database server.

The server is the Paparazzi logbook from https://github.com/tudelft/paparazzi_log_parsing
(webui); the same protocol is used by its standalone upload_logs.py script.
It must accept an API key as "Authorization: Bearer <key>" and provide GET /api/me,
POST /api/logs/check, POST /api/upload-reservations and POST /api/upload.
Uploads only run while no session is running in the Paparazzi Center. Logs recorded
during a session of this Paparazzi Center are uploaded when the session stops, logs
extracted from an SD card right after extraction; other logs (e.g. from a server
started elsewhere) only after they were unchanged for QUIET_TIME.
Uploaded, skipped and rejected pairs are remembered in
$PAPARAZZI_HOME/var/log_upload_state.json so they are not hashed or sent again.
"""
import hashlib
import http.client
import json
import os
import threading
import time
import uuid
from urllib.parse import urlsplit
from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget, QTableWidgetItem, QHeaderView
import utils
from generated.ui_log_upload import Ui_LogUpload

LOGS_DIR = os.path.join(utils.PAPARAZZI_HOME, "var", "logs")
STATE_FILE = os.path.join(utils.PAPARAZZI_HOME, "var", "log_upload_state.json")
CHUNK = 1 << 20
QUIET_TIME = 60 * 60            # seconds without changes before a log from outside a session is complete
RETRY_INTERVAL = 60 * 60        # seconds between automatic runs, to retry when the server was not reachable
SESSION_STOP_DELAY = 5          # seconds after a session stopped before uploading its logs
HISTORY_SIZE = 200
CAPACITY_WAIT = 60 * 60         # seconds to wait for server capacity before retrying in a later run
RETRY_STATUS = (401, 403, 408, 409, 429)    # HTTP errors on upload which are not about the log itself


class HTTPError(RuntimeError):
    def __init__(self, status, message, retry_after=None):
        RuntimeError.__init__(self, f"HTTP {status}: {message}")
        self.status = status
        self.retry_after = retry_after


class Paused(Exception):
    """A session started while waiting for the server."""


class Client:
    def __init__(self, server, key, timeout=600):
        u = urlsplit(server if "//" in server else "//" + server, scheme="https")
        self.https = u.scheme == "https"
        self.host, self.port = u.hostname, u.port or (443 if self.https else 80)
        self.prefix = u.path.rstrip("/")
        self.key, self.timeout = key, timeout

    def request(self, method, path, body=None, headers=None, timeout=None):
        conn_class = http.client.HTTPSConnection if self.https else http.client.HTTPConnection
        c = conn_class(self.host, self.port, timeout=timeout or self.timeout)
        try:
            c.request(method, self.prefix + path, body=body,
                      headers={"Authorization": f"Bearer {self.key}", **(headers or {})})
            r = c.getresponse()
            raw = r.read()
            retry_after = r.getheader("Retry-After")
        finally:
            c.close()
        try:
            data = json.loads(raw)
        except ValueError:
            data = {"error": raw[:200].decode("utf-8", "replace")}
        if r.status >= 400 or not isinstance(data, dict):
            raise HTTPError(r.status, data.get('error', data) if isinstance(data, dict) else data, retry_after)
        return data

    def check(self, hashes):
        body = json.dumps({"sha256": hashes}).encode()
        return self.request("POST", "/api/logs/check", body, {"Content-Type": "application/json"})["known"]

    def reserve(self, size, cancel=None, report=print):
        """Reserve room in the server's shared upload budget for size bytes, like upload_logs.py --bulk,
        so a large backlog does not overload the server. Waits while the server is busy."""
        token = uuid.uuid4().hex
        body = json.dumps({"token": token, "size_bytes": size}).encode()
        started = time.monotonic()
        while True:
            try:
                self.request("POST", "/api/upload-reservations", body, {"Content-Type": "application/json"})
                return token
            except HTTPError as e:
                if e.status != 429 or time.monotonic() - started > CAPACITY_WAIT:
                    raise
                try:
                    delay = min(max(float(e.retry_after), 5), 60)
                except (TypeError, ValueError):
                    delay = 5
                report("Log upload: waiting for server capacity...")
                if cancel is not None and cancel.wait(delay):
                    raise Paused()
                if cancel is None:
                    time.sleep(delay)

    def upload(self, files, reservation=None):
        """files: list of (name, path). The files are streamed, not loaded in memory."""
        boundary = uuid.uuid4().hex
        parts = [f'--{boundary}\r\nContent-Disposition: form-data; name="names"\r\n\r\n{name}\r\n'.encode()
                 for name, _ in files]
        for name, path in files:
            parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="files"; '
                         f'filename="{os.path.basename(path)}"\r\n'
                         'Content-Type: application/octet-stream\r\n\r\n'.encode())
            parts.append(path)
            parts.append(b"\r\n")
        parts.append(f"--{boundary}--\r\n".encode())
        length = sum(len(p) if isinstance(p, bytes) else os.path.getsize(p) for p in parts)

        def body():
            for p in parts:
                if isinstance(p, bytes):
                    yield p
                else:
                    with open(p, "rb") as f:
                        while block := f.read(CHUNK):
                            yield block

        headers = {"Content-Type": f"multipart/form-data; boundary={boundary}", "Content-Length": str(length)}
        if reservation:
            headers["X-Bulk-Reservation"] = reservation
        return self.request("POST", "/api/upload", body(), headers)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while block := f.read(CHUNK):
            h.update(block)
    return h.hexdigest()


def find_pairs(root):
    """Yield (relative name, data path, log path, data stat, log stat) for every .data/.log pair below root."""
    for folder, _, files in os.walk(root):
        names = set(files)
        for f in sorted(files):
            stem, ext = os.path.splitext(f)
            if ext != ".data" or stem + ".log" not in names:
                continue
            data, log = os.path.join(folder, f), os.path.join(folder, stem + ".log")
            try:
                yield os.path.relpath(data, root), data, log, os.stat(data), os.stat(log)
            except OSError:
                continue


def is_complete(data, data_stat, log_stat, sessions, finished, now):
    """A pair is complete when it was recorded during a finished session of this Paparazzi Center
    (the server writes the .log at start and the .data until it stops), when it was extracted from
    an SD card (data path in finished), or when it was not changed for QUIET_TIME."""
    if os.path.realpath(data) in finished:
        return True
    last_change = max(data_stat.st_mtime, log_stat.st_mtime)
    if now - last_change >= QUIET_TIME:
        return True
    return any(start <= log_stat.st_mtime and last_change <= stop for start, stop in sessions)


def load_state():
    try:
        with open(STATE_FILE) as f:
            state = json.load(f)
    except (OSError, ValueError):
        state = None
    if not isinstance(state, dict) or not isinstance(state.get("servers"), dict):
        state = {"servers": {}, "history": []}
    state.setdefault("history", [])
    return state


def save_state(state):
    tmp = STATE_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state, f)
    os.replace(tmp, STATE_FILE)


def upload_new_logs(server, key, sessions=(), finished=(), cancel=None, report=print, record=None,
                    folder=LOGS_DIR):
    """Upload the complete pairs below folder which were not handled for this server yet.
    sessions: list of (start, stop) times of finished sessions; finished: real paths of .data files
    known to be complete; cancel: threading.Event to stop early.
    report(str) gets a short status, record(entry) every history entry [time, log, result, details]."""
    state = load_state()
    handled = state["servers"].setdefault(server.rstrip("/"), {})
    now = time.time()
    todo, waiting = [], 0
    for name, data, log, ds, ls in find_pairs(folder) if os.path.isdir(folder) else ():
        sig = [[ds.st_size, ds.st_mtime_ns], [ls.st_size, ls.st_mtime_ns]]
        if handled.get(name, {}).get("sig") == sig:
            continue
        if not is_complete(data, ds, ls, sessions, finished, now):
            waiting += 1
        else:
            todo.append((name, data, log, sig))
    waiting_txt = f", {waiting} not finished yet" if waiting else ""

    def add(name, sig, result, details=""):
        if sig is not None:
            handled[name] = {"sig": sig, "result": result}
        entry = [time.time(), name, result, details]
        state["history"] = (state["history"] + [entry])[-HISTORY_SIZE:]
        save_state(state)
        if record:
            record(entry)

    if not todo:
        report(f"Log upload: up to date{waiting_txt}")
        return
    client = Client(server, key)
    try:
        client.request("GET", "/api/me", timeout=10)
    except (OSError, http.client.HTTPException, RuntimeError) as e:
        refused = isinstance(e, HTTPError) and e.status in (401, 403)
        result = "Key refused" if refused else "Not reachable"
        report(f"Log upload: {len(todo)} waiting, " + ("API key refused" if refused else "server not reachable"))
        if not state["history"] or state["history"][-1][2] != result:
            add("", None, result, f"{e} (retried later)")
        return
    counts = {"uploaded": 0, "failed": 0}
    for i, (name, data, log, sig) in enumerate(todo):
        if cancel is not None and cancel.is_set():
            report(f"Log upload: paused, {len(todo) - i} waiting for the session to stop")
            return
        if sig[0][0] == 0:
            add(name, sig, "Skipped", "empty .data file (no telemetry received)")
            continue
        report(f"Log upload: uploading {i + 1}/{len(todo)}...")
        try:
            digest = sha256_file(data)
            known = client.check([digest]).get(digest)
            if known and known.get("has_log") and known.get("mine", True):
                add(name, sig, "On server", "already uploaded")
                continue
            try:
                reservation = client.reserve(sig[0][0] + sig[1][0], cancel, report)
            except Paused:
                report(f"Log upload: paused, {len(todo) - i} waiting for the session to stop")
                return
            except HTTPError as e:
                report(f"Log upload: stopped, {e}")
                add(name, None, "Failed", f"no upload slot: {e} (retried later)")
                return
            report(f"Log upload: uploading {i + 1}/{len(todo)}...")
            res = client.upload([(name, data), (os.path.splitext(name)[0] + ".log", log)], reservation)
            receipt = (res.get("results") or [{}])[0]
            if receipt.get("status") == "accepted":
                counts["uploaded"] += 1
                add(name, sig, "Uploaded", f"{(sig[0][0] + sig[1][0]) / 1e6:.1f} MB")
            else:
                counts["failed"] += 1
                add(name, sig, "Rejected", receipt.get("reason") or f"upload not accepted: {receipt}")
        except HTTPError as e:
            if e.status in RETRY_STATUS or e.status >= 500:
                report(f"Log upload: stopped, {e}")
                add(name, None, "Failed", f"{e} (retried later)")
                return
            counts["failed"] += 1
            add(name, sig, "Rejected", str(e))
        except (OSError, http.client.HTTPException, RuntimeError) as e:
            report(f"Log upload: stopped, {e}")
            add(name, None, "Failed", f"{e} (retried later)")
            return
    failed_txt = f", {counts['failed']} rejected" if counts["failed"] else ""
    report(f"Log upload: {counts['uploaded']} uploaded{failed_txt}{waiting_txt}")


class LogUploadWidget(QWidget, Ui_LogUpload):
    """Utilities tab which uploads new logs in a background thread when a server URL and API key
    are set, and shows what happened."""
    status = QtCore.pyqtSignal(str)
    recorded = QtCore.pyqtSignal(list)
    done = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        header = self.history_table.horizontalHeader()
        for col in range(3):
            header.setSectionResizeMode(col, QHeaderView.ResizeToContents)
        self.thread = None
        self.rerun = False
        self.cancel = threading.Event()
        self.session = None
        self.session_start = None
        self.sessions = []
        self.finished = set()
        self.status.connect(self.set_summary)
        self.recorded.connect(self.add_entry)
        self.done.connect(self.handle_done)
        self.upload_button.clicked.connect(self.start)
        self.save_button.clicked.connect(self.save_settings)
        self.server_edit.returnPressed.connect(self.save_settings)
        self.key_edit.returnPressed.connect(self.save_settings)
        server, key = self.settings()
        self.server_edit.setText(server)
        self.key_edit.setText(key)
        for entry in load_state()["history"]:
            self.add_entry(entry)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.start)
        self.timer.start(RETRY_INTERVAL * 1000)
        QtCore.QTimer.singleShot(10000, self.start)
        self.set_summary("")

    def set_session(self, session):
        """Follow the programs of the Operation tab session (SessionWidget)."""
        self.session = session
        session.program_spawned.connect(self.handle_session_started)
        session.programs_all_stopped.connect(self.handle_session_stopped)

    def session_running(self):
        return self.session is not None and self.session.any_program_running()

    def handle_session_started(self):
        if self.session_start is None:
            self.session_start = time.time() - 1
        self.cancel.set()

    def handle_session_stopped(self):
        if self.session_start is not None and not self.session_running():
            self.sessions.append((self.session_start, time.time()))
            self.session_start = None
            QtCore.QTimer.singleShot(SESSION_STOP_DELAY * 1000, self.start)

    def add_finished_log(self, data_path):
        """A log which is complete, e.g. just extracted from an SD card: upload it now."""
        self.finished.add(os.path.realpath(data_path))
        self.start()

    def settings(self):
        settings = utils.get_settings()
        return (settings.value("log_upload/server", "", str).strip(),
                settings.value("log_upload/api_key", "", str).strip())

    def save_settings(self):
        settings = utils.get_settings()
        settings.setValue("log_upload/server", self.server_edit.text().strip())
        settings.setValue("log_upload/api_key", self.key_edit.text().strip())
        settings.sync()
        self.set_summary("")
        self.start()

    def start(self):
        server, key = self.settings()
        if not server or not key:
            self.set_summary("")
            return
        if self.thread is not None and self.thread.is_alive():
            self.rerun = True   # new logs may have finished meanwhile
            return
        if self.session_running():
            self.status.emit("Log upload: waiting for the session to stop")
            return
        self.cancel.clear()
        self.rerun = False
        kwargs = {"sessions": list(self.sessions), "finished": set(self.finished), "cancel": self.cancel,
                  "report": self.status.emit, "record": self.recorded.emit}

        def run():
            try:
                upload_new_logs(server, key, **kwargs)
            finally:
                self.done.emit()

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def handle_done(self):
        if self.rerun:
            self.start()

    def set_summary(self, text):
        server, key = self.settings()
        if not server or not key:
            text = "Enter the log server URL and API key and press Save to upload logs automatically."
        self.summary_label.setText(text)
        self.summary_label.setToolTip(
            f"Server: {server or '-'}\nLogs folder: {LOGS_DIR}\n"
            f"Logs are uploaded when a session stops, after an SD log extraction, at start-up and every "
            f"{RETRY_INTERVAL // 60} minutes.\n"
            f"Logs not recorded by this Paparazzi Center are uploaded after {QUIET_TIME // 60} minutes without changes.")
        self.upload_button.setEnabled(bool(server and key))

    def add_entry(self, entry):
        t, name, result, details = entry
        self.history_table.insertRow(0)
        values = (time.strftime("%Y-%m-%d %H:%M", time.localtime(t)), name, result, details)
        for col, value in enumerate(values):
            item = QTableWidgetItem(value)
            item.setToolTip(value)
            self.history_table.setItem(0, col, item)
        while self.history_table.rowCount() > HISTORY_SIZE:
            self.history_table.removeRow(self.history_table.rowCount() - 1)
