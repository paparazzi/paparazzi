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
Uploaded, skipped, rejected and ignored ("Don't upload") pairs are remembered in
$PAPARAZZI_HOME/var/log_upload_state.json so they are not hashed or sent again.
The tab lists the pending logs and why they wait, and uploading can be paused.
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


def resolve_key(value):
    """The API key setting is either the key itself or the path of a file holding it (starting with / or ~),
    so the key does not have to be stored in the settings. The file is read on every run."""
    if not value.startswith(("/", "~")):
        return value
    with open(os.path.expanduser(value)) as f:
        return f.read().strip()


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


STATE_LOCK = threading.Lock()   # the state file is changed by the upload thread and the tab


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


def update_state(change):
    """Apply change(state) to the state file and return its result."""
    with STATE_LOCK:
        state = load_state()
        result = change(state)
        save_state(state)
        return result


def server_key(server):
    return server.rstrip("/")


def signature(data_stat, log_stat):
    return [[data_stat.st_size, data_stat.st_mtime_ns], [log_stat.st_size, log_stat.st_mtime_ns]]


def scan_pending(server, sessions=(), finished=(), folder=LOGS_DIR):
    """The pairs below folder which were not handled for this server yet, as dicts with
    name, data, log, sig, size, mtime and complete (finished recording)."""
    with STATE_LOCK:
        handled = load_state()["servers"].get(server_key(server), {})
    now = time.time()
    pending = []
    for name, data, log, ds, ls in find_pairs(folder) if os.path.isdir(folder) else ():
        sig = signature(ds, ls)
        if handled.get(name, {}).get("sig") == sig:
            continue
        pending.append({"name": name, "data": data, "log": log, "sig": sig, "size": ds.st_size + ls.st_size,
                        "mtime": max(ds.st_mtime, ls.st_mtime),
                        "complete": is_complete(data, ds, ls, sessions, finished, now)})
    return pending


def new_entry(name, result, details=""):
    return [time.time(), name, result, details]


def record_result(server, name, sig, result, details=""):
    """Remember what happened to a log (sig None: try again later) and add it to the history."""
    entry = new_entry(name, result, details)

    def change(state):
        if sig is not None:
            state["servers"].setdefault(server_key(server), {})[name] = {"sig": sig, "result": result}
        state["history"] = (state["history"] + [entry])[-HISTORY_SIZE:]
    update_state(change)
    return entry


def ignore_logs(server, logs):
    """Never upload these pending logs (dicts from scan_pending) unless they change."""
    entries = [new_entry(log["name"], "Ignored", "marked Don't upload") for log in logs]

    def change(state):
        handled = state["servers"].setdefault(server_key(server), {})
        for log in logs:
            handled[log["name"]] = {"sig": log["sig"], "result": "Ignored"}
        state["history"] = (state["history"] + entries)[-HISTORY_SIZE:]
    update_state(change)
    return entries


def requeue_logs(server, names):
    """Put logs which were marked Don't upload back in the queue."""
    def change(state):
        handled = state["servers"].setdefault(server_key(server), {})
        names_done = [n for n in names if handled.get(n, {}).get("result") == "Ignored"]
        for n in names_done:
            del handled[n]
        entries = [new_entry(n, "Queued", "Upload anyway") for n in names_done]
        state["history"] = (state["history"] + entries)[-HISTORY_SIZE:]
        return entries
    return update_state(change)


def still_pending(server, name, sig):
    with STATE_LOCK:
        return load_state()["servers"].get(server_key(server), {}).get(name, {}).get("sig") != sig


def upload_new_logs(server, key, sessions=(), finished=(), cancel=None, report=print, record=None,
                    blocked=None, folder=LOGS_DIR):
    """Upload the complete pairs below folder which were not handled for this server yet.
    sessions: list of (start, stop) times of finished sessions; finished: real paths of .data files
    known to be complete; cancel: threading.Event to stop early.
    report(str) gets a short status, record(entry) every history entry [time, log, result, details],
    blocked(str) why the server cannot be used now ("" once it can)."""
    pending = scan_pending(server, sessions, finished, folder)
    todo = [p for p in pending if p["complete"]]
    waiting = len(pending) - len(todo)
    waiting_txt = f", {waiting} not finished yet" if waiting else ""

    def add(name, sig, result, details=""):
        entry = record_result(server, name, sig, result, details)
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
        problem = "API key refused" if refused else "server not reachable"
        report(f"Log upload: {len(todo)} waiting, {problem}")
        if blocked:
            blocked(problem)
        with STATE_LOCK:
            history = load_state()["history"]
        result = "Key refused" if refused else "Not reachable"
        if not history or history[-1][2] != result:
            add("", None, result, f"{e} (retried later)")
        return
    if blocked:
        blocked("")
    counts = {"uploaded": 0, "on_server": 0, "skipped": 0, "failed": 0}
    for i, p in enumerate(todo):
        name, data, log, sig = p["name"], p["data"], p["log"], p["sig"]
        if cancel is not None and cancel.is_set():
            report(f"Log upload: paused, {len(todo) - i} waiting")
            return
        if not still_pending(server, name, sig):
            continue    # marked Don't upload meanwhile
        if sig[0][0] == 0:
            counts["skipped"] += 1
            add(name, sig, "Skipped", "empty .data file (no telemetry received)")
            continue
        report(f"Log upload: verifying {i + 1}/{len(todo)}...")
        try:
            digest = sha256_file(data)
            known = client.check([digest]).get(digest)
            if known and known.get("has_log") and known.get("mine", True):
                counts["on_server"] += 1
                add(name, sig, "On server", "already uploaded")
                continue
            try:
                reservation = client.reserve(sig[0][0] + sig[1][0], cancel, report)
            except Paused:
                report(f"Log upload: paused, {len(todo) - i} waiting")
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
    others = ((counts["on_server"], "already on the server"), (counts["skipped"], "skipped (empty)"),
              (counts["failed"], "rejected"))
    others_txt = "".join(f", {n} {what}" for n, what in others if n)
    report(f"Log upload: {counts['uploaded']} uploaded{others_txt}{waiting_txt}")


def human_size(n):
    return f"{n / 1e6:.1f} MB" if n >= 1e5 else f"{n / 1e3:.0f} kB"


class LogUploadWidget(QWidget, Ui_LogUpload):
    """Utilities tab which uploads new logs in a background thread when a server URL and API key
    are set, and shows the pending logs and what happened."""
    status = QtCore.pyqtSignal(str)
    recorded = QtCore.pyqtSignal(list)
    blocked = QtCore.pyqtSignal(str)
    done = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        QWidget.__init__(self, parent=parent)
        self.setupUi(self)
        for table, stretch in ((self.history_table, 3), (self.pending_table, 3)):
            header = table.horizontalHeader()
            for col in range(stretch):
                header.setSectionResizeMode(col, QHeaderView.ResizeToContents)
        self.thread = None
        self.rerun = False
        self.cancel = threading.Event()
        self.session = None
        self.session_start = None
        self.sessions = []
        self.finished = set()
        self.pending = []
        self.problem = ""
        self.last_status = ""
        self.status.connect(self.handle_status)
        self.recorded.connect(self.handle_recorded)
        self.blocked.connect(self.handle_blocked)
        self.done.connect(self.handle_done)
        self.upload_button.clicked.connect(self.start)
        self.save_button.clicked.connect(self.save_settings)
        self.server_edit.returnPressed.connect(self.save_settings)
        self.key_edit.returnPressed.connect(self.save_settings)
        self.pause_button.toggled.connect(self.set_paused)
        self.pending_button.toggled.connect(self.show_view)
        self.history_button.toggled.connect(self.show_view)
        self.ignore_button.clicked.connect(self.ignore_selected)
        self.ignore_all_button.clicked.connect(self.ignore_all)
        self.requeue_button.clicked.connect(self.requeue_selected)
        self.pending_table.itemSelectionChanged.connect(self.update_buttons)
        self.history_table.itemSelectionChanged.connect(self.update_buttons)
        server, key = self.settings()
        self.server_edit.setText(server)
        self.key_edit.setText(key)
        self.pause_button.setChecked(utils.get_settings().value("log_upload/paused", False, bool))
        with STATE_LOCK:
            history = load_state()["history"]
        for entry in history:
            self.add_entry(entry)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.start)
        self.timer.start(RETRY_INTERVAL * 1000)
        QtCore.QTimer.singleShot(10000, self.start)
        self.show_view()
        self.refresh_pending()

    # --- sessions

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
        self.refresh_pending()

    def handle_session_stopped(self):
        if self.session_start is not None and not self.session_running():
            self.sessions.append((self.session_start, time.time()))
            self.session_start = None
            self.refresh_pending()
            QtCore.QTimer.singleShot(SESSION_STOP_DELAY * 1000, self.start)

    def add_finished_log(self, data_path):
        """A log which is complete, e.g. just extracted from an SD card: upload it now."""
        self.finished.add(os.path.realpath(data_path))
        self.refresh_pending()
        self.start()

    # --- settings

    def settings(self):
        settings = utils.get_settings()
        return (settings.value("log_upload/server", "", str).strip(),
                settings.value("log_upload/api_key", "", str).strip())

    def configured(self):
        server, key = self.settings()
        return bool(server and key)

    def save_settings(self):
        settings = utils.get_settings()
        settings.setValue("log_upload/server", self.server_edit.text().strip())
        settings.setValue("log_upload/api_key", self.key_edit.text().strip())
        settings.sync()
        self.problem = ""
        self.refresh_pending()
        self.start()

    def set_paused(self, paused):
        settings = utils.get_settings()
        settings.setValue("log_upload/paused", paused)
        settings.sync()
        self.pause_button.setText("Resume" if paused else "Pause")
        if paused:
            self.cancel.set()
            self.status.emit("Log upload: paused")
        else:
            self.start()
        self.refresh_pending()

    # --- uploading

    def start(self):
        server, key = self.settings()
        if not server or not key:
            self.update_summary()
            return
        if self.pause_button.isChecked():
            self.status.emit("Log upload: paused")
            return
        if self.thread is not None and self.thread.is_alive():
            self.rerun = True   # new logs may have finished meanwhile
            return
        if self.session_running():
            self.status.emit("Log upload: waiting for the session to stop")
            return
        try:
            key = resolve_key(key)
        except OSError as e:
            self.status.emit(f"Log upload: cannot read the API key file ({e.strerror})")
            return
        if not key:
            self.status.emit("Log upload: the API key file is empty")
            return
        self.cancel.clear()
        self.rerun = False
        kwargs = {"sessions": list(self.sessions), "finished": set(self.finished), "cancel": self.cancel,
                  "report": self.status.emit, "record": self.recorded.emit, "blocked": self.blocked.emit}

        def run():
            try:
                upload_new_logs(server, key, **kwargs)
            finally:
                self.done.emit()

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()

    def handle_done(self):
        self.refresh_pending()
        if self.rerun:
            self.start()

    def handle_status(self, text):
        self.last_status = text
        self.update_summary()

    def handle_blocked(self, problem):
        self.problem = problem
        self.update_pending_status()

    def handle_recorded(self, entry):
        self.add_entry(entry)
        name = entry[1]
        if name and entry[2] != "Failed":
            self.pending = [p for p in self.pending if p["name"] != name]
            self.fill_pending()

    # --- pending logs

    def refresh_pending(self):
        server, _ = self.settings()
        self.pending = scan_pending(server, self.sessions, self.finished) if server else []
        self.fill_pending()

    def pending_status(self, p):
        """Why this pending log is not uploaded yet."""
        if not p["complete"]:
            if self.session_running() and self.session_start is not None and p["mtime"] >= self.session_start:
                return "Recording (session running)"
            ready = time.strftime("%H:%M", time.localtime(p["mtime"] + QUIET_TIME))
            return f"Still changing, waits until unchanged for {QUIET_TIME // 60} min (about {ready})"
        if p["sig"][0][0] == 0:
            return "Empty, will be skipped"
        if self.pause_button.isChecked():
            return "Paused"
        if self.session_running():
            return "Waiting for the session to stop"
        failed = self.last_failure.get(p["name"])
        if failed:
            return f"Failed, retried later: {failed}"
        if self.problem:
            return f"Waiting: {self.problem}"
        return "Queued"

    @property
    def last_failure(self):
        failures = {}
        for row in range(self.history_table.rowCount() - 1, -1, -1):  # oldest first
            name = self.history_table.item(row, 1).text()
            result = self.history_table.item(row, 2).text()
            if name:
                failures.pop(name, None)
                if result == "Failed":
                    failures[name] = self.history_table.item(row, 3).text()
        return failures

    def fill_pending(self):
        selected = {self.pending_table.item(i.row(), 0).text() for i in self.pending_table.selectionModel().selectedRows()}
        self.pending_table.setRowCount(0)
        for p in sorted(self.pending, key=lambda p: p["name"], reverse=True):
            row = self.pending_table.rowCount()
            self.pending_table.insertRow(row)
            values = (p["name"], human_size(p["size"]),
                      time.strftime("%Y-%m-%d %H:%M", time.localtime(p["mtime"])), "")
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setToolTip(value)
                if col == 1:
                    item.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                self.pending_table.setItem(row, col, item)
            if p["name"] in selected:
                self.pending_table.selectRow(row)
        self.update_pending_status()

    def update_pending_status(self):
        by_name = {p["name"]: p for p in self.pending}
        for row in range(self.pending_table.rowCount()):
            p = by_name[self.pending_table.item(row, 0).text()]
            text = self.pending_status(p)
            item = QTableWidgetItem(text)
            item.setToolTip(text)
            self.pending_table.setItem(row, 3, item)
        n = len(self.pending)
        self.pending_button.setText(f"Pending ({n})" if n else "Pending")
        self.update_summary()
        self.update_buttons()

    def selected_names(self, table):
        return [table.item(i.row(), 0 if table is self.pending_table else 1).text()
                for i in table.selectionModel().selectedRows()]

    def ignore(self, logs):
        server, _ = self.settings()
        if not server or not logs:
            return
        for entry in ignore_logs(server, logs):
            self.add_entry(entry)
        names = {log["name"] for log in logs}
        self.pending = [p for p in self.pending if p["name"] not in names]
        self.fill_pending()

    def ignore_selected(self):
        names = set(self.selected_names(self.pending_table))
        self.ignore([p for p in self.pending if p["name"] in names])

    def ignore_all(self):
        self.ignore(list(self.pending))

    def requeue_selected(self):
        server, _ = self.settings()
        names = [n for n in dict.fromkeys(self.selected_names(self.history_table)) if n]
        for entry in requeue_logs(server, names):
            self.add_entry(entry)
        self.refresh_pending()
        self.start()

    # --- view

    def show_view(self, *_):
        self.views.setCurrentWidget(self.pending_page if self.pending_button.isChecked() else self.history_page)
        self.update_buttons()

    def update_buttons(self):
        pending_view = self.pending_button.isChecked()
        configured = self.configured()
        self.ignore_button.setVisible(pending_view)
        self.ignore_all_button.setVisible(pending_view)
        self.requeue_button.setVisible(not pending_view)
        self.ignore_button.setEnabled(configured and bool(self.pending_table.selectionModel().selectedRows()))
        self.ignore_all_button.setEnabled(configured and bool(self.pending))
        ignored = [i for i in self.history_table.selectionModel().selectedRows()
                   if self.history_table.item(i.row(), 2).text() == "Ignored"]
        self.requeue_button.setEnabled(configured and bool(ignored))
        self.upload_button.setEnabled(configured and not self.pause_button.isChecked())
        self.pause_button.setEnabled(configured)

    def update_summary(self):
        server, key = self.settings()
        if not server or not key:
            text = "Enter the log server URL and API key and press Save to upload logs automatically."
        else:
            n = len(self.pending)
            size = sum(p["size"] for p in self.pending)
            queue = f"{n} pending ({human_size(size)})" if n else "nothing pending"
            text = f"{self.last_status or 'Log upload'} — {queue}"
            if (self.problem and not self.pause_button.isChecked()
                    and "not reachable" not in text and "refused" not in text):
                text += f", {self.problem}"
        self.summary_label.setText(text)
        self.summary_label.setToolTip(
            f"Server: {server or '-'}\nLogs folder: {LOGS_DIR}\n"
            f"Logs are uploaded when a session stops, after an SD log extraction, at start-up and every "
            f"{RETRY_INTERVAL // 60} minutes.\n"
            f"Logs not recorded by this Paparazzi Center are uploaded after {QUIET_TIME // 60} minutes without changes.")
        self.update_buttons()

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
