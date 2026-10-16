#!/usr/bin/env python3
"""Control panel for the observer demo: record, tour, tap, analyze, from a browser.

Runs on the arm computer beside the arm node. Every button maps to one of the
scripts in this folder; the page polls status once a second and embeds the
latest report. Stdlib only, plus rclpy for the live joint-state rate.

  source env.sh
  python3 panel.py --port 8790            # then  ssh -L 8790:localhost:8790 abra
  python3 panel.py --port 8790 --mock     # no ROS, no arm: synthetic runs, for UI work

The tour button MOVES THE ARM. The page asks before it does, and the
e-stop is still the only real safety.
"""

import argparse
import http.server
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
import urllib.parse

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")
SAFE = re.compile(r"^[A-Za-z0-9_.-]{1,64}$")


class State:
    """Everything the page polls. One lock; subprocesses are tracked by Popen."""

    def __init__(self, urdf, mock):
        self.lock = threading.Lock()
        self.urdf = urdf
        self.mock = mock
        self.recorder = None  # Popen
        self.run = None  # current run name
        self.tour = None  # Popen
        self.tour_log = []
        self.sequence = None  # phase string while the one-button sequence runs
        self.markers = 0
        self.rate_hz = 0.0
        self.arm = "unknown"
        self.last_report = None
        self.error = None
        self.rows_at_start = 0

    def snapshot(self):
        with self.lock:
            rec = self.recorder is not None and self.recorder.poll() is None
            tour = self.tour is not None and self.tour.poll() is None
            rows = 0
            if self.run:
                p = os.path.join(RUNS, self.run + ".csv")
                if os.path.exists(p):
                    with open(p, "rb") as f:
                        rows = max(0, sum(1 for _ in f) - 1)
            return {
                "recording": rec,
                "run": self.run,
                "rows": rows,
                "markers": self.markers,
                "tour_running": tour,
                "tour_log": self.tour_log[-8:],
                "sequence": self.sequence,
                "rate_hz": round(self.rate_hz, 1),
                "arm": self.arm,
                "last_report": self.last_report,
                "error": self.error,
                "mock": self.mock,
                "runs": sorted(f[:-5] for f in os.listdir(RUNS) if f.endswith(".html"))
                if os.path.isdir(RUNS)
                else [],
            }


S = None


def env_python():
    return [sys.executable]


def start_record(name):
    if not SAFE.match(name):
        raise ValueError("run name: letters, digits, _ . - only")
    with S.lock:
        if S.recorder is not None and S.recorder.poll() is None:
            raise RuntimeError("already recording")
        os.makedirs(RUNS, exist_ok=True)
        csv = os.path.join(RUNS, name + ".csv")
        marker = os.path.join(RUNS, name + ".TAP")
        if os.path.exists(marker):
            os.remove(marker)
        if S.mock:
            cmd = env_python() + [
                os.path.join(HERE, "synthetic.py"),
                "--urdf",
                S.urdf,
                "--out",
                csv,
            ]
            S.recorder = subprocess.Popen(
                ["sh", "-c", " ".join(map(_q, cmd)) + " && sleep 3600"]
            )
        else:
            cmd = env_python() + [
                os.path.join(HERE, "record_joint_states.py"),
                "--out",
                csv,
                "--marker-file",
                marker,
            ]
            S.recorder = subprocess.Popen(
                cmd,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        S.run = name
        S.markers = 0
        S.error = None


def _q(s):
    return "'" + s.replace("'", "'\\''") + "'"


def stop_record():
    with S.lock:
        if S.recorder is None:
            return
        if S.recorder.poll() is None:
            S.recorder.send_signal(signal.SIGINT)
            try:
                S.recorder.wait(timeout=5)
            except subprocess.TimeoutExpired:
                S.recorder.kill()
        S.recorder = None


def tap(kind="tap"):
    """kind: "tap" for a real poke, "phase" for the baseline/tap boundary."""
    with S.lock:
        if S.run is None:
            raise RuntimeError("not recording")
        with open(os.path.join(RUNS, S.run + ".TAP"), "w") as f:
            f.write(kind + "\n")
        if kind == "tap":
            S.markers += 1


def start_tour(loops, speed):
    with S.lock:
        if S.tour is not None and S.tour.poll() is None:
            raise RuntimeError("tour already running")
        S.tour_log = []
        if S.mock:
            S.tour = subprocess.Popen(
                [
                    "sh",
                    "-c",
                    "for i in 0 1 2 3; do echo leg $i: SUCCESSFUL; sleep 1; done",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
        else:
            cmd = env_python() + [
                os.path.join(HERE, "tour.py"),
                "--go",
                "--loops",
                str(int(loops)),
                "--speed",
                str(float(speed)),
            ]
            S.tour = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
            )
        proc = S.tour
    threading.Thread(target=_pump, args=(proc,), daemon=True).start()


def _pump(proc):
    for line in proc.stdout:
        with S.lock:
            S.tour_log.append(line.rstrip())


def wait_tour():
    while True:
        with S.lock:
            p = S.tour
        if p is None or p.poll() is not None:
            return p.returncode if p else 0
        time.sleep(0.2)


def analyze(name, gain, before, after):
    if not SAFE.match(name):
        raise ValueError("bad run name")
    csv = os.path.join(RUNS, name + ".csv")
    out = os.path.join(RUNS, name + ".html")
    cmd = env_python() + [
        os.path.join(HERE, "report.py"),
        "--csv",
        csv,
        "--urdf",
        S.urdf,
        "--out",
        out,
        "--gain",
        str(gain),
        "--tap-before",
        str(before),
        "--tap-after",
        str(after),
    ]
    r = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if r.returncode != 0:
        raise RuntimeError(
            r.stderr.strip().splitlines()[-1] if r.stderr.strip() else "report failed"
        )
    with S.lock:
        S.last_report = name
    return name


def sequence(name, loops, speed):
    """One button: record, baseline loop, marker, tap loops, stop, analyze."""

    def phase(p):
        with S.lock:
            S.sequence = p

    try:
        phase("starting recorder")
        start_record(name)
        time.sleep(3)
        phase("baseline loop (do not touch the arm)")
        start_tour(1, speed)
        if wait_tour() != 0:
            raise RuntimeError("baseline tour failed")
        time.sleep(1)
        tap("phase")  # boundary between baseline and tap loops; not a tap
        phase("TAP NOW: poke the arm while it moves")
        start_tour(loops, speed)
        if wait_tour() != 0:
            raise RuntimeError("tap tour failed")
        time.sleep(1)
        stop_record()
        phase("analyzing")
        analyze(name, 20.0, 0.2, 0.8)
        phase(None)
    except Exception as e:  # noqa: BLE001  (reported to the page, not swallowed)
        stop_record()
        with S.lock:
            S.error = str(e)
            S.sequence = None


def arm_status_loop():
    """sheppy status, every 5 s, from the deployments dir if it exists."""
    dep = os.path.expanduser("~/rammp-deployments/december_2026")
    while True:
        if S.mock:
            with S.lock:
                S.arm = "mock"
        elif os.path.isdir(dep):
            try:
                r = subprocess.run(
                    ["sheppy", "status"],
                    check=False,
                    cwd=dep,
                    capture_output=True,
                    text=True,
                    timeout=10,
                )
                m = re.search(r"^arm\s+(\S+)", r.stdout, re.MULTILINE)
                with S.lock:
                    S.arm = m.group(1) if m else "unknown"
            except Exception:  # noqa: BLE001
                with S.lock:
                    S.arm = "unknown"
        time.sleep(5)


def rate_loop():
    """Joint-state rate from a subscription in this process; 0 when nothing arrives."""
    if S.mock:
        while True:
            with S.lock:
                S.rate_hz = 98.0 if S.recorder is not None else 0.0
            time.sleep(1)
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
    except ImportError:
        return
    rclpy.init()
    node = Node("momentum_observer_panel")
    count = [0]
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    node.create_subscription(
        JointState, "/joint_states", lambda m: count.__setitem__(0, count[0] + 1), qos
    )
    last = time.monotonic()
    while True:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if now - last >= 1.0:
            with S.lock:
                S.rate_hz = count[0] / (now - last)
            count[0] = 0
            last = now


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=HERE, **k)

    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urllib.parse.urlparse(self.path).path
        if path == "/":
            with open(os.path.join(HERE, "panel.html"), "rb") as f:
                body = f.read()
            for n in ("tokens.css", "ax-charts.js", "ax-fx.js"):
                with open(os.path.join(HERE, "web", n), "rb") as f:
                    body = body.replace(
                        b"/*INLINE " + n.encode() + b"*/",
                        f.read().replace(b"</", b"<\\/"),
                    )
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path == "/api/status":
            self._json(S.snapshot())
        elif path.startswith("/runs/"):
            super().do_GET()
        else:
            self.send_error(404)

    def do_POST(self):
        path = urllib.parse.urlparse(self.path).path
        n = int(self.headers.get("Content-Length") or 0)
        body = json.loads(self.rfile.read(n) or b"{}")
        try:
            if path == "/api/record/start":
                start_record(body.get("name") or time.strftime("run_%H%M%S"))
            elif path == "/api/record/stop":
                stop_record()
            elif path == "/api/tap":
                tap()
            elif path == "/api/tour":
                start_tour(body.get("loops", 1), body.get("speed", 0.6))
            elif path == "/api/analyze":
                analyze(
                    body["name"],
                    float(body.get("gain", 20)),
                    float(body.get("before", 0.2)),
                    float(body.get("after", 0.8)),
                )
            elif path == "/api/sequence":
                name = body.get("name") or time.strftime("run_%H%M%S")
                if not SAFE.match(name):
                    raise ValueError("run name: letters, digits, _ . - only")
                with S.lock:
                    if S.sequence:
                        raise RuntimeError("a sequence is already running")
                    S.sequence = "queued"
                threading.Thread(
                    target=sequence,
                    args=(
                        name,
                        int(body.get("loops", 2)),
                        float(body.get("speed", 0.6)),
                    ),
                    daemon=True,
                ).start()
            elif path == "/api/arm/restart":
                dep = os.path.expanduser("~/rammp-deployments/december_2026")
                subprocess.Popen(["sheppy", "restart", "arm"], cwd=dep)
            else:
                return self.send_error(404)
            self._json({"ok": True, **S.snapshot()})
        except Exception as e:  # noqa: BLE001
            self._json({"ok": False, "error": str(e)}, 400)


def main():
    global S
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument(
        "--bind",
        default="127.0.0.1",
        help="0.0.0.0 to reach it from a phone on the LAN",
    )
    ap.add_argument(
        "--urdf", default=os.environ.get("KINOVA_URDF", os.path.join(HERE, "arm.urdf"))
    )
    ap.add_argument(
        "--mock", action="store_true", help="no ROS, no arm; synthetic data"
    )
    args = ap.parse_args()
    S = State(args.urdf, args.mock)
    os.makedirs(RUNS, exist_ok=True)
    threading.Thread(target=arm_status_loop, daemon=True).start()
    threading.Thread(target=rate_loop, daemon=True).start()
    srv = http.server.ThreadingHTTPServer((args.bind, args.port), Handler)

    # rclpy installs its own SIGTERM/SIGINT handling once the rate thread starts;
    # make a plain kill still stop the recorder and exit.
    def _stop(signum, frame):
        stop_record()
        os._exit(0)

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    print(
        f"panel on http://{args.bind}:{args.port}/  (mock={args.mock}, urdf={args.urdf})",
        flush=True,
    )
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        stop_record()


if __name__ == "__main__":
    main()
