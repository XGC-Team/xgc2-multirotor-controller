#!/usr/bin/env python3
"""The reference module, the controller module and a scripted vehicle in the real module host.

The unit tests drive the modules with an in-test host. This runs them in xgc2-module-host with
the channels of an entity, a scripted vehicle (scripted_vehicle.cpp) in place of the MAVROS edge,
and the operator's commands: takeoff, then custom1 once hovering, then land. It follows the flight
through the host's health report and checks what only the real host can show: the instances stay
healthy, the controller asks the reference module once and gets its active reference, and (for NMPC)
every solve of the worker wakes the host and is answered with an attitude-rate command.

    run_chain.py --host xgc2-module-host --controller libpx4_multirotor_controller_module.so \
        --reference-dirs DIR[:DIR...] --vehicle libpx4_scripted_vehicle.so [--backend nmpc|dfbc]

Exit status 0 when every check holds.
"""
import argparse
import http.client
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid

FLIGHT_LIMIT_S = 60.0

MANIFEST = """\
entity = "px4_chain_{backend}"

[host]
workers = 3

[control]
socket = "{socket}"

[[module]]
name = "reference"
path = "{reference}"
[[module]]
name = "controller"
path = "{controller}"
[[module]]
name = "vehicle"
path = "{vehicle}"

[[instance]]
name = "reference"
module = "reference"
[instance.bind]
analytic = "ref.request.analytic"
status = "ref.status"
active_analytic = "ref.active.analytic"
active_sampled = "ref.active.sampled"

[[instance]]
name = "controller"
module = "controller"
[instance.config]
world_boundary_json = "null"
tracking_backend = "{backend}"
skip_takeoff_init_disarm = true
takeoff_altitude = 1.0
[instance.bind]
state_estimate = "est.state"
local_pose = "mavros.local_pose"
local_velocity = "mavros.local_velocity"
imu = "mavros.imu"
fcu_state = "mavros.state"
battery = "mavros.battery"
vrpn_pose = "pose"
command = "command"
hover_thrust = "hover_thrust"
ref_active_analytic = "ref.active.analytic"
ref_active_sampled = "ref.active.sampled"
setpoint = "mavros.setpoint"
attitude_rate = "mavros.attitude_rate"
fcu_request = "mavros.fcu_request"
status = "controller.status"
ref_request = "ref.request.analytic"

[[instance]]
name = "vehicle"
module = "vehicle"
[instance.bind]
state_estimate = "est.state"
local_pose = "mavros.local_pose"
local_velocity = "mavros.local_velocity"
imu = "mavros.imu"
fcu_state = "mavros.state"
battery = "mavros.battery"
vrpn_pose = "pose"
command = "command"
hover_thrust = "hover_thrust"
setpoint = "mavros.setpoint"
attitude_rate = "mavros.attitude_rate"
fcu_request = "mavros.fcu_request"
status = "controller.status"
"""


class UnixConnection(http.client.HTTPConnection):
    def __init__(self, path):
        super().__init__("localhost")
        self.path = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect(self.path)


class Host:
    """xgc2-module-host as a child process."""

    def __init__(self, binary, manifest, socket_path):
        self.socket_path = socket_path
        self.lines = []
        self.instance_id = None
        self.proc = subprocess.Popen([binary, "--manifest", manifest, "--log-level", "info"],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.proc.stdout:
            self.lines.append(line.rstrip("\n"))
            if self.instance_id is None and line.startswith('{"service_ref"'):
                self.instance_id = json.loads(line)["service_ref"]["instance_id"]

    def wait_for_control_plane(self, limit=15.0):
        deadline = time.time() + limit
        while time.time() < deadline:
            if self.proc.poll() is not None:
                raise RuntimeError("the host exited at start-up")
            if self.instance_id is not None and os.path.exists(self.socket_path):
                return
            time.sleep(0.05)
        raise RuntimeError("the host's control plane did not come up")

    def health(self):
        conn = UnixConnection(self.socket_path)
        try:
            conn.request("GET", "/v1/health", headers={
                "X-Xrpc-Instance-ID": self.instance_id,
                "X-Request-ID": str(uuid.uuid4()),
                "X-Xrpc-Timeout-Ms": "3000",
            })
            response = conn.getresponse()
            body = json.loads(response.read())
            if response.status != 200:
                raise RuntimeError("health: %s" % body)
            return body
        finally:
            conn.close()

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
        try:
            self.proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        self.reader.join(timeout=5)
        return self.proc.returncode


def by_name(items):
    return {item["name"]: item for item in items}


def find_reference(dirs):
    for directory in dirs.split(":"):
        path = os.path.abspath(os.path.join(directory, "libpx4_multirotor_reference_module.so"))
        if os.path.exists(path):
            return path
    raise SystemExit("libpx4_multirotor_reference_module.so is not built in " + dirs)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--host", required=True)
    parser.add_argument("--controller", required=True)
    parser.add_argument("--reference-dirs", required=True)
    parser.add_argument("--vehicle", required=True)
    parser.add_argument("--backend", choices=("nmpc", "dfbc"), default="nmpc")
    args = parser.parse_args()

    work = tempfile.mkdtemp(prefix="px4chain-")  # mode 0700, as the control socket requires
    manifest = os.path.join(work, "entity.toml")
    socket_path = os.path.join(work, "control.sock")
    with open(manifest, "w") as out:
        out.write(MANIFEST.format(backend=args.backend, socket=socket_path,
                                  reference=find_reference(args.reference_dirs),
                                  controller=os.path.abspath(args.controller),
                                  vehicle=os.path.abspath(args.vehicle)))

    host = Host(args.host, manifest, socket_path)
    problems = []
    final = None
    try:
        host.wait_for_control_plane()
        deadline = time.time() + FLIGHT_LIMIT_S
        while time.time() < deadline:
            time.sleep(0.5)
            health = host.health()
            instances = by_name(health["instances"])
            if instances["vehicle"]["reported"]["detail"] == "landing":
                final = health
                break
        else:
            problems.append("the flight did not reach the landing within %d s" % FLIGHT_LIMIT_S)
    except Exception as error:  # noqa: BLE001 - report whatever went wrong with the log
        problems.append("%s: %s" % (type(error).__name__, error))
    status = host.stop()
    shutil.rmtree(work, ignore_errors=True)
    if status != 0:
        problems.append("the host exited with status %s" % status)

    if final is not None:
        instances = by_name(final["instances"])
        channels = by_name(final["channels"])
        for name, instance in instances.items():
            if instance["state"] != "running" or instance["health"] != "ok":
                problems.append("%s: state %s, health %s" % (name, instance["state"], instance["health"]))
            for counter in ("step_errors", "misuse", "overruns"):
                if instance[counter] != 0:
                    problems.append("%s: %s = %s" % (name, counter, instance[counter]))
        if instances["reference"]["reported"]["detail"] != "Active":
            problems.append("the reference module reports %r, not Active"
                            % instances["reference"]["reported"]["detail"])
        request = channels["ref.request.analytic"]
        if request["commits"] != 1 or request["drops"] != 0:
            problems.append("the controller made %s reference requests (%s dropped), expected 1"
                            % (request["commits"], request["drops"]))
        if channels["ref.active.analytic"]["commits"] < 10:
            problems.append("the reference module published %s active references"
                            % channels["ref.active.analytic"]["commits"])
        commands = channels["mavros.attitude_rate"]["commits"]
        if commands < 100:
            problems.append("the controller sent %s attitude-rate commands in 3 s of tracking" % commands)
        wakeups = instances["controller"]["wakeups"]
        if args.backend == "nmpc" and wakeups < 100:
            problems.append("the controller was woken %s times by its solver" % wakeups)
        if args.backend == "nmpc" and wakeups < commands - 5:
            problems.append("%s attitude-rate commands but only %s wakes of the solver"
                            % (commands, wakeups))

    if problems:
        print("\n".join("FAIL: " + p for p in problems))
        print("--- host output (last 40 lines)")
        print("\n".join(host.lines[-40:]))
        return 1
    instances = by_name(final["instances"])
    print("PASS: %s chain in the real host: %d attitude-rate commands, %d solver wakeups, "
          "%d controller steps, reference %s"
          % (args.backend, by_name(final["channels"])["mavros.attitude_rate"]["commits"],
             instances["controller"]["wakeups"], instances["controller"]["steps"],
             instances["reference"]["reported"]["detail"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
