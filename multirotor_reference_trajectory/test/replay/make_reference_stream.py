#!/usr/bin/env python3
"""Write the scripted request stream for the reference replay harness.

Usage: make_reference_stream.py OUT.stream

Stream format (little-endian): magic b"MRTRPLY1", then records of
u64 receive-time ns, u8 kind, u32 length, ROS-serialized message bytes.
Kinds: 1 AnalyticReference, 2 WaypointReferenceRequest, 3 SampledReference,
4 reset (std_msgs/Empty).

The scenario covers every analytic type (with default, explicit and
non-finite parameters), sampled references, MINCO waypoint plans (fixed and
optimized segment times, region constraints), rejected requests, a future
start time, expiry back to Ready, and reset.
"""
import io
import math
import struct
import sys

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from multirotor_reference_trajectory_msgs.msg import (AnalyticReference, FlatReferencePoint,
                                                      SampledReference, WaypointReferenceRequest)
from std_msgs.msg import Empty

T0 = 1000.0  # receive time of the first record, seconds
records = []


def at(t, kind, msg):
    buf = io.BytesIO()
    msg.serialize(buf)
    records.append((int(round((T0 + t) * 1e9)), kind, buf.getvalue()))


def pose(x, y, z, yaw=0.0):
    return Pose(Point(x, y, z), Quaternion(0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)))


next_id = [0]


def analytic(t, kind, params, duration=6.0, origin=(0.0, 0.0, 1.0, 0.0), start=0.0):
    next_id[0] += 1
    m = AnalyticReference()
    m.header.stamp = rospy.Time.from_sec(T0 + t)
    m.request_id = 100 + next_id[0]
    m.trajectory_id = next_id[0]
    m.revision = 1
    m.analytic_type = kind
    m.flags = 0
    m.start_time = rospy.Time.from_sec(start) if start else rospy.Time()
    m.duration = duration
    m.origin = pose(*origin)
    m.params = params
    at(t, 1, m)


def sampled(t, n=40, dt=0.1, monotonic=True):
    next_id[0] += 1
    m = SampledReference()
    m.header.stamp = rospy.Time.from_sec(T0 + t)
    m.trajectory_id = next_id[0]
    m.revision = 1
    m.sample_dt = dt
    for i in range(n):
        s = i * dt if monotonic or i != n // 2 else 0.0
        w = 0.5
        p = FlatReferencePoint()
        p.t_from_start = s
        p.position = Point(math.cos(w * s), math.sin(w * s), 1.0 + 0.1 * s)
        p.velocity = Vector3(-w * math.sin(w * s), w * math.cos(w * s), 0.1)
        p.acceleration = Vector3(-w * w * math.cos(w * s), -w * w * math.sin(w * s), 0.0)
        p.jerk = Vector3(w ** 3 * math.sin(w * s), -w ** 3 * math.cos(w * s), 0.0)
        p.snap = Vector3(w ** 4 * math.cos(w * s), w ** 4 * math.sin(w * s), 0.0)
        p.yaw = 0.2 * s
        p.yaw_rate = 0.2
        m.points.append(p)
    at(t, 3, m)


def waypoints(t, points, segment_times=(), constraints=(), sizes=(), revision=0, stamp=None):
    next_id[0] += 1
    m = WaypointReferenceRequest()
    m.header.stamp = rospy.Time.from_sec(T0 + t if stamp is None else stamp)
    m.request_id = 100 + next_id[0]
    m.trajectory_id = next_id[0]
    m.revision = revision
    m.waypoints = [pose(*p) for p in points]
    m.constraint_types = list(constraints)
    m.region_size = [Vector3(*s) for s in sizes]
    m.segment_times = list(segment_times)
    m.desired_speed = 1.0
    m.time_weight = 0.1
    m.max_iterations = 80
    m.rel_cost_tol = 1.0e-5
    m.objective = WaypointReferenceRequest.OBJECTIVE_MINCO
    at(t, 2, m)


A = AnalyticReference
W = WaypointReferenceRequest
t = 0.5
analytic(t, A.ANALYTIC_CIRCLE_ENTRY, [1.5, 1.0, 1.2, 0.2, 0.3, 2.0, 0.5, -0.5], duration=8.0); t += 2.0
analytic(t, A.ANALYTIC_HOLD, [], duration=3.0, origin=(0.5, 0.2, 1.0, 0.7)); t += 2.0
analytic(t, A.ANALYTIC_CIRCLE, [1.0, 0.8, 1.5]); t += 2.0
analytic(t, A.ANALYTIC_HEIGHT_CIRCLE, [1.0, 0.8, 1.5, 0.3, 0.4]); t += 2.0
analytic(t, A.ANALYTIC_FIGURE_EIGHT, [1.2, 0.9, 1.4], origin=(0.1, 0.1, 1.0, 0.0)); t += 2.0
analytic(t, A.ANALYTIC_LINE, [2.0, 1.0, 1.5, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0], duration=4.0); t += 2.0
analytic(t, A.ANALYTIC_LINE, [], duration=0.0); t += 2.0  # defaults, no duration
analytic(t, A.ANALYTIC_LEMNISCATE, [1.0, 0.9, 1.2]); t += 2.0
analytic(t, A.ANALYTIC_HELIX_YZ, [0.8, 1.5, 10.0]); t += 2.0
analytic(t, A.ANALYTIC_HELIX_XY, [0.8, 0.9, 10.0]); t += 2.0
analytic(t, A.ANALYTIC_TORUS_KNOT, [0.3, 0.3], origin=(0.0, 0.0, 1.5, 0.3)); t += 2.0
analytic(t, A.ANALYTIC_TORUS_KNOT, [0.3, 0.3, 2.0, 0.2, 0.1, 1.6], duration=10.0, origin=(0.0, 0.0, 1.0, 0.3)); t += 2.0
analytic(t, A.ANALYTIC_CIRCLE, [float("nan"), 0.8, float("inf")]); t += 2.0  # non-finite -> defaults
analytic(t, 42, [1.0, 1.0, 1.0]); t += 2.0  # unknown type -> circle entry
analytic(t, A.ANALYTIC_CIRCLE, [1.0, 0.8, 1.5], start=T0 + t + 3.0); t += 2.0  # future start
sampled(t); t += 2.0
sampled(t, monotonic=False); t += 1.0  # rejected
waypoints(t, [(0, 0, 1), (1, 0.5, 1.2), (2, 0, 1)], segment_times=(1.0, 1.0)); t += 3.0
waypoints(t, [(0, 0, 1), (1, 1, 1.5), (2, 0, 1.2), (3, 1, 1)], constraints=(W.CONSTRAINT_POINT, W.CONSTRAINT_SPHERE, W.CONSTRAINT_BOX, W.CONSTRAINT_POINT),
          sizes=((0, 0, 0), (0.2, 0.2, 0.2), (0.3, 0.2, 0.1), (0, 0, 0)), revision=5); t += 4.0
waypoints(t, [(0, 0, 1)]); t += 1.0  # rejected: one waypoint
waypoints(t, [(0, 0, 1), (1, 0, 1)], segment_times=(1.0, 1.0)); t += 1.0  # rejected: segment count
waypoints(t, [(0, 0, 1), (1, 0, 1), (1, 1, 1)], segment_times=(0.8, 0.8), stamp=T0 + t + 1.5); t += 1.0
analytic(t, A.ANALYTIC_HOLD, [], duration=1.0); t += 3.0  # expires -> Ready
at(t, 4, Empty()); t += 1.0  # reset -> SelfCheck -> Ready
sampled(t); t += 1.0
at(t, 4, Empty()); t += 0.5  # reset while Active
analytic(t, A.ANALYTIC_CIRCLE_ENTRY, [], duration=5.0); t += 3.0

records.sort(key=lambda r: r[0])
with open(sys.argv[1], "wb") as out:
    out.write(b"MRTRPLY1")
    for t_ns, kind, data in records:
        out.write(struct.pack("<QBI", t_ns, kind, len(data)))
        out.write(data)
print(f"{len(records)} records -> {sys.argv[1]}")
