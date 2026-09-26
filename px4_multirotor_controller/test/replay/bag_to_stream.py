#!/usr/bin/env python3
"""Convert a recorded controller-input bag into the replay stream.

Usage: bag_to_stream.py FLIGHT.bag OUT.stream [NAMESPACE]   (NAMESPACE default /uav1)

Stream format (little-endian): magic b"PMCRPLY1", then records of
u64 receive-time ns, u8 kind, u32 length, ROS-serialized message bytes.
Kinds: 1 state estimate, 2 local position, 3 local velocity, 4 IMU,
5 FCU state, 6 battery, 7 VRPN pose, 8 command, 9 planner setpoint
(alg/setpoint_raw/local), 10 hover-thrust estimate. Records keep bag order.
"""
import struct
import sys

import rosbag

bag_path, out_path = sys.argv[1], sys.argv[2]
ns = sys.argv[3] if len(sys.argv) > 3 else "/uav1"
kinds = {
    ns + "/alg/state_estimator/state": 1,
    ns + "/mavros/local_position/pose": 2,
    ns + "/mavros/local_position/velocity_local": 3,
    ns + "/mavros/imu/data": 4,
    ns + "/mavros/state": 5,
    ns + "/mavros/battery": 6,
    ns + "/pose": 7,
    "/command": 8,
    ns + "/alg/setpoint_raw/local": 9,
    ns + "/hover_thrust/estimate_state": 10,
}
count = 0
with rosbag.Bag(bag_path) as bag, open(out_path, "wb") as out:
    out.write(b"PMCRPLY1")
    for topic, raw, t in bag.read_messages(topics=list(kinds), raw=True):
        data = raw[1]
        out.write(struct.pack("<QBI", t.to_nsec(), kinds[topic], len(data)))
        out.write(data)
        count += 1
print(f"{count} records -> {out_path}")
