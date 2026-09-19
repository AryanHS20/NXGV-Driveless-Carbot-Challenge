#!/usr/bin/env python3
"""Per-anchor range offset calibration for the V4 UWB bridge.

Put the tag at a KNOWN spot, then run:
  python3 ~/uwb_calib.py 500 150        # true x y of the tag in cm
  python3 ~/uwb_calib.py 500 150 30     # optional: collect for 30 s (default 20)

It consumes each radio sample only once, compares the median raw distance with
the true tape-measured distance, and prints both legacy centimetre values and
the metre parameters used by config/uwb.yaml.

Needs uwb_xy.py in the same folder (it reads ANCHORS and TAG_Z_CM from it).
"""
import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

from uwb_xy import ANCHORS, TAG_Z_CM


class Calib(Node):
    def __init__(self):
        super().__init__("uwb_calib")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(String, "/uwb3/input_json", self.on_msg, qos)
        self.samples = {aid: [] for aid in ANCHORS}
        self.boot_id = None
        self.last_sequence = {}
        self.duplicates = 0
        self.stale = 0

    def on_msg(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        boot_id = str(data.get("boot_id", "")).strip()
        if not boot_id:
            return
        if boot_id != self.boot_id:
            self.boot_id = boot_id
            self.last_sequence.clear()
        for link in data.get("links", []):
            aid = link.get("A")
            if aid not in self.samples:
                continue
            try:
                age_ms = float(link["age_ms"])
                sequence = int(link["sample_seq"])
                distance_cm = float(link["R"]) * 100.0
            except (KeyError, TypeError, ValueError):
                continue
            if not all(math.isfinite(value) for value in (age_ms, distance_cm)):
                continue
            if age_ms < 0.0 or age_ms > 400.0 or distance_cm <= 0.0:
                self.stale += 1
                continue
            previous = self.last_sequence.get(aid)
            if previous is not None:
                difference = (sequence - previous) & 0xFFFFFFFF
                if difference == 0:
                    self.duplicates += 1
                    continue
                if difference >= 0x80000000:
                    continue
            self.last_sequence[aid] = sequence
            self.samples[aid].append(distance_cm)


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    tx, ty = float(sys.argv[1]), float(sys.argv[2])
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

    rclpy.init()
    node = Calib()
    print(f"Tag must stay still at ({tx:.0f}, {ty:.0f}) cm. Collecting for {seconds:.0f} s...")
    end = time.time() + seconds
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    print()
    print(f"{'anchor':>6} {'samples':>7} {'median':>9} {'mean':>9} {'true':>7} "
          f"{'offset':>7} {'std':>7} {'MAD':>7} {'min':>7} {'max':>7}")
    offsets = {}
    for aid, (ax, ay, az) in ANCHORS.items():
        s = node.samples[aid]
        true = math.sqrt((tx - ax) ** 2 + (ty - ay) ** 2 + (TAG_Z_CM - az) ** 2)
        if len(s) < 20:
            print(f"{aid:>6} {len(s):>7}   too few samples -- is this anchor on?")
            continue
        meas = statistics.median(s)
        mean = statistics.fmean(s)
        spread = statistics.pstdev(s)
        mad = statistics.median(abs(value - meas) for value in s)
        offsets[aid] = meas - true
        print(f"{aid:>6} {len(s):>7} {meas:9.1f} {mean:9.1f} {true:7.1f} "
              f"{offsets[aid]:7.1f} {spread:7.1f} {mad:7.1f} {min(s):7.1f} {max(s):7.1f}")

    print(f"\nSkipped repeated samples: {node.duplicates}; stale/invalid-age samples: {node.stale}")

    if len(offsets) == len(ANCHORS):
        print("\nPaste this into uwb_xy.py, replacing the RANGE_OFFSET_CM block:\n")
        print("RANGE_OFFSET_CM = {")
        for aid in ANCHORS:
            print(f'    "{aid}": {offsets[aid]:.1f},')
        print("}")
        print("\nPaste these measured values into config/uwb.yaml:\n")
        for aid in ANCHORS:
            print(f"anchor_{aid}_range_offset_m: {offsets[aid] / 100.0:.4f}")

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
