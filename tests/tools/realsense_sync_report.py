#!/usr/bin/env python3
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Frame delivery and multi-camera synchronization report of RealSense QCNode frame logs.

Each RealSense sample of QCNodeSampleApp writes one CSV line per delivered sample with
``-k frame_log -v <file>``:

    camera_name,serial,stream,frame_number,timestamp_ns,sensor_timestamp_ms,timestamp_domain,
    metadata_sensor_timestamp_us,host_receive_ns

This script reads the logs of one or more cameras and reports, per camera and stream:
  - the actual rate, frame counter gaps, missing frame rate and maximum consecutive missing frames;
and, per stream across the cameras (the first camera is the reference, e.g. the sync master):
  - the complete multi-camera frameset ratio: reference frames with a frame of every camera
    within half a frame period;
  - the inter-camera timestamp gap (each camera vs. the reference): mean, standard deviation,
    P50, P90, P99, minimum, maximum and outliers (above --outlier-ms).

The first --skip-s seconds of each stream are ignored (start-up).

Example:
    realsense_sync_report.py /tmp/cam1.csv /tmp/cam2.csv /tmp/cam3.csv --streams color infra1 infra2
"""
import argparse
import bisect
import csv
import math
import statistics
import sys
from collections import defaultdict


def percentile(values, p):
    if not values:
        return float("nan")
    s = sorted(values)
    k = (len(s) - 1) * p / 100.0
    lo, hi = math.floor(k), math.ceil(k)
    return s[lo] if lo == hi else s[lo] + (s[hi] - s[lo]) * (k - lo)


def load(paths):
    """{(camera, stream): [(frame_number, timestamp_ns, host_ns)]}"""
    data = defaultdict(list)
    for path in paths:
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                key = (row["camera_name"], row["stream"])
                data[key].append((int(row["frame_number"]), int(row["timestamp_ns"]),
                                  int(row["host_receive_ns"] or 0)))
    return data


def skip_start(samples, skip_s):
    if not samples:
        return samples
    t0 = samples[0][1]
    return [s for s in samples if s[1] - t0 >= skip_s * 1e9]


def stream_report(samples):
    n = len(samples)
    if n < 2:
        return None
    duration = (samples[-1][1] - samples[0][1]) / 1e9
    gaps = missed = max_missed = 0
    for (fn0, _, _), (fn1, _, _) in zip(samples, samples[1:]):
        if fn1 > fn0 + 1:
            gaps += 1
            missed += fn1 - fn0 - 1
            max_missed = max(max_missed, fn1 - fn0 - 1)
    expected = samples[-1][0] - samples[0][0] + 1
    return {
        "samples": n,
        "fps": (n - 1) / duration if duration > 0 else 0.0,
        "gaps": gaps,
        "missed": missed,
        "missing_rate": 100.0 * missed / expected if expected > 0 else 0.0,
        "max_consecutive_missed": max_missed,
        "period_ns": (samples[-1][1] - samples[0][1]) / (n - 1),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+", help="frame_log CSV files, the first camera is the reference")
    ap.add_argument("--streams", nargs="*", default=None,
                    help="streams to report (default: all), e.g. color infra1 infra2")
    ap.add_argument("--skip-s", type=float, default=2.0, help="start-up seconds ignored (2)")
    ap.add_argument("--outlier-ms", type=float, default=2.0, help="gap counted as outlier (2 ms)")
    args = ap.parse_args()

    data = load(args.logs)
    cameras = []
    for key in data:
        if key[0] not in cameras:
            cameras.append(key[0])
    streams = sorted({k[1] for k in data})
    if args.streams:
        streams = [s for s in streams if s in args.streams]
    for key in data:
        data[key] = skip_start(sorted(data[key], key=lambda s: s[1]), args.skip_s)

    print("== Per camera and stream")
    print(f"{'camera':<10} {'stream':<24} {'samples':>8} {'fps':>8} {'gaps':>6} {'missed':>7} "
          f"{'missing %':>10} {'max consec':>10}")
    periods = {}
    for cam in cameras:
        for st in streams:
            r = stream_report(data.get((cam, st), []))
            if r is None:
                continue
            periods[(cam, st)] = r["period_ns"]
            print(f"{cam:<10} {st:<24} {r['samples']:>8} {r['fps']:>8.3f} {r['gaps']:>6} "
                  f"{r['missed']:>7} {r['missing_rate']:>10.4f} {r['max_consecutive_missed']:>10}")

    if len(cameras) < 2:
        return 0

    ref = cameras[0]
    print(f"\n== Inter-camera synchronization (reference {ref}, outlier > {args.outlier_ms} ms)")
    for st in streams:
        ref_samples = data.get((ref, st), [])
        if (ref, st) not in periods or not ref_samples:
            continue
        half = periods[(ref, st)] / 2.0
        others = [c for c in cameras[1:] if data.get((c, st))]
        if not others:
            continue
        stamps = {c: [s[1] for s in data[(c, st)]] for c in others}
        complete = 0
        gaps_ms = defaultdict(list)
        for _, t_ref, _ in ref_samples:
            matched = True
            for c in others:
                ts = stamps[c]
                i = bisect.bisect_left(ts, t_ref)
                best = None
                for j in (i - 1, i):
                    if 0 <= j < len(ts) and (best is None or abs(ts[j] - t_ref) < abs(best - t_ref)):
                        best = ts[j]
                if best is None or abs(best - t_ref) > half:
                    matched = False
                    continue
                gaps_ms[c].append((best - t_ref) / 1e6)
            complete += matched
        print(f"-- {st}: complete framesets {complete}/{len(ref_samples)} "
              f"({100.0 * complete / len(ref_samples):.3f} %)")
        print(f"   {'camera':<10} {'pairs':>7} {'mean':>8} {'std':>8} {'P50':>8} {'P90':>8} "
              f"{'P99':>8} {'min':>8} {'max':>8} {'outliers':>8}   (|gap| in ms)")
        for c in others:
            g = [abs(x) for x in gaps_ms[c]]
            if not g:
                print(f"   {c:<10} {0:>7}")
                continue
            print(f"   {c:<10} {len(g):>7} {statistics.fmean(g):>8.3f} "
                  f"{(statistics.pstdev(g) if len(g) > 1 else 0.0):>8.3f} {percentile(g, 50):>8.3f} "
                  f"{percentile(g, 90):>8.3f} {percentile(g, 99):>8.3f} {min(g):>8.3f} {max(g):>8.3f} "
                  f"{sum(1 for x in g if x > args.outlier_ms):>8}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
