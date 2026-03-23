#!/usr/bin/env python3
"""Analyze teleop session JSONL logs for profiling.

Usage:
    python3 scripts/analyze_teleop.py <session.jsonl>

Reads the JSONL session log produced by the vive teleop module and produces:
- gRPC round-trip time over time
- RDK pipeline breakdown (inputs, plan, exec, exec wait)
- Drop rate (windowed) over time
- Jump distance (mm) over time
- Drops-before-send + deadzone filtered counts
- Summary statistics (median, p95, p99)
"""

import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_jsonl(path: str) -> tuple[list[dict], list[dict], list[dict]]:
    """Load JSONL file, separating send entries, ack entries, and rdk_status entries."""
    sends = []
    acks = []
    rdk_statuses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            entry = json.loads(line)
            event = entry.get("event")
            if event == "ack":
                acks.append(entry)
            elif event == "rdk_status":
                rdk_statuses.append(entry)
            elif "x" in entry:  # send entry has pose fields
                sends.append(entry)
    return sends, acks, rdk_statuses


def print_stats(name: str, values: np.ndarray, unit: str = "ms"):
    """Print median/p95/p99/max for an array."""
    if len(values) == 0:
        return
    print(f"  {name} ({unit}):")
    print(f"    median: {np.median(values):.2f}")
    print(f"    p95:    {np.percentile(values, 95):.2f}")
    print(f"    p99:    {np.percentile(values, 99):.2f}")
    print(f"    max:    {np.max(values):.2f}")
    print()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    path = sys.argv[1]
    if not Path(path).exists():
        print(f"File not found: {path}")
        sys.exit(1)

    sends, acks, rdk_statuses = load_jsonl(path)
    if not sends:
        print("No send entries found in log.")
        sys.exit(1)

    # Build ack lookup by seq.
    ack_by_seq = {a["seq"]: a for a in acks if "seq" in a}

    # Extract send arrays.
    t0 = sends[0]["t"]
    times = np.array([s["t"] for s in sends], dtype=float)
    times_sec = (times - t0) / 1000.0

    drops = np.array([s.get("drops", 0) for s in sends])
    jump_mm = np.array([s.get("jump_mm", 0.0) for s in sends])
    dz_filtered = np.array([s.get("dz_filtered", 0) for s in sends])

    # Match gRPC times from acks.
    grpc_ms = []
    grpc_times = []
    for s in sends:
        seq = s.get("seq")
        if seq and seq in ack_by_seq:
            grpc_ms.append(ack_by_seq[seq]["grpc_ms"])
            grpc_times.append((s["t"] - t0) / 1000.0)
    grpc_ms = np.array(grpc_ms)
    grpc_times = np.array(grpc_times)

    # Extract RDK status arrays.
    rdk_times = np.array([(r["t"] - t0) / 1000.0 for r in rdk_statuses]) if rdk_statuses else np.array([])
    rdk_inputs_ms = np.array([r.get("inputs_ms", 0) for r in rdk_statuses]) if rdk_statuses else np.array([])
    rdk_plan_ms = np.array([r.get("plan_ms", 0) for r in rdk_statuses]) if rdk_statuses else np.array([])
    rdk_exec_ms = np.array([r.get("exec_ms", 0) for r in rdk_statuses]) if rdk_statuses else np.array([])
    rdk_exec_wait_ms = np.array([r.get("exec_wait_ms", 0) for r in rdk_statuses]) if rdk_statuses else np.array([])

    # Print summary.
    print(f"Session: {path}")
    print(f"  Duration: {times_sec[-1]:.1f}s")
    print(f"  Total sends: {len(sends)}")
    print(f"  Total acks: {len(acks)}")
    print(f"  RDK status samples: {len(rdk_statuses)}")
    print(f"  Total drops (sum): {int(drops.sum())}")
    print()

    print_stats("gRPC round-trip", grpc_ms)
    print_stats("Drops per send", drops, unit="count")
    print_stats("Jump distance", jump_mm, unit="mm")

    if len(rdk_plan_ms) > 0:
        print_stats("RDK CurrentInputs", rdk_inputs_ms)
        print_stats("RDK planTeleop", rdk_plan_ms)
        print_stats("RDK execute", rdk_exec_ms)
        print_stats("RDK executor wait", rdk_exec_wait_ms)

    # Correlation between drops and jump distance.
    if len(drops) > 1 and drops.sum() > 0 and jump_mm.sum() > 0:
        corr = np.corrcoef(drops, jump_mm)[0, 1]
        print(f"  Correlation (drops vs jump_mm): {corr:.3f}")
        print()

    # Determine plot layout based on available data.
    has_rdk = len(rdk_times) > 0
    nrows = 5 if has_rdk else 4
    fig, axes = plt.subplots(nrows, 1, figsize=(14, 2.5 * nrows), sharex=True)
    fig.suptitle(f"Teleop Session: {Path(path).name}", fontsize=14)

    # 1. gRPC round-trip.
    ax = axes[0]
    if len(grpc_ms) > 0:
        ax.plot(grpc_times, grpc_ms, "b-", alpha=0.7, linewidth=0.8)
        ax.axhline(y=11.1, color="r", linestyle="--", alpha=0.5, label="90Hz frame (11.1ms)")
        ax.legend(fontsize=8)
    ax.set_ylabel("gRPC (ms)")
    ax.set_title("gRPC Round-Trip Time (vive module)")
    ax.grid(True, alpha=0.3)

    # 2. RDK pipeline breakdown (if available).
    if has_rdk:
        ax = axes[1]
        ax.plot(rdk_times, rdk_plan_ms, "m-", alpha=0.8, linewidth=0.8, label="planTeleop")
        ax.plot(rdk_times, rdk_exec_ms, "c-", alpha=0.8, linewidth=0.8, label="execute")
        ax.plot(rdk_times, rdk_exec_wait_ms, "y-", alpha=0.6, linewidth=0.8, label="executor wait")
        ax.plot(rdk_times, rdk_inputs_ms, "k-", alpha=0.4, linewidth=0.8, label="CurrentInputs")
        ax.set_ylabel("Time (ms)")
        ax.set_title("RDK Pipeline Breakdown")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        plot_offset = 2
    else:
        plot_offset = 1

    # 3. Drop rate (windowed over 10 sends).
    ax = axes[plot_offset]
    if len(drops) > 10:
        window = 10
        drop_rate = np.convolve(drops > 0, np.ones(window) / window, mode="valid")
        ax.plot(times_sec[window - 1 :], drop_rate * 100, "r-", alpha=0.7, linewidth=0.8)
    ax.set_ylabel("Drop Rate (%)")
    ax.set_title("Drop Rate (10-send window)")
    ax.grid(True, alpha=0.3)

    # 4. Jump distance.
    ax = axes[plot_offset + 1]
    ax.plot(times_sec, jump_mm, "g-", alpha=0.7, linewidth=0.8)
    ax.set_ylabel("Jump (mm)")
    ax.set_title("Pose Jump Distance Between Sends")
    ax.grid(True, alpha=0.3)

    # 5. Drops + deadzone filtered per send.
    ax = axes[plot_offset + 2]
    ax.bar(times_sec, dz_filtered, width=0.05, alpha=0.5, color="orange", label="Deadzone filtered")
    ax.bar(times_sec, drops, width=0.05, alpha=0.7, color="red", label="Drops before send")
    ax.set_ylabel("Count")
    ax.set_xlabel("Time (s)")
    ax.set_title("Per-Send: Drops & Deadzone Filtered Frames")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = Path(path).with_suffix(".png")
    plt.savefig(out_path, dpi=150)
    print(f"  Plot saved to: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
