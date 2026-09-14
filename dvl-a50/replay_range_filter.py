"""Offline replay: uv run --with matplotlib --with mcap-ros2-support --with mcap \
--with loguru --with python3-nmap --with requests python dvl-a50/replay_range_filter.py [recording.mcap]

Loads the original driver from the pinned review baseline, never starts threads
or calls networking methods. Plots are pre-MAVLink-rate-limit filter outputs.
"""

import argparse
import json
import math
import subprocess
import types
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mcap_ros2.reader import read_ros2_messages

from dvl import DvlDriver

BASELINE = "20923bcb734e9957ff030bb9783e16757fe563ec"
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/range-smoothing"


def original_driver():
    source = subprocess.check_output(["git", "show", f"{BASELINE}:dvl-a50/dvl.py"], cwd=ROOT, text=True)
    module = types.ModuleType("original_dvl")
    exec(compile(source, "original_dvl.py", "exec"), module.__dict__)  # pylint: disable=exec-used
    return module.DvlDriver()


def frame(values):
    return {
        "velocity_valid": True,
        "transducers": [
            {"id": i, "distance": value / math.cos(math.radians(22.5)), "beam_valid": True}
            for i, value in enumerate(values)
        ],
    }


def compare(samples):
    original, proposed = original_driver(), DvlDriver()
    rows = []
    for timestamp, data in samples:
        before = after = None
        if data["velocity_valid"]:
            before = original.filtered_rangefinder_distance(data, timestamp)
            after = proposed.filtered_rangefinder_distance(data, timestamp)
        else:
            proposed.reset_rangefinder_filter()
        rows.append((timestamp, before, after))
    return rows


def plot(ax, rows, title):
    start = rows[0][0]
    for column, label in ((1, "Original selected"), (2, "Proposed")):
        ax.plot([r[0] - start for r in rows], [r[column] for r in rows], label=label, linewidth=1.2)
    invalid = [r[0] - start for r in rows if r[2] is None]
    ax.plot(invalid, [0.02] * len(invalid), "|", transform=ax.get_xaxis_transform(), color="red", label="No output")
    ax.set(title=title, xlabel="Time (s)", ylabel="Clearance (m)")
    ax.grid(alpha=0.25)
    ax.legend(loc="best", fontsize=8)


def synthetic():
    sequences = {
        "Synthetic: brief then sustained +30 cm, then shallower": lambda i: [2.3 if 20 <= i < 23 or 40 <= i < 90 else 2]
        * 4,
        "Synthetic: alternate shallow/deep every frame": lambda i: [2.3 if i >= 20 and i % 2 else 2] * 4,
        "Synthetic: wandering candidates, then invalid beam interval": lambda i: (
            [float("nan")] * 4 if 70 <= i < 75 else [2 + (0.3 + (i % 15) * 0.02 if i >= 20 else 0)] * 4
        ),
        "Synthetic: one beam at 8.7 m": lambda i: [2, 2, 2, 8.7 if i % 7 < 4 else 2],
    }
    fig, axes = plt.subplots(4, 1, figsize=(11, 11), constrained_layout=True)
    stats = {}
    for ax, (name, values) in zip(axes, sequences.items()):
        rows = compare((i / 10, frame(values(i))) for i in range(120))
        plot(ax, rows, name)
        if name.startswith("Synthetic: brief"):
            selected_step = next(t for t, before, _ in rows if t >= 4 and before > 2.1)
            first_response = next(t for t, _, after in rows if t >= 4 and after > 2)
            t63 = next(t for t, _, after in rows if t >= 4 and after >= 2 + 0.3 * (1 - math.exp(-1)))
            t95 = next(t for t, _, after in rows if t >= 4 and after >= 2.285)
            stats.update(
                selected_step_s=selected_step,
                first_response_s=first_response,
                acceptance_delay_s=first_response - selected_step,
                response_63_s=t63,
                response_95_s=t95,
                shallower_output_s=next(t for t, _, a in rows if t >= 9 and a == 2),
            )
    fig.savefig(OUT / "synthetic.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    noise = compare((i / 10, frame([2 + (0.02 if i % 2 else -0.02)] * 4)) for i in range(200))
    plot(axes[0], noise, "Synthetic: symmetric 2 cm noise (median can change phase)")
    ramp = compare((i / 10, frame([2 + max(0, i / 10 - 2) * 0.4] * 4)) for i in range(100))
    plot(axes[1], ramp, "Synthetic limitation: clearance increasing at 0.4 m/s")
    stats["ramp_final_selected_m"] = ramp[-1][1]
    stats["ramp_final_proposed_m"] = ramp[-1][2]
    fig.savefig(OUT / "noise-motion.png", dpi=150)
    plt.close(fig)
    return stats


def recording(path):
    samples = []
    last = -math.inf
    duplicates = 0
    for message in read_ros2_messages(path, topics=["/px4/dvl_beam_data"]):
        beam = message.ros_msg
        timestamp = beam.sensor_timestamp / 1e6
        if timestamp <= last:
            duplicates += 1
            continue
        last = timestamp
        data = {
            "velocity_valid": beam.velocity_valid,
            "transducers": [
                {"id": beam.beam_id[i], "distance": beam.range_m[i], "beam_valid": beam.beam_valid[i]}
                for i in range(4)
                if beam.beam_present[i]
            ],
        }
        samples.append((timestamp, data))
    rows = compare(samples)
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True)
    plot(axes[0], rows, "Recorded per-beam replay: entire segment")
    # Largest original consecutive jump with valid outputs: deterministic zoom.
    index = max(
        range(1, len(rows)),
        key=lambda i: (
            abs((rows[i][1] or 0) - (rows[i - 1][1] or 0))
            if rows[i][1] is not None and rows[i - 1][1] is not None
            else -1
        ),
    )
    center = rows[index][0]
    plot(
        axes[1], [r for r in rows if center - 5 <= r[0] <= center + 15], "Recorded replay: largest original jump window"
    )
    fig.savefig(OUT / "recording.png", dpi=150)
    plt.close(fig)
    valid = [r for r in rows if r[1] is not None and r[2] is not None]
    return {
        "path": str(path),
        "frames": len(rows),
        "duration_s": rows[-1][0] - rows[0][0],
        "non_increasing_skipped": duplicates,
        "original_no_output": sum(r[1] is None for r in rows),
        "proposed_no_output": sum(r[2] is None for r in rows),
        "max_selected_minus_proposed_m": max(r[1] - r[2] for r in valid),
        "max_step_original_m": max(abs(b[1] - a[1]) for a, b in zip(rows, rows[1:]) if a[1] and b[1]),
        "max_step_proposed_m": max(abs(b[2] - a[2]) for a, b in zip(rows, rows[1:]) if a[2] and b[2]),
        "max_increase_original_m": max(b[1] - a[1] for a, b in zip(rows, rows[1:]) if a[1] and b[1]),
        "max_increase_proposed_m": max(b[2] - a[2] for a, b in zip(rows, rows[1:]) if a[2] and b[2]),
        "gaps_over_400ms": sum(b[0] - a[0] > 0.4 for a, b in zip(rows, rows[1:])),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcap", nargs="?", type=Path)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    stats = {"baseline": BASELINE, "synthetic_10hz": synthetic()}
    if args.mcap:
        stats["recording"] = recording(args.mcap)
    (OUT / "metrics.json").write_text(json.dumps(stats, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()
