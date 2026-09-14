"""Compare actual driver publication paths using downstream MCAP beam arrivals.

Run with uv run --no-project --with mcap --with mcap-ros2-support --with loguru
--with python3-nmap --with requests python dvl-a50/replay_range_publication.py
recording.mcap [--duration 287] [--output metrics.json]. No networking is performed.
"""

import argparse
import json
import subprocess
import types
from pathlib import Path
from unittest.mock import Mock, patch

from mcap_ros2.reader import read_ros2_messages

import dvl

BASELINE = "e0280f7"
ROOT = Path(__file__).resolve().parents[1]


def baseline_module():
    source = subprocess.check_output(["git", "show", f"{BASELINE}:dvl-a50/dvl.py"], cwd=ROOT, text=True)
    filter_source = subprocess.check_output(["git", "show", f"{BASELINE}:dvl-a50/range_filter.py"], cwd=ROOT, text=True)
    original_filter = types.ModuleType("range_filter")
    exec(  # pylint: disable=exec-used
        compile(filter_source, "baseline_range_filter.py", "exec"), original_filter.__dict__
    )
    module = types.ModuleType("baseline_dvl")
    with patch.dict("sys.modules", {"range_filter": original_filter}):
        exec(compile(source, "baseline_dvl.py", "exec"), module.__dict__)  # pylint: disable=exec-used
    return module


def replay(module, samples):
    driver = module.DvlDriver()
    driver.mav = Mock()
    sent = []
    for arrival, data in samples:
        count = driver.mav.send_rangefinder.call_count
        with patch.object(module.time, "time", return_value=arrival), patch.object(
            module.time, "monotonic", return_value=arrival
        ):
            driver.handle_velocity(data)
        if driver.mav.send_rangefinder.call_count > count:
            sent.append(arrival)
    intervals = [b - a for a, b in zip(sent, sent[1:])]
    # Include the terminal period without output, but exclude initial filter warmup.
    silence = intervals + ([samples[-1][0] - sent[-1]] if sent else [])
    return {
        "range_messages": len(sent),
        "gaps_over_400ms": sum(dt > 0.4 for dt in intervals),
        "max_gap_s": max(intervals, default=None),
        "seconds_over_400ms_since_output": sum(max(0, dt - 0.4) for dt in silence),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcap", type=Path)
    parser.add_argument("--duration", type=float, default=float("inf"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    samples = []
    for message in read_ros2_messages(args.mcap, topics=["/px4/dvl_beam_data"]):
        beam = message.ros_msg
        arrival = beam.timestamp / 1e6
        if samples and arrival - samples[0][0] > args.duration:
            break
        samples.append(
            (
                arrival,
                {
                    "vx": 0,
                    "vy": 0,
                    "vz": 0,
                    "fom": 0,
                    "time": 100,
                    "velocity_valid": beam.velocity_valid,
                    "time_of_transmission": beam.sensor_timestamp,
                    "transducers": [
                        {"id": beam.beam_id[i], "distance": beam.range_m[i], "beam_valid": beam.beam_valid[i]}
                        for i in range(4)
                        if beam.beam_present[i]
                    ],
                },
            )
        )
    if not samples:
        parser.error("No DVL beam samples found in recording")
    original = baseline_module()
    result = {
        "baseline": subprocess.check_output(["git", "rev-parse", BASELINE], cwd=ROOT, text=True).strip(),
        "recording": str(args.mcap),
        "frames": len(samples),
        "duration_s": samples[-1][0] - samples[0][0],
        "baseline_output": replay(original, samples),
        "proposed_output": replay(dvl, samples),
        "limitation": "PX4 beam receipt times approximate bridge ingress; this is not an EKF or mission replay.",
    }
    payload = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")


if __name__ == "__main__":
    main()
