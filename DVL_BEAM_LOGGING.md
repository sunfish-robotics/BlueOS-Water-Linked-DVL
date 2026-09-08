# DVL beam diagnostics in PX4

Each processed DVL velocity frame sends a MAVLink `DEBUG_FLOAT_ARRAY` named
`DVL_BEAMS`, array ID `48001`. PX4 publishes it on `debug_array`; it is not
an EKF input. The combined `DISTANCE_SENSOR` output is unchanged.

| Array index | Meaning |
| --- | --- |
| 0–3 | Raw slant range in metres for transducer IDs 0–3; no projection or median filtering |
| 4–7 | Corresponding `beam_valid`: 1 valid, 0 invalid, -1 missing |
| 8 | Overall `velocity_valid`: 1 valid, 0 invalid |
| 9–57 | Reserved, zero |

Missing/nonfinite ranges are encoded as -1. A finite range is retained even
when the beam is invalid. Nonfinite ranges are marked invalid. Diagnostics are
sent before the invalid-velocity early return. TCP backlog coalescing still
applies, so this is not guaranteed to capture every sensor ping.

The MAVLink timestamp carries DVL `time_of_transmission`, but the existing PX4
receiver replaces it with PX4 reception time in `debug_array.timestamp`.

Use `listener debug_array` in NSH. For ULog, enable bit 5 of `SDLOG_PROFILE`
while preserving its existing bits (default 1 becomes 33), then restart the
logger or reboot for the profile to take effect. This does not add the topic
to DDS/MCAP; that needs a separate bridge configuration.
