# DVL beam diagnostics in PX4

Each processed DVL velocity frame sends a MAVLink `DEBUG_FLOAT_ARRAY` named
`DVL_BEAMS`, array ID `48001`. PX4 publishes it on `debug_array`; it is not
an EKF input. The combined `DISTANCE_SENSOR` output is unchanged.

| Array index | Meaning |
| --- | --- |
| 0–3 | Raw slant range in metres for transducer IDs 0–3; no projection or median filtering |
| 4–7 | Corresponding `beam_valid`: 1 valid, 0 invalid, -1 missing |
| 8 | Overall `velocity_valid`: 1 valid, 0 invalid |
| 9 | Mounting extension version: 1 when rotation supplied, otherwise 0 |
| 10–13 | Sensor-to-BODY_FRD quaternion, w,x,y,z, derived from configured mounting orientation |
| 14–57 | Reserved, zero |

Missing/nonfinite ranges are encoded as -1. A finite range is retained even
when the beam is invalid. Nonfinite ranges are marked invalid. Diagnostics are
sent before the invalid-velocity early return. TCP backlog coalescing still
applies, so this is not guaranteed to capture every sensor ping.

The MAVLink timestamp carries DVL `time_of_transmission`, but the existing PX4
receiver replaces it with PX4 reception time in `debug_array.timestamp`.

Use `listener debug_array` in NSH. For ULog, enable bit 5 of `SDLOG_PROFILE`
while preserving its existing bits (default 1 becomes 33), then restart the
logger or reboot for the profile to take effect. This does not add the topic
to DDS/MCAP on older firmware; that needs a separate bridge configuration.

The updated PX4 DDS branch also decodes this envelope into `dvl_beam_data`,
logs it in the default ULog profile, and publishes `/fmu/out/dvl_beam_data`.
DDS consumers must generate the matching `DvlBeamData` schema. Old senders
remain supported with `mounting_rotation_valid=false`.

The mounting quaternion describes rotation only. No mounting translation or
per-beam direction is inferred. A full body-to-DVL TF and individual beam TFs
require measured mounting offsets and a verified transducer direction map.
