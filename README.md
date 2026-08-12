# BlueOS-Water-Linked-DVL

## Changelog

### v1.0.10
 - Improve DVL finding logic
 - Refactor settings file

### v1.0.7
  - Fix using lat/long inputs with no internet

### v1.0.6
 - No longer sets parameters automatically. Users can now change for two modes of operation:
     - DVL-only, the recommended mode
     - DVL+GPS, experimental mode which allows fusing (underwater) GPS and DVL data

### v1.0.5
 - Update texts to make support of DVL A125 obvious

### v1.0.4
 - Fix issue introduced in v1.0.3 where the extension was unable to talk to Cable-guy

### v1.0.3
 - Uses an random available port instead of 9001 to avoid conflict
 - Updated menu icon

### v1.0.2
 - Improved style

### v1.0.1
 - Fixed an issue where the driver was sending Rangefinder messages with invalid data

This is a docker implementation of a Water Linked DVL A50 and A125 driver as a BlueOS Extension.

The extension publishes valid DVL velocity samples as MAVLink `ODOMETRY` by
default. Existing installations can override this through their persisted
settings.

## PX4

Select `ODOMETRY` as the message type when using PX4. This sends DVL velocity in
the vehicle body FRD frame using MAVLink `ODOMETRY`; pose and angular velocity
are left unavailable. Configure PX4 with `EKF2_EV_CTRL=4` to fuse external-vision
3D velocity without enabling external-vision position or height fusion.

The legacy `SPEED_ESTIMATE` option is retained for ArduPilot. Do not use it for
PX4: `VISION_SPEED_ESTIMATE` has no field that can identify the DVL velocity as
body-frame data.

## Install

Install it from [BlueOS extensions tab](https://docs.bluerobotics.com/ardusub-zola/software/onboard/BlueOS-1.1/extensions/).

The service will show in the "Extension Manager" section in BlueOS, where there are some configuration options.
