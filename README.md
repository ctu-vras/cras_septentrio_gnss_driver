<!--
SPDX-License-Identifier: BSD-3-Clause
SPDX-FileCopyrightText: Czech Technical University in Prague
-->

# cras_septentrio_gnss_driver

The septentrio_gnss_driver publishes various DO-NOT-USE values in its output, which are usually -2*10^10.
Such values are unsuitable for downstream ROS processing. This node filters these values out and replaces them with
more suitable values, or throws away whole messages if they would be invalid from the ROS point of view.

It also creates alternative message types from the attitude message: compass_msgs/Azimuth (for generic azimuth
processing nodes) and sensor_msgs/Imu (for robot_localization). The IMU message contains only ENU heading (and maybe
pitch), but the third angle is always reported as zero. Covariance of the invalid angles is set to (2*pi)^2. Also,
the angular velocities are added to this IMU message. Accelerations are not used.

## Udev rules

To ensure stable configuration of the unit that doesn't change every boot, install the included udev rule
`80-cras-septentrio-gps.rules` into `/etc/udev/rules.d` . This rule, however, only supports one unit connected via
USB at a time, so remember that if you would work with multiple units.

## Nodelets

### cras_septentrio_gnss_driver/process

(also available as node `cras_gnss_septentrio_driver/process`)

#### Subscribed topics

Everything published from Septentrio driver, expected in sub-namespace `raw`. Eg. `raw/atteuler`.

#### Published topics

The fixed topics in the empty namespace, e.g. `atteuler`. In addition to them, these other topics are published:

- `azimuth` (`compass_msgs/Azimuth`): The ENU geographic azimuth in radians.
- `azimuth_imu` (`sensor_msgs/Imu`): IMU message with filled orientation containing heading and pitch in ENU.

#### Parameters

- `~max_position_error` (double, default 40000000 m): Maximum error in cartesian position used instead of DO-NOT-USE values. Default is 40 000 km, Earth circumference.
- `~max_altitude_error` (double, default 100000 m): Maximum error in altitude used instead of DO-NOT-USE values. Default is 100 km.
- `~max_orientation_error` (double, default 2*pi rad): Maximum error in orientation angles used instead of DO-NOT-USE values.
- `~max_horizontal_velocity_error` (double, default 10 m/s): Maximum error in horizontal velocity used instead of DO-NOT-USE values.
- `~max_vertical_velocity_error` (double, default 10 m/s): Maximum error in vertical velocity used instead of DO-NOT-USE values.
- `~max_clock_bias_error` (double, default 10 m/s): Maximum error in clock bias converted to velocity error used instead of DO-NOT-USE values.
- `~valid_heading_error_threshold_deg` (double, default 5 deg): Heading measurements with larger error will be discarded.
- `~publish_invalid_fix` (bool, default true): If true, even invalid fix messages will be published. The reader must check the `status.status` field for `NO_FIX` value to figure out validity of the message.
- `~publish_invalid_heading` (bool, default false): Whether to publish invalid heading messages (with NaNs or high covariance).