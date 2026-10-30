# gnss_poser

## Overview

The `gnss_poser` is a node that subscribes gnss sensing messages and calculates vehicle pose with covariance.

## Design

This node subscribes to NavSatFix to publish the pose of **base_link**. The data in NavSatFix represents the antenna's position. Therefore, it performs a coordinate transformation using the tf from `base_link` to the antenna's position. The frame_id of the antenna's position refers to NavSatFix's `header.frame_id`.
(**Note that `header.frame_id` in NavSatFix indicates the antenna's frame_id, not the Earth or reference ellipsoid.** [See also NavSatFix definition.](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html))

The antenna is rigidly mounted on the vehicle, so this transformation is expected to be static; an antenna that moves relative to `base_link` is out of scope. Publishing it on `/tf_static` is recommended, but a setup that publishes the same fixed relation on `/tf` is supported as well: the node looks up the latest transform rather than the transform at the fix's `header.stamp`, which `/tf` cannot always provide.

If the transformation from `base_link` to the antenna cannot be obtained, the fix is skipped: `gnss_fixed` still reports the receiver's status, but no pose is published. This is expected right after startup, before the transformation has been received, so while TF does not know the antenna frame the fixes are skipped silently for up to 5 seconds. After that, or at once when TF knows the frame but cannot connect it to `base_link`, the node logs an error (throttled to once a second) naming both frames, so that a `header.frame_id` that never gets a transformation is visible instead of silently yielding no pose.

### Orientation

NavSatFix carries no orientation. `orientation_source` selects where the orientation of the pose comes from:

| Value                      | Orientation                                                                                      | Rotation covariance (roll, pitch, yaw)      |
| -------------------------- | ------------------------------------------------------------------------------------------------ | ------------------------------------------- |
| `ins`                      | the latest `autoware_orientation`. No pose is published until the first one has arrived.         | its `rmse_rotation_*` squared               |
| `motion`                   | the heading of the displacement from the previous position to the current one (roll = pitch = 0) | 0.1, 0.1, 1.0 [rad^2]                       |
| `none`                     | unknown: the identity, that is heading east in the map frame                                     | 1.0, 1.0, 1.0 [rad^2]                       |
| `ins_or_none` (deprecated) | as `ins` once `autoware_orientation` has arrived, as `none` until then                           | as `ins`, or as `none` before the first one |

`orientation_source` replaces `use_gnss_ins_orientation`, which is deprecated. A configuration that sets only `use_gnss_ins_orientation` still works and gets a warning at startup: `true` is read as `ins_or_none` (what it did) and `false` as `motion`. When both are set, `use_gnss_ins_orientation` is ignored. `ins_or_none` is deprecated as well, because a pose with the stand-in orientation cannot be told apart from one with a measured orientation: use `ins` when `autoware_orientation` is published and `none` otherwise.

The node logs the source in use at startup, and warns once when `autoware_orientation` arrives while the source is `none`, since it is then ignored.

## Inputs / Outputs

### Input

| Name                           | Type                                                    | Description                                                                                                                    |
| ------------------------------ | ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `/map/map_projector_info`      | `autoware_map_msgs::msg::MapProjectorInfo`              | map projection info                                                                                                            |
| `~/input/fix`                  | `sensor_msgs::msg::NavSatFix`                           | gnss status message                                                                                                            |
| `~/input/autoware_orientation` | `autoware_sensing_msgs::msg::GnssInsOrientationStamped` | orientation [click here for more details](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_sensing_msgs) |

### Output

| Name                     | Type                                             | Description                                                    |
| ------------------------ | ------------------------------------------------ | -------------------------------------------------------------- |
| `~/output/pose`          | `geometry_msgs::msg::PoseStamped`                | vehicle pose calculated from gnss sensing data                 |
| `~/output/gnss_pose_cov` | `geometry_msgs::msg::PoseWithCovarianceStamped`  | vehicle pose with covariance calculated from gnss sensing data |
| `~/output/gnss_fixed`    | `autoware_internal_debug_msgs::msg::BoolStamped` | gnss fix status                                                |

## Parameters

Parameters in below table

| Name                       | Type      | Default          | Description                                                                                                                                 |
| -------------------------- | --------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| `base_frame`               | `string`  | `base_link`      | frame id for base_frame                                                                                                                     |
| `gnss_base_frame`          | `string`  | `gnss_base_link` | frame id for gnss_base_frame                                                                                                                |
| `map_frame`                | `string`  | `map`            | frame id for map_frame                                                                                                                      |
| `orientation_source`       | `string`  | `none`           | `ins`, `motion`, `none` or `ins_or_none` (deprecated); see [Orientation](#orientation). Any other value is rejected at startup.             |
| `use_gnss_ins_orientation` | `boolean` | not set          | Deprecated: set `orientation_source` instead. Read only when `orientation_source` is not set: `true` as `ins_or_none`, `false` as `motion`. |
| `gnss_pose_pub_method`     | `integer` | `0`              | 0: Instant Value 1: Average Value 2: Median Value. Any other value is rejected at startup.                                                  |
| `buff_epoch`               | `integer` | `1`              | Number of positions the average / median is taken over (ignored for method 0). Range: 1~inf; smaller values are rejected at startup.        |

All above parameters can be changed in config file [gnss_poser.param.yaml](./config/gnss_poser.param.yaml "Click here to open config file") .
