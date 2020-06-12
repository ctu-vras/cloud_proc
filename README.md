# Utilities for working with poinclouds

## box_filter

Apply a cropbox to a pointcloud. Similar to CropBoxImproved, but less powerful.

Topics:
 - `in`: Incoming pointcloud.
 - `out`: The filtered pointcloud.

Parameters:
 - `box_frame`: Frame to which the pointcloud should be transformed before applying the cropping.
 - `target_frame`: Frame to which the cropped pointcloud should be transformed.
 - `fields`: Pointcloud fields which should be considered for cropping and transformation.
 - `min`, `max`: 3-tuples describing the cropping box in `box_frame` coordinates.
 - `tf_timeout`: Timeout for all lookuptransforms. If a transform is not found in this time, the cloud is dropped.
 - `max_age`: Maximum age of the point cloud. If an older cloud arrives, it is dropped.
 
## cloud_proc

Python implementation of a cloud filter chain.

Topics:
 - `in`: Incoming pointcloud.
 - `out`: The filtered pointcloud.

Parameters:
 - `out_queue_size`: Queue size for outgoing clouds.
 - `in_queue_size`: Queue size for incoming clouds.
 - `in_buff_size`: Buffer size for incoming clouds.
 - `overwrite_in_frame`: If nonempty, the frame of incoming clouds will be set to this value.
 - `filters`: List of filters to initialize. Example config:

```
- cloud_proc.DiscardOld:
    max_age: 10.0
- cloud_proc.Box:
    keep: 1
    lower: [-6.0, -6.0, -1.5]
    upper: [ 6.0,  6.0,  1.0]
    fields: ["x", "y", "z"]
    frame: "base_link"
    timeout: 1.0

```

Filters:

 - `Transform`: Transform cloud to another frame. Parameters:
   - `target_frame`: The frame to transform to.
   - `fields`: List of fields to which the 6-DOF transform should be applied.
   - `rotate`: List of fields to which only the rotation part of the transform should be applied (e.g. normals).
   - `timeout`: TF can_transform timeout
   - `update_frame`: Whether to update frame_id of the resulting cloud.
 - `Box`: Apply box filter to the cloud. Parameters:
   - `keep`: If True, points inside the box will be kept, otherwise points outside will be kept.
   - `lower`, `upper`: Corners of the cropping box.
   - `fields`, `frame`, `timeout`: If `frame` is set, a `Transform` filter will be pre-applied with these arguments.
 - `KeepFields`: Keep only the given fields. Parameters:
   - `fields`: The list of fields to keep.
 - `DiscardOld`: Discard pointclouds older than the given max age.
   - `max_age`: The maximum allowed age of incoming pointclouds.

## odom_cloud_transformer

Transforms point clouds by transforms published as odometry.

Topics:
 - `input`: Incoming pointcloud.
 - `odom`: The incoming odometry topic (matched to pointclouds using time synchronizers).
 - `output`: The filtered pointcloud.
 
Parameters:
 - `check_frames`: Make sure that the frames in the pointcloud and odometry message match.
 - `queue_size`: Time synchronizer queue size.
 - `slop`: Allowed difference between odometry and pointcloud time. If zero, exact match is required.
 - `fields`: List of fields to which the 6-DOF transform should be applied.
 - `rotate`: List of fields to which only the rotation part of the transform should be applied (e.g. normals).

## odom_tf

Transform odometry messages into TFs

Topics:
 - `odom`: Incoming odometry.
 - `odom_new`: The odometry after applying all options of this node (just for verification).
 - TF output.
 
Parameters:
 - `stamp`: If True, add current timestamp to the transform, otherwise use the one from the odom message.
 - `time_offset`: Add this offset to the timestamp.
 - `max_age`: Do not process odom messages older than this limit.
 - `odom_frame`: If nonempty, override the incoming odom parent frame to this value.
 - `robot_frame`: If nonempty, use TF to lookup transform between odom child frame and this frame, and publish the TF
   between odom parent and `robot_frame`.
 - `reuse_tf`: If `robot_frame` is set, this option specifies whether the transform between odom child and `robot_frame`
   will be read only once (as a static transform), or every time.

## tf_odom

Publish PoseStamped and Odometry messages that correspond to a given transform.

Topics:
 - TF input.
 - `odom`: The output odometry.
 - `pose`: The output pose.
 - `trigger`: Input message of any type with header. If used, the odometry will be published towards the same time as is
              in the messagë́'s header.

Parameters:
 - `parent_frame`: Parent frame for the TF lookup.
 - `child_frame`: Child frame for the TF lookup.
 - `timeout`: Timeout for the TF lookup.
 - `timer_freq`: If set, the node will publish odometry at the given frequency.
 - `no_wait`: If set, the TF lookup will look up just the latest available transform, not the one with the given timestamp.

## Improved PCL-based filters CropBoxImproved and PassThroughImproved

This is a bunch of C++ nodelets that extend the functionality of nodelets from `pcl_ros`.

### ImprovedFilter - common base class

This is the common base class for all the improved filters. In addition to the `pcl_ros` filters, they allow for periodic
publishing of the received pointclouds on frequencies different from the input pointcloud. They also provide very detailed
diagnostics about the received/published pointclouds.

The transformation procedures also take good care about non-default pointcloud fields like normals (`normal_x`, `normal_y`,
`normal_z`) and viewpoints (`vp_x`, `vp_y`, `vp_z`). Normals are only rotated, viewpoints are fully transformed

Topics:
 - `input`: Incoming pointcloud.
 - `indices`: If used, only the selected indices will be taken into account from the incoming pointcloud.
 - `output`: The filtered pointcloud.
 - `diagnostics`: For publishing the diagnostics output.
 
Parameters:
 - From `nodelet_topic_tools::NodeletLazy`:
   - `use_multithread_callback`: If true, use the MT nodehandle.
   - `lazy`: If true, use lazy subscription to input topics (only if `output` has subscribers).
   - `verbose_connection`: If true, print verbose info about (dis)connections.
   - `duration_to_warn_no_connection`: How long to wait before warning that there is no subscriber on `output`.
 - From `pcl_ros::PCLNodelet`:
   - `max_queue_size`: Queue size for both incoming and outgoing pointclouds.
   - `use_indices`: Whether to use indices published on `indices` topic or use whole pointclouds.
   - `latched_indices`: Whether to subscribe to `indices` as a latched topic.
   - `approximate_sync`: If listening for indices, synchronize them with pointclouds approximately (otherwise exact match is required).
 - From `pcl_ros::Filter`:
   - `input_frame`: If nonempty, the pointcloud will be transformed to this frame prior to any other actions on it.
   - `output_frame`: If nonempty, the pointcloud will be transformed to this frame just before publishing.
   - `keep_organized`: If true, organized pointclouds will be output as organized again (filtered values will be set to `NaN`).
 - Improved filter - periodic publishing:
   - `publish_periodically`: If true, the filter will publish pointclouds transformed at the current time on rate
     `publish_rate`. When a cloud comes in, it is converted to `input_frame` and stored. When the publication timer
     triggers, the stored cloud is converted to `input_frame` at current time using the given `fixed_frame`. Then the
     filter is applied, the pointcloud is converted to the `output_frame` and published.
   - `fixed_frame`: Only needed when `publish_periodically` is true. See the description above.
 - Improved filter - general:
   - `tf_wait_timeout`: Timeout for TF canTransform calls.
   - `produce_diagnostics`: Whether to compute and publish diagnostics of the publisher and subscriber.
   - `disable_orig_tf_listener`: If true and a shared TF2 buffer has been set to the nodelet before the `onInit()` call,
     the original TF listener from pcl_ros::PCLNodelet is disabled (by redirecting its tf topics to nonexistent topics).
     To launch the nodelet with a shared TF2 buffer, load it into `cras_cpp_common/nodelet_manager_sharing_tf_buffer`
     and not a normal nodelet manager. The shared buffer is accessible using the `getBuffer()` method.
 - Improved filter - transform arbitrary channels:
   - `transform_channels_point`: List of prefixes of channels to be transformed as points (6DOF). If channel consists of
     `vp_x`, `vp_y` and `vp_z`, pass only `vp_` as the parameter here.
   - `transform_channels_direction`: List of prefixes of channels to be transformed as vectors or directions (only
     rotation). If channel consists of `normal_x`, `normal_y` and `normal_z`, pass only `normal_` as the parameter here.
 - Improved filter - diagnostics:
   - There are two identical sets of settings, one for the subscriber (input pointclouds, param prefix `receive_`), and
     one for the publisher (output pointclouds, param prefix `publish_`). We describe them here generically with prefix '*'.
   - `diagnostic_period`: How often the diagnostics should be published. Only whole seconds are supported.
   - `*_rate`: The expected publishing/receiving rate. `publish_rate` is also used to configure the periodic publishing.
   - `*_rate_min`/`*_rate_max`: The minimum/maximum expected publishing/receiving rate. If `*_rate` is set,
     it is used as a default value for these two.
   - `*_rate_tolerance`: Real min rate is (`*_rate_min` * (1-tolerance)), real max rate is (`*_rate_max` * (1+tolerance)).
   - `*_rate_window_size`: For how many updates (called once in `diagnostic_period`) should the topic statistics be
     stored. If you set the window too small, the computed rates will have high variance. Also, consider setting this to
     a higher value when the expected rate of the topic is lower than 1.
   - `min_acceptable_*_delay`/`max_acceptable_*_delay`: Min/max allowed difference between current time and time in the
     processed message's header.
     
### PassThrough (ImprovedFilter)

Transform pointclouds between frames with the possibility to crop them in one dimension.

Additional parameters:
 - `filter_limit_min`/`filter_limit_max`: Min/max values for the filter.
 - `filter_field_name`: Name of the pointcloud field to filter by.
 - `filter_limit_negative`: If false, retain data inside the interval. If false, return the data that are outside.

### CropBox (ImprovedFilter)

Transform pointclouds between frames with the possibility to crop them in all three dimensions.

Additional parameters:
 - `min_x`/`min_y`/`min_z`/`max_x`/`max_y`/`max_z`/: Min/max corners of the crop box.
 - `negative`: If false, retain data inside the cropbox. If false, return the data that are outside.