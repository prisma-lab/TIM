The hardware image pins `robotiq/ros` to
`8d7b8412ad685ffe1db5719da6e8fce6c1896e5e` and applies
`robotiq-feedback-health.patch` before compilation.

Upstream `read()` copies the last received device status even when the serial
connection is no longer operational. A fresh ROS timestamp can therefore carry
stale object detection. This patch publishes unknown object/fault values and an
invalid position during that condition. Hardware primitives reject these states
and request cancellation of active motion. It leaves the gripper hardware active;
returning a ros2_control hardware error could trigger deactivation, which resets
and releases this gripper. Recovery does not automatically resume a failed task.

The build checks the patch against the pinned revision. If replacing the driver,
preserve this feedback contract or implement an equivalent device-health check.
The vendor's upstream BSD license remains in the dependency checkout.
