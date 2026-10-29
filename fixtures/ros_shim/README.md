# ros_shim

Just enough of ROS's headers for FAST-LIO's ROS-coupled sources, such as `src/IMU_Processing.hpp`, to compile inside the fixtures image.

The shim provides plain message structs with the fields FAST-LIO reads, a `ros::Time` that stores seconds, and logging macros that print to stderr. It has no transport, no node and no publishers. FAST-LIO's algorithm code compiles against it unchanged.
