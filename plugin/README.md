# How to get the plugin working

***These are notes for LM staff, students will have a simplified version of these instructions later in the evening.***

Regenerate protos, make a build directory, and build the library. This should make a `libagl_tracker.so` in your plugin folder.

``` sh
# Regenerate protos
export GAZEBO_HOME=/usr/include/gazebo-11
protoc -I=. -I=$GAZEBO_HOME/gazebo/msgs/proto --cpp_out=. agl_debug.proto

# Build
mkdir build
cd build
cmake ../..
make -j8

# Rerun the setup script to copy the generated files to the proper places in PX4
cd ../..
sudo ./lm_setup.sh
```

When running with the plugin, use the command `make px4_sitl gazebo___terrain2d` to launch PX4 and Gazebo. There will be another drone without a lidar. Disregard it. The drone you are working with will be the one spawned in the center of the terrain.


