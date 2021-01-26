# What to do if the plugin isn't working?

Make a build directory, and build the library. This should make a `libagl_tracker.so` in your plugin folder.

``` sh
cd lm-mit-momentum/plugin

# Build
mkdir build
cd build
cmake ../..
make -j8

# Rerun the setup script to copy the generated files to the proper places in PX4
cd ../..
sudo ./lm_setup.sh
```

When running with the plugin, use the command `make px4_sitl gazebo___terrain2d 2>&1 | tee team_name.txt` to launch PX4 and Gazebo and save the log prints to a log file. There will be another drone without a lidar. Disregard it. The drone you are working with will be the one spawned in the center of the terrain.
