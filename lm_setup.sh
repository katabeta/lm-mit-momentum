#!/bin/bash
# Copy world assets
cp -f worlds/*.world ../PX4/PX4-Autopilot/Tools/sitl_gazebo/worlds/
su -c 'mkdir -p ~/.gazebo/models' $SUDO_USER # Make sure that the models folder exists before trying to copy into it
su -c 'cp -af ./worlds/models/terrain3d/ ~/.gazebo/models' $SUDO_USER # Make sure that the models folder exists before trying to copy into it
su -c 'cp -af ./worlds/models/terrain2d/ ~/.gazebo/models' $SUDO_USER # Make sure that the models folder exists before trying to copy into it

# Copy vehicle and sensor assets
cp -af models/. ../PX4/PX4-Autopilot/Tools/sitl_gazebo/models/
cp ./ROMFS/1043_iris_lmlidar ../PX4/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
cp ./ROMFS/1043_iris_lmlidar ../PX4/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/

# Copy cmake file
cp -f cmake/sitl_target.cmake ../PX4/PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake

# Copy py3gazebo setup file
cp -f py3gazebo_setup/setup.py ../py3gazebo/setup.py
cp -f py3gazebo_setup/pygazebo.py ../py3gazebo/pygazebo/pygazebo.py

# Copy other helpful files
cp scritps/set_home.sh ../PX4/PX4-Autopilot/
cp px4_param_defaults/iris_defaults ../PX4/PX4-Autopilot/build/px4_sitl_default/tmp/rootfs/

# Change to the PX4 directory
cd ../PX4/PX4-Autopilot/
mkdir -p build/px4_sitl_default/build_gazebo

# Build and check that everything worked
make px4_sitl list_vmd_make_targets | grep -o gazebo_iris_lmlidar__terrain2d
make px4_sitl list_vmd_make_targets | grep -o gazebo_iris_lmlidar__terrain3d

# Copy the AGL Tracker plugin so Gazebo can use it
# Requires the build directory in PX4 to be populated -- error otherwise
cp ../../lm-mit-momentum/plugin/libagl_tracker.so build/px4_sitl_default/build_gazebo
