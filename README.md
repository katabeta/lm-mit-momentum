# Getting Started <!-- omit in toc -->
<!-- TOC and section numbers automatically generated, do not manually edit -->
- [1. Install Ubuntu 18.04 LTS or 20.04 LTS](#1-install-ubuntu-1804-lts-or-2004-lts)
- [2. Install IDE - VS Code](#2-install-ide---vs-code)
- [3. Get Gazebo and PX4](#3-get-gazebo-and-px4)
- [4. Build PX4](#4-build-px4)
- [5. OPTIONAL Install QGroundControl](#5-optional-install-qgroundcontrol)
  - [5.1. Fix problem where PX4 running Gazebo can't connect to QGroundControl](#51-fix-problem-where-px4-running-gazebo-cant-connect-to-qgroundcontrol)
- [6. Install MAVSDK](#6-install-mavsdk)
- [7. Install py3gazebo](#7-install-py3gazebo)
  - [7.1. How to find sensor topic name and message type](#71-how-to-find-sensor-topic-name-and-message-type)
- [8. Adding LM-provided LiDAR and terrain](#8-adding-lm-provided-lidar-and-terrain)
  - [8.1. Creating your own terrain](#81-creating-your-own-terrain)
- [9. Launch simulation](#9-launch-simulation)
  - [9.1. Set home position](#91-set-home-position)
  - [9.2. Set firmware parameters](#92-set-firmware-parameters)
  - [9.3. Launch PX4 with Gazebo](#93-launch-px4-with-gazebo)
  - [9.4. OPTIONAL Launch PX4 with jMAVSim (instead of Gazebo)](#94-optional-launch-px4-with-jmavsim-instead-of-gazebo)
  - [9.5. OPTIONAL Launch QGroundControl](#95-optional-launch-qgroundcontrol)
- [10. Run mission file](#10-run-mission-file)
- [11. Query sensor values using py3gazebo - GPS Example](#11-query-sensor-values-using-py3gazebo---gps-example)
- [12. Getting started with MAVSDK](#12-getting-started-with-mavsdk)
<!-- TOC and section numbers automatically generated, do not manually edit -->

## 1. Install Ubuntu 18.04 LTS or 20.04 LTS

Basic steps to install Ubuntu are:

1. [Download an Ubuntu image](https://ubuntu.com/download/desktop)
2. [Verify image download is not corrupted](https://ubuntu.com/tutorials/how-to-verify-ubuntu#1-overview)
3. Make a bootable live USB in current OS
    - Make live USB in [Windows](https://ubuntu.com/tutorials/create-a-usb-stick-on-windows#1-overview)
    - Make live USB in [Ubuntu](https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview)
    - Make live USB in [Mac](https://ubuntu.com/tutorials/create-a-usb-stick-on-macos#1-overview)
4. [Boot from created USB](https://ubuntu.com/tutorials/install-ubuntu-desktop#4-boot-from-usb-flash-drive)
   - If your computer doesn't automatically boot from from the USB and `F12` does not work to invoke the boot menu, pay attention to the bootscreen for the key specific for your computer.
5. Choose a location to make a persistent image
    - [Install to dual boot with current OS](https://ubuntu.com/tutorials/install-ubuntu-desktop#6-allocate-drive-space) (**recommended**)
    - [Install with persistent storage onto a USB drive](https://www.techrepublic.com/article/how-to-create-a-live-usb-drive-with-persistent-storage-in-ubuntu/) (**advanced**)
    - [Install instead of current OS](https://ubuntu.com/tutorials/install-ubuntu-desktop#6-allocate-drive-space) (**not recommended**)

## 2. Install IDE - VS Code

Summary of [VS Code setup instructions](https://code.visualstudio.com/docs/setup/linux):

``` sh
# Install repository and signing key
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64 signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'

# Install VS Code
sudo apt-get install apt-transport-https
sudo apt-get update
sudo apt-get install code
```

## 3. Get Gazebo and PX4

Summary of [PX4 setup guide for ubuntu](https://dev.px4.io/master/en/setup/dev_env_linux_ubuntu.html):

``` sh
# Clone PX4
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

# Get into the PX4 project directory
cd PX4-Autopilot

# Run installer script
bash ./Tools/setup/ubuntu.sh

#
# Log out and log back in.
#
```

## 4. Build PX4

Summary of [PX4 simulation with Gazebo](https://dev.px4.io/master/en/simulation/gazebo.html):

``` sh
# Get into the PX4 project directory
cd /path/to/PX4-Autopilot

# Make the project with default vehicle target and gazebo simulation target
make px4_sitl gazebo
```

Take note:

- IP-address on the line that looks like `[Msg] Publicized address: 192.168.0.10` will be useful if PX4 can't connect to QGroundControl when running Gazebo.
- IP-address on the line that looks like `[Msg] Connected to gazebo master @ http://127.0.0.1:11345` will be useful when setting up py3gazebo.

## 5. OPTIONAL Install QGroundControl

Summary of [QGRoundControl installation instructions](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html):

``` sh
# Give yourself permissions to use the serial port
sudo usermod -a -G dialout $USER

# Remove modemmanager
sudo apt-get remove modemmanager -y

# Install dependencies
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y

#
# Log out and log back in.
#

# Download QGroundControl app image
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage

# Add execution permissions to the downloaded file
chmod +x ./QGroundControl.AppImage

# Launch app
./QGroundControl.AppImage # (or double-click)
```

### 5.1. Fix problem where PX4 running Gazebo can't connect to QGroundControl

Summary of [forum post solving this issue](https://discuss.px4.io/t/how-to-make-qgcontrol-connect-to-gazebo-simulation-instance-in-another-host-in-same-lan/9941):

``` sh
# Get into the PX4 project directory
cd /path/to/PX4-Autopilot

# Edit the startup script
gedit ROMFS/px4fmu_common/init.d-posix/rcS 
```

Add the IP address you see in the console when you run `make px4_sitl gazebo` to the line `mavlink start -x -u $udp_gcs_port_local -r 4000000` using the syntax

``` sh
-t 192.168.x.y
```

e.g.

``` sh
mavlink start -x -u $udp_gcs_port_local -r 4000000 -t 192.168.0.10
```

## 6. Install MAVSDK

Summary of [Python MAVSDK installation guide](https://github.com/mavlink/MAVSDK-Python#mavsdk-python):

``` sh
# Install MAVSDK Python library
pip3 install mavsdk
```

## 7. Install py3gazebo

``` sh
# Clone py3gazebo from GitHub and cd into the folder
git clone https://github.com/wil3/py3gazebo.git
cd py3gazebo

# Run 2to3 on the project
2to3 -w *.py

# Replace all instances of deprecated `asyncio.async` with `asyncio.ensure_future`
find ./ -type f -exec sed -i 's/asyncio.async/asyncio.ensure_future/g' {} \;

# Update proto definitions from the root of the py3gazebo project
GAZEBO_HOME=/usr/include/gazebo-11
protoc --proto_path=$GAZEBO_HOME/gazebo/msgs --python_out=pygazebo/msg $GAZEBO_HOME/gazebo/msgs/*proto
```

Replace these lines in `setup.py` with

``` python
# Line 15
readme = open('README.md').read()

# Line 42 and the rest of the packages definition
packages=[
    'pygazebo',
    'pygazebo.msg',
    'pygazebo.msg.v11',
    'pygazebo.msg.v9',
],
```

``` sh
# Install the Python library
sudo python3 setup.py install
```

### 7.1. How to find sensor topic name and message type

Should be `/gazebo/default/iris_lmlidar/lmlidar/link/laser/scan`

``` sh
gz topic -l
# ...
# `/gazebo/default/iris_lmlidar/lmlidar/link/laser/scan`
#...

gz topic -i /gazebo/default/iris_lmlidar/lmlidar/link/laser/scan
# Type: gazebo.msgs.LaserScanStamped
# 
# Publishers:
#   192.168.0.10:43455
# 
# Subscribers:
#   192.168.0.10:46127
```

## 8. Adding LM-provided LiDAR and terrain

Based on this [forum post](https://discuss.px4.io/t/create-custom-model-for-sitl/6700/2).

1. Download the LM provided assets from this repository and place parallel to your PX4 top folder
   - Your workspace directory should look like this:

        ``` tree
        .
         ├── LM-assets
         │   ├── ...
         ├── PX4
         │   ├── PX4-Autopilot
         │   ├── requirements.txt
         │   └── ubuntu.sh
         ├── py3gazebo
         │   └── ...
         └── ...
        ```

2. Run the bash script in the downloaded folder (`bash lm_setup.sh`)
   - If you see the following printed to the terminal, the script has succeded:

      ```sh
      gazebo_iris_lmlidar__terrain2d
      gazebo_iris_lmlidar__terrain3d
      ```

### 8.1. Creating your own terrain

You should **not** need to create your own terrain for this project, so this section will be brief.

This [video](https://vimeo.com/58409707) shows the steps.
In summary:

1. Create a `.png` in your favorite graphical editing software.
    - Make sure to remove the alpha channel
    - Make sure that the image is unsigned 8-bit
    - Make sure that the image is square
    - Make sure that the number of pixels per edge is $2^n+1$, where $n=0,1,2,3...$
    - White marks the tallest areas and black marks the shortest areas of your terrain
2. Add the `.png` you created to the `.world` file for your new terrain.
    - The `<size>` tag specifies how large your terrain actually is in simulation (x, y, z in meters)
3. Modify `cmake/sitl_target.cmake` with the name of your world
4. Add your terrain files to the script to place them in right places or do so manually

To create a terrain based on real world terrain, replace step 1 above with the following.

Summary of [Gazebo DEMs Tutorial](http://gazebosim.org/tutorials/?tut=dem):

1. Follow the steps below to create a terrain and world.

   ``` sh
   # Install dependencies for manipulating terrain data
   sudo apt-get install gdal-bin libgdal-dev
   pip3 install GDAL

   # Make and step into gazebo resources directory
   mkdir -p /tmp/gazebo
   cd /tmp/gazebo

   # Get terrain file and unzip
   wget https://github.com/osrf/gazebo_tutorials/raw/master/dem/files/mtsthelens_after.zip
   unzip ./mtsthelens_after.zip -d .

   # Rename file to something friendly
   mv ./10.2.1.1043901.dem ./mtsthelens.dem

   # Make a directory to store dem files
   mkdir -p media/dem

   # Downsample the terrain file and make sure it is the right format at the output
   gdalwarp -ts 129 129 /tmp/gazebo/mtsthelens.dem /tmp/gazebo/media/dem/mtsthelens_129.tiff
   gdal_translate -of USGSDEM /tmp/gazebo/media/dem/mtsthelens_129.tiff /tmp/gazebo/media/dem/mtsthelens_129.dem

   # Remove temporary files
   rm -f /tmp/gazebo/media/dem/mtsthelens_129.tiff /tmp/gazebo/media/dem/mtsthelens_129.dem.aux.xml /tmp/gazebo/mtsthelens.dem /tmp/gazebo/mtsthelens_after.zip

   # Get the world definition for the terrain
   wget https://github.com/osrf/gazebo_tutorials/raw/master/dem/files/volcano.world

   # Tell Gazebo where to look for the terrain file and launch to double check that it worked
   # Be sure of sourcing gazebo setup.sh in your own installation path
   source /usr/share/gazebo/setup.sh
   GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:/tmp/gazebo" gazebo /tmp/gazebo/volcano.world
   ```

2. Follow steps 2-4 from above.

## 9. Launch simulation

### 9.1. Set home position

``` sh
# Source home position
# https://dev.px4.io/master/en/simulation/gazebo.html#set-custom-takeoff-location
export PX4_HOME_LAT= 42.356219 # Deg
export PX4_HOME_LON=-71.100801 # Deg
export PX4_HOME_ALT= 9.35      # Meters
```

### 9.2. Set firmware parameters

``` sh
# Run from the shell (with pxh>)
# Set params manually
# https://dev.px4.io/master/en/advanced/parameter_reference.html
param set MPC_Z_VEL_MAX_DN 1.0 # m/s, max vertical velocity down
param set MPC_Z_VEL_MAX_UP 3.0 # m/s, max vertical velocity up
param set MPC_XY_VEL_MAX  12.0 # m/s, max horizontal velocity

# Or load from file (root location /PX4-Autopilot/build/px4_sitl_default/tmp/rootfs)
param load iris_defaults # Reset the parameters to when file was saved
param save # Optionally save params (not done automatically with load)

# Reset all params to default
param reset_all
```

### 9.3. Launch PX4 with Gazebo

Summary of [PX4 simulation using Gazebo](https://dev.px4.io/master/en/simulation/gazebo.html):

```sh
# Get into the PX4 project folder
cd /path/to/PX4-Autopilot

# Launch PX4 and Gazebo with the lidar and default terrain
make px4_sitl gazebo_iris_lmlidar

# OR Launch PX4 and Gazebo with the lidar and 2D terrain
make px4_sitl gazebo_iris_lmlidar__terrain2d

# OR Launch PX4 and Gazebo with the lidar and 3D terrain
make px4_sitl gazebo_iris_lmlidar__terrain3d
```

### 9.4. OPTIONAL Launch PX4 with jMAVSim (instead of Gazebo)

Summary of [PX4 simulation with jMAVSim](https://dev.px4.io/master/en/simulation/jmavsim.html):

``` sh
# Get into the PX4 project folder
cd /path/to/PX4-Autopilot

# Launch PX4 and jMAVSim
make px4_sitl jmavsim
```

### 9.5. OPTIONAL Launch QGroundControl

Summary of [QGRoundControl installation instructions](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html):

``` sh
# Launch QGroundControl
`./QGroundControl.AppImage` # (or double-click)
```

## 10. Run mission file

``` sh
# Get into the folder with the mission file
cd path/to/mission_file.py

# Run mission file
python3 mission_file.py
```

## 11. Query sensor values using py3gazebo - GPS Example

Get the available Gazebo topics and get the information on the topic of interest (take note of the message type). Gazebo and PX4 have to be running for this to work.

``` sh
gz topic -l
# ...
# '/gazebo/default/iris/gps0/link/gps'
# ...

gz topic -i /gazebo/default/iris/gps0/link/gps
# Type: gazebo.msgs.GPS
# 
# Publishers:
#   192.168.0.10:45247
# 
# Subscribers:
```

Create a message subscriber class with a message callback and a way to poll the data when needed. Get your Gazebo Master IP-Address and Port from the following message when launching PX4 `[Msg] Connected to gazebo master @ http://127.0.0.1:11345`.

``` python
import time # For the example only
import asyncio
import pygazebo

# What you import here depends on the message type you are subscribing to
import pygazebo.msg.v11.gps_pb2



# This is the gazebo master from PX4 message `[Msg] Connected to gazebo master @ http://127.0.0.1:11345`
HOST, PORT = "127.0.0.1", 11345


class GazeboMessageSubscriber: 

    def __init__(self, host, port, timeout=30):
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout

    async def connect(self):
        connected = False
        for i in range(self.timeout):
            try:
                self.manager = await pygazebo.connect((self.host, self.port))
                connected = True
                break
            except Exception as e:
                print(e)
            await asyncio.sleep(1)

        if connected: 
            # info from gz topic -l, gz topic -i arg goes here
            self.gps_subscriber = self.manager.subscribe('/gazebo/default/iris/gps0/link/gps', 'gazebo.msgs.GPS', self.gps_callback)

            await self.gps_subscriber.wait_for_connection()
            self.running = True
            while self.running:
                await asyncio.sleep(0.1)
        else:
            raise Exception("Timeout connecting to Gazebo.")

    def gps_callback(self, data):
        # What *_pb2 you use here depends on the message type you are subscribing to
        self.GPS = pygazebo.msg.v11.gps_pb2.GPS()
        self.GPS.ParseFromString(data)
    
    async def get_GPS(self):
        for i in range(self.timeout):
            try:
                return self.GPS
            except Exception as e:
                # print(e)
                pass
            await asyncio.sleep(1)
    

async def run():
    gz_sub = GazeboMessageSubscriber(HOST, PORT)
    asyncio.ensure_future(gz_sub.connect())

    # Simulate doing stuff and polling for the gps values only when needed
    start = time.time()
    current_time = 0
    last_time = 0
    while (current_time < 20):
        current_time = round(time.time() - start)
        if(current_time % 5 == 0 and last_time < current_time):
            gps_val = await gz_sub.get_GPS()
            print(gps_val)
            last_time = current_time
        if(current_time % 1  == 0 and last_time < current_time):
            print(current_time)
            last_time = current_time


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
```

## 12. Getting started with MAVSDK

Follow the [MAVSDK quickstart guide](https://mavsdk.mavlink.io/develop/en/python/quickstart.html) to get a headstart on using MAVSDK with your setup. The [API reference](http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/) is useful for understanding how the examples work and for figuring out how to write your own code.

Download MAVSDK examples

``` sh
# Install subversion if not already installed
sudo apt install subversion

# Download MAVSDK Python examples from the GitHub repository
svn checkout https://github.com/mavlink/MAVSDK-Python/trunk/examples
```
