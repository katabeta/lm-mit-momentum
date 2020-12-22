# How to make your own terrain <!-- omit in toc -->
<!-- TOC and section numbers automatically generated, do not manually edit -->
- [1. Create a terrain based on a heightmap](#1-create-a-terrain-based-on-a-heightmap)
- [2. Create a terrain based on real world terrain](#2-create-a-terrain-based-on-real-world-terrain)
<!-- TOC and section numbers automatically generated, do not manually edit -->

You should **not** need to create your own terrain for this project, so this section will be brief.

## 1. Create a terrain based on a heightmap

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

## 2. Create a terrain based on real world terrain

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