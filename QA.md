# Questions and Answers <!-- omit in toc -->

The following is a constantly updating question and answer page with common questions posed by students regarding this project. Please check if your question is answer here before seeking other solutions.

## Table of Contents <!-- omit in toc -->
<!-- TOC and section numbers automatically generated, do not manually edit -->
- [1. How do I make a mission?](#1-how-do-i-make-a-mission)
- [2. How do I read LiDAR returns from code?](#2-how-do-i-read-lidar-returns-from-code)
- [3. How do I tell the drone to go in a specific direction?](#3-how-do-i-tell-the-drone-to-go-in-a-specific-direction)
- [4. What is the terrain like?](#4-what-is-the-terrain-like)
- [5. What are the kinematic capabilities of the drone we are flying?](#5-what-are-the-kinematic-capabilities-of-the-drone-we-are-flying)
- [6. What is the field of view (FoV) of the LiDAR?](#6-what-is-the-field-of-view-fov-of-the-lidar)
- [7. What is contour flight?](#7-what-is-contour-flight)
<!-- TOC and section numbers automatically generated, do not manually edit -->

## 1. How do I make a mission?

The objective for you is to write an altitude selection algorithm, while the rest of the mission logistics are handled for you.

The mission is written using Python and asyncronous calls to the [MAVSDK library](http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/). Missions are constructed as API calls for the drone to start up, takeoff, complete mission, and land. For examples, check out the [MAVSDK examples](https://github.com/mavlink/MAVSDK-Python/tree/main/examples).

## 2. How do I read LiDAR returns from code?

Gazebo is the software that handles the sensor simulation and is the software that publishes the returns. The internal structure of the communication is implemented using [Protocol Buffers](https://developers.google.com/protocol-buffers) and you can listen for the messages using external software. This is what [Py3Gazebo](https://github.com/katabeta/lm-mit-momentum#8-install-py3gazebo) is for in our case.

If you have successfully installed Py3Gazebo, you should be able to run the file [demos/demo_lidar_read.py](https://github.com/katabeta/lm-mit-momentum/blob/master/demos/demo_lidar_read.py), which will print out LiDAR readings every 5 seconds for 20 seconds total. You must have PX4 and Gazebo running with the LiDAR model enabled.

## 3. How do I tell the drone to go in a specific direction?

You can tell the drone to go to a specific location by appending a mission item with the desired location to the mission and uploading the mission to the drone (as in the [mission example](https://github.com/mavlink/MAVSDK-Python/blob/main/examples/mission.py)), or you can use the `drone.action.goto_location` command (as in the [goto example](https://github.com/mavlink/MAVSDK-Python/blob/main/examples/goto.py)).

Specifying locations, as opposed to directions of flight, is the preferred way of commanding the drone for this competition.

For more examples, check out the [MAVSDK examples](https://github.com/mavlink/MAVSDK-Python/tree/main/examples).

## 4. What is the terrain like?

The competition terrain is a wavy structure with dimensions 40x40 m and a maximum height of 5 m. The terrain is positioned at 0 deg N, 0 deg W, at -5 m altitude.

The terrain definition is located in [worlds/models/terrain2d/model.sdf](https://github.com/katabeta/lm-mit-momentum/blob/master/worlds/models/terrain2d/model.sdf) if you seek further information.

## 5. What are the kinematic capabilities of the drone we are flying?

The drone has a [default set of parameters with default values](https://dev.px4.io/master/en/advanced/parameter_reference.html).

The following are pertinent to this competition:

- Max horizontal velocity: 12 m/s
- Max ascent velocity: 3 m/s
- Max descent velocity: 1 m/s

## 6. What is the field of view (FoV) of the LiDAR?

This LiDAR's FoV is defined as

- Horizontal: -30 to 30 degs, 3 deg increments, 0 degs points directly in front of vehicle
- Vertical: -90 to 0 degs, 10 deg increments, 0 degs points directly in front of vehicle, -90 degs points directly down
- Range: 0.2 m to 10 m, with infinity indicating an unreturned ray

For further information, check out the LiDAR definition in the [models/lidar/model.sdf](https://github.com/katabeta/lm-mit-momentum/blob/master/models/lmlidar/model.sdf) file, within the `<ray>` element.

## 7. What is contour flight?

Countour flight is a type of terrain flight, where the aircraft is flying near terrain in order perform surveillance, survey, avoid detection or other reasons. In addition to contour flight, terrain flight also includes low-level flight, and Nap of the Earth (NOE) flight. The following is a brief explanation of each type, in decreasing order of distance to the ground and increasing order of pilot workload.

- Low-level flight: flight low to the ground, typically between 500 ft and 1000 ft in unpopulated areas, where the aircraft is holding a steady airspeed and altitude
- Contour flight: constant airspeed flight that flies close to the ground, typically between 150 ft and 500 ft, and varies altitude to match the terrain
- Nap of the Earth: flight that attempts to stay extremely close to the ground, typically <= 150 ft, by varying both airspeed and altitude to match the terrain relief

Your goal is to implement contour flight, so you will have to maneuver the drone to follow the terrain relief, but maintain a constant airspeed.

For further reading on terrain flight and why it's an important topic in autonomy, reference this [article](https://ianrp.org/terrain-flight/) by the International Association of Natural Resource Pilots.
