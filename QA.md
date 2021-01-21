# Questions and Answers <!-- omit in toc -->

The following is a constantly updating question and answer page with common questions posed by students regarding this project. Please check if your question is answered here before seeking other solutions.

## Table of Contents <!-- omit in toc -->
<!-- TOC and section numbers automatically generated, do not manually edit -->
- [1. How do I make a mission?](#1-how-do-i-make-a-mission)
- [2. How do I read LiDAR returns from code?](#2-how-do-i-read-lidar-returns-from-code)
- [3. How do I tell the drone to go in a specific direction?](#3-how-do-i-tell-the-drone-to-go-in-a-specific-direction)
- [4. What is the terrain like?](#4-what-is-the-terrain-like)
- [5. What are the kinematic capabilities of the drone we are flying?](#5-what-are-the-kinematic-capabilities-of-the-drone-we-are-flying)
- [6. What is the field of view (FoV) of the LiDAR?](#6-what-is-the-field-of-view-fov-of-the-lidar)
- [7. What is contour flight?](#7-what-is-contour-flight)
- [8. How will my team be scored?](#8-how-will-my-team-be-scored)
- [9. How are LiDAR ranges arranged in the LaserScan topic?](#9-how-are-lidar-ranges-arranged-in-the-laserscan-topic)
- [10. What are the intensity returns from the LiDAR?](#10-what-are-the-intensity-returns-from-the-lidar)
<!-- TOC and section numbers automatically generated, do not manually edit -->

## 1. How do I make a mission?

The objective for you is to write an altitude selection algorithm, while the rest of the mission logistics are handled for you. To get started, use the [template mission using the mission construct](https://github.com/katabeta/lm-mit-momentum/blob/master/template_mission.py) or the [template mission using the goto_location construct](https://github.com/katabeta/lm-mit-momentum/blob/master/template_goto.py)

If you already have something that's working or would prefer to work something out from scratch, you are welcome to continue using that as it will help with your creativity score.

The mission is written using Python and asyncronous calls to the [MAVSDK library](http://mavsdk-python-docs.s3-website.eu-central-1.amazonaws.com/). Missions are constructed as API calls for the drone to start up, takeoff, complete mission, and land. For examples, check out the [MAVSDK examples](https://github.com/mavlink/MAVSDK-Python/tree/main/examples).

## 2. How do I read LiDAR returns from code?

Gazebo is the software that handles the sensor simulation and is the software that publishes the returns. The internal structure of the communication is implemented using [Protocol Buffers](https://developers.google.com/protocol-buffers) and you can listen for the messages using external software. This is what [Py3Gazebo](https://github.com/katabeta/lm-mit-momentum#8-install-py3gazebo) is for in our case.

If you have successfully installed Py3Gazebo, you should be able to run the file [tutorial/demos/demo_lidar_read.py](https://github.com/katabeta/lm-mit-momentum/blob/master/tutorial/demos/demo_lidar_read.py), which will print out LiDAR readings every 5 seconds for 20 seconds total. You must have PX4 and Gazebo running with the LiDAR model enabled.

To test your knowledge and understanding, please read along the [LaserScan tutorial](https://github.com/katabeta/lm-mit-momentum/blob/master/tutorial/tutorial_lidar.py) and help make it complete. By the end of the tutorial you should be more familiar with:

- Checking and acquiring more information on the published data from Gazebo
- Looking at a "snapshot" from the published data from Gazebo
- The structure of the 'LaserScan' topic
- Iterating through the known output of our 'LaserScan' topic

## 3. How do I tell the drone to go in a specific direction?

You can tell the drone to go to a specific *location* by appending a mission item with the desired location to the mission and uploading the mission to the drone (as in the [mission example](https://github.com/mavlink/MAVSDK-Python/blob/main/examples/mission.py)), or you can use the `drone.action.goto_location` command (as in the [goto example](https://github.com/mavlink/MAVSDK-Python/blob/main/examples/goto.py)). Note that the `goto_location` command does not block while executing and you will not know from code when you have reached the waypoint unless you also monitor your position.

Specifying locations, as opposed to directions of flight, is the preferred way of commanding the drone for this competition.

For more examples, check out the [MAVSDK examples](https://github.com/mavlink/MAVSDK-Python/tree/main/examples).

## 4. What is the terrain like?

The competition terrain is a wavy structure with active area dimensions 40x40 m and a maximum height of 5 m. The terrain is positioned at 0 deg N, 0 deg W, at 0 m altitude. If you don't find that's the case for you, make sure you run `source set_home.sh` in the terminal before launching Gazebo and PX4.

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

- Low-level flight: flight low to the ground where the aircraft is holding a steady airspeed and altitude
- Contour flight: constant airspeed flight that flies close to the ground and varies altitude to match the terrain
- Nap of the Earth: flight that attempts to stay extremely close to the ground by varying both airspeed and altitude to match the terrain relief

Your goal is to implement contour flight, so you will have to maneuver the drone to follow the terrain relief, but try to maintain a constant airspeed.

The altitudes at which these modes of flight are performed is dependent on vehicle capabilities, weather conditions, pilot workload, and any number of other conditions. For the purposes of this competition, you will perform your contour flight as close to the terrain as possible while not colliding with it.

For further reading on terrain flight, reference this [article](https://ianrp.org/terrain-flight/) by the International Association of Natural Resource Pilots.

## 8. How will my team be scored?

Teams will be ranked based on their performance in three areas: time, distance-average of AGL, and a judge's score. The team's competition score will be a sum of all of these ranks and as such the maximum score a team can get is $3 * (number\ of\ teams)$.

The judges score will consist of (but is not limited to):

- Creativity
- Presentation
- Customer satisfaction (e.g. reliability, testability, etc.)

The only metric that will be scored live is the judges score. For the other two, the teams will submit their times and averages as reported by a forthcoming tool. The location to enter times is TBD.

Teams will also be required to create a video showcasing their strategy and results. This video will be shown during the team's presentation. The teams are free to structure their presentation as they see fit, but we recommend that teams use this reel to aid the team in showcasing their strategy and in talking about their development process. As such, this reel does not have to show the entire flight. This video will be uploaded to YouTube and the link will be submitted with the times and averages.

## 9. How are LiDAR ranges arranged in the LaserScan topic?

The LiDAR ranges are returned in a flattened 1D array that has the 20 horizontal ranges superimposed on each other (`[v1h1 .. v20h1 v1h2 .. v20h2 v1h3 .. v20h3 ...]`). The returns are ordered minimum to maximum based on the FoV ranges reported in the LaserScan message.

Teams should experiment reading the LiDAR while Gazebo is running to convince themselves that they understand the order of the returned ranges.

## 10. What are the intensity returns from the LiDAR?

A typical LiDAR returns an intensity value, which is a measure of the intensity of the returned beam. This means that this return measures the reflectivity of the surface to the wavelength of the beams cast by the LiDAR.

This value is irrelevant for the purposes of this competition.
