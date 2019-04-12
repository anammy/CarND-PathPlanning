# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Project Introduction

The goal of this project to drive the around the track with no incidents and maintain a speed close to, but under the speed limit. 

### Objectives

- The car should maintain a speed close to the speed limit of 50 MPH. The traffic around the car is driving at +- 10 MPH of the 50 MPH speed limit. The car should pass slower moving traffic  when it is safe to do so.

- The car should not collide with any other cars and should drive within the lane markers, unless changing lanes.

- The car should be able to make one complete loop around the 6946m highway.

- The car should not exceed a total acceleration of 10 m/s^2 and jerk greater than 10 m/s^3. 

### Simulator

The simulator used in this project can be downloaded from [releases tab] (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

The map of the highway is in data/highway_map.txt. The car's localization and sensor fusion data is provided by the simulator. There is also a sparse map list of waypoints around the highway. Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

####

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path. With optimized code, it is usually just 1-3 time steps. During this delay the simulator will continue using points that it was last given.

## Code

The code consists of 3 main sections: Prediction, Behaviour Planning, and Trajectory Generation.

#### Prediction

In the prediction section, the sensor fusion data is used to check if there is a slow car ahead, a car in the left lane within 30 m, and/or a car in the right lane within 30 m. If any of these conditions are satisfied, the corresponding flags are set. If the other vehicles are ahead of our car, the s values of these other vehicles are stored.

#### Behaviour Planning

In the behaviour planning section, the flags from the prediction section are used to determine whether to stay in the same lane or perform a lane change when safe if there is a slow car ahead. The lane change is only performed if the forward progress achieved by the lane change is greater than 10m. Also, if both left and right lanes are available for a lane change, whichever lane has a car thats further ahead and thus, offers a greater forward progress is chosen for the lane change.

If a lane change is not possible, the target car's speed is lowered by 5 m/s increments until it is lower than the speed of the slow car ahead.

After the lane change has been made and the target car has passed the slower traffic, the target car merges back into the middle lane when its safe to do so. The car will then speed up close to the speed limit if there is no slow car ahead.

#### Trajectory Planning

The lane choice from the behaviour planning section is used to generate a new trajectory. The new path uses the last two points from the previous trajectory, if the previous path list is not empty, to ensure a smooth transition (i.e. no sudden accelerations or jerk) between different trajectories. The trajectory planning algorithm from the class is used to generate trajectories using the spline function.


## Results

The car is able to successfully drive around the track with no incidents. A clip of the car driving in the simulator can be found [here](https://github.com/anammy/CarND-PathPlanning/blob/master/video/PathPlanningClip.mp4).

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
