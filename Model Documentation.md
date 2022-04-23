
## Model Documentation - Highway Driving
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goal was to design a path planner that would be able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. A path planner is considered to be successful if it is able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data.

This write-up includes the rubric points as well as my detailed description of how I addressed each point. This includes a detailed description of the code and links to supporting documents and/or external references.


### [Rubric Points](https://review.udacity.com/#!/rubrics/1971/view)

### 1. Compilation
I successfully compiled the code as recommended:
-   `mkdir build && cd build` to create and enter the build directory
-   `cmake .. && make` to compile the project
-   `./path_planning` to run the code

My implementation uses `spline.h`, an implementation of Cubic Spline interpolation functions provided by [kluge.in-chemnitz.de](https://kluge.in-chemnitz.de/opensource/spline/). A copy of the header file is part of my submission.

### 2. Valid Trajectory Generation
The path planner has to generate a safe and drivable path for a vehicle to get to a desired goal. This path has to be updated/adapted as soon as environmental changes prohibit safe driving. <br>
The project *Highway Driving* simulates a three lane highway environment with traffic. Cars move at ~50 mph, they may change lanes and may accelerate. The movements for generated paths in this environment should:
- never exceed a 50 mph speed limit,
- be performed with jerk and total acceleration $< 10 \frac{m}{s^2}$,
- avoid collisions and adapt to traffic,
- keep the car within one of three highway lanes

The implemented path planning starts at line 106 of `main.cpp`. First, the ego vehicle collects sensor fusion data to detect other cars. If a car in front is detected and it is within $30m$, then this discovery is flagged.

```cpp
// Check for car being right ahead in current lane  
if ((d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))  
    && ((check_car_s > car_s) && ((check_car_s - car_s) < point_space))) {  
    too_close = true;  
}
```
*(lines 132-135)*

In order to safely perform a lane change it is also necessary to check for traffic to the left and to the right of the car, that is within a distance of $30m$ to the ego vehicle.

```cpp
// Check for car(s) to the left  
if (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2)  
    && (((check_car_s > car_s) && ((check_car_s - car_s) < point_space)) ||  
        ((check_car_s < car_s) && ((check_car_s - car_s) > -point_space)))) {  
    left_lane_free = false;  
}  
  
// Check for car(s) to the right  
if (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2)  
    && (((check_car_s > car_s) && ((check_car_s - car_s) < point_space)) ||  
        ((check_car_s < car_s) && ((check_car_s - car_s) > -point_space)))) {  
    right_lane_free = false;  
}
```
*(lines 137-149)*

If the car is able to change lane to its left or its right and if there is actually a left or right lane to change into and if there are no other cars on the intended lane $30m$ behind and $30m$ in front of the ego vehicle, the car changes lane. Otherwise, it slows down. This was implemented like so:

```cpp
// There is a car ahead -> We need to act -> Determine what to do  
if (too_close) {  
    // Check if proposed left lane change is possible (there is a lane) and execute  
    if (left_lane_free && lane > 0) {  
        lane -= 1;  
        // If not make sure proposed right lane change is possible (there is a lane) and execute  
    } else if (right_lane_free && lane < 2) {  
        lane += 1;  
        // If not reduce speed and stay in lane  
    } else if (ref_vel > check_speed - 5) {  
        ref_vel -= 0.24;  
    }  
    // There is nothing ahead, we can accelerate (slowly) to v_max  
} else if (ref_vel < 49.5) {  
    ref_vel += 0.24;  
}
```
*(lines 152-167)*

In order to now generate smooth, driveable trajectories I realized the approach discussed in the [Project Q&A](https://www.youtube.com/watch?time_continue=2&v=7sI3VHFPP0w&feature=emb_logo).<br>
A new generated set of points should always define a path continuation that starts tangential to the previously driven one as jerk is to be reduced. I therefore have to make sure that a 'connection' between these two point sets (partial paths) is as smooth as possible. In lines 182-209, I make sure two recent points from the previous path exist from which I calculate their yaw. The points serve as starting points for the path continuation.

In lines 211-235 I add a small set of three spaced out points (spaced out by $30m$). Along these points and the two starting points I then create a spline. This smooth path now has to be driven at certain speed, so more points - now along the spline - have to be generated. I realize this in lines 243-279 and add these points for the actual `next_x_vals` and `next_y_vals`.