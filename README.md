# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

[//]: # (Image References)
[p0.1_i0_d0]: ./recordings/p0.1_i0_d0.gif "P 0.1, I 0, D 0"
[p0.1_i0_d1.0]: ./recordings/p0.1_i0_d1.0.gif "P 0.1, I 0, D 1.0"
[p0.1_i0.0001_d1.0]: ./recordings/p0.1_i0.0001_d1.0.gif "P 0.1, I 0.0001, D 1.0"

## Reflection

### P - Proportional Component
The P componenent of the controller uses a value proportional to the cross-track error (CTE). This value causes the overall controller to oscillate around the target CTE. A high P coefficient causes higher oscillations, while a low P coefficient causes slower oscillations. The high oscillations matched with a high enough speed causes the vehicle to overshoot off the track quickly. The slower oscillations cause the vehicle to react too slowly to sharper turns, also causing the vehicle to veer off the track. 

**P: 0.1, I: 0, D: 0**

![P 0.1, I 0, D 0][p0.1_i0_d0]

### I - Integral Component
The I component takes into account the total error of the controller. This accumulates the previous errors and contributes to the control signal when the total error is significant. This can correct for a continuous drift from the target error by summing the errors built up and adjusting the signal when significant enough. 

**P: 0.1, I: 0.0001, D: 1.0**

![P 0.1, I 0.0001, D 1.0][p0.1_i0.0001_d1.0]

### D - Differential Component
The D component represents a differential of the error on the controller. This effectively takes into account the change in error over a window of readings. A large change in error in either direction warrants a large adjustment in the control signal. This manifests itself in the vehicle on sharper turns, where the P component is insufficiently responsive to the changing turn angle. The D component provides the needed boost to correct the steering angle.

**P: 0.1, I: 0, D: 1.0**

![P 0.1, I 0, D 1.0][p0.1_i0_d1.0]

### Final Coefficients
I determined my final PID coefficients through manual tuning. This was to allow for a visual feedback to what each component contributed to the control signal. I arrived at values of **P: 0.1, I: 0.0001, D: 1.0** for my controller. Although higher values reduced the overall CTE and kept the vehicle closer to the center, I found that the vehicle would wobble much more significantly. I prioritized the percieved "smoothness" of the vehicle's turns over minimizing CTE. I found my selected final values to balance the smoothness while being able to complete the track.

I also helped the vehicle by introducing some variability to the speed. I reduced the speed when the observed CTE reached a certain threshold. I also looked at the differential error to reduce speed on sharp turns.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
