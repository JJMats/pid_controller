# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

*** Reflection

The PID Controller project for the Udacity Self-Driving Car Engineer Nanodegree program was a great opportunity to practice tuning PID control algorithms and their parameters to allow a vehicle to steer itself back to the center of the lane and maintain that position while it traverses a lap around a simulated track. This is helpful for many types of control systems as they are rarely ever able to function “perfectly.”

** Implementation

The PID architecture chosen for the project was a parallel PID controller, whose gain and error parameters are calculated individually and then summed to produce an output value. The difference in the commanded output value and the target value is the new calculated error that is fed back into the input of the controller to allow for continuous adjustment.

The implementation began with setting the I and the D term to zero values, and then changing the <em>proportional</em> (P) term to an initial starting value. A typical approach to PID tuning is to adjust the P parameter first, and slowly increment the value up to a point at which the error is slightly overcorrected and the system becomes unstable. At this point, the D term is implemented, followed by the I term as necessary. This approach did not work well for this system.

Setting the P term alone would cause the system to form a diverging oscillation on a section of road that was not straight, which would inevitably result in the vehicle exiting the roadway. Small P values tended to delay this end-result, while large P values caused this to occur more expediently.

It was determined that the <em>derivative</em> (D) term was immediately necessary to keep the system under control by adjusting the steering angle value in relation to the rate at which the angle was changing. If the value was set too low, the system would still help reduce the diverging oscillation problem but would struggle to minimize the error correction overshoot caused by the P term. Setting the D term too high would allow the vehicle to track better in the center of the lane but would cause rapid steering jitter as the vehicle would try to make frequent and extreme steering angle adjustments to correct small changes in error.

Once a balance was achieved between the P and D terms, the <em>integral</em> (I) term was increased to help reduce the error in turns. This parameter had to be set high enough to allow the I-correction value to increase and decrease rapidly enough to increase assist on turn entry and reduce assist on turn exit. If this response is delayed, the vehicle can tend to steer towards the outside of the turn on entry, over-correct and oscillate throughout the turn, and steer towards the inside of the turn on exit.

A common problem with integral gain in a PID controller is that a long-term error may allow the integral error to increase to a very large value that may take an excessively long time to decrease back to zero or may cause the system to become increasingly unstable. This is known as “integral wind-up.” To circumvent this problem, the integral error was clamped to a maximum value of +/- 5.0 (PID.cpp, code lines 97-104).

** Simulation

Various methods were implemented to assist in tuning the PID parameters of the system, but it was determined that a starting point should be manually resolved. Through multiple simulations, parameters were chosen that allowed the system to traverse multiple laps around the track without leaving the road surface.

After manually tuning the PID controller to a desired result, the Twiddle algorithm was implemented (PID.cpp, code lines 117-145) to facilitate PID parameter fine tuning. The implementation consisted of steps to record the total error (cumulative deviation from the target location in the center of the track for each update) of a travelled distance that was longer than one lap around the test track, make small incremental adjustments (positive and negative) to the gain terms individually, and recheck this error to determine if an improvement was made. If an improvement was calculated from the last iteration, the next term is subsequently adjusted, and the algorithm continues.

One concern of using the Twiddle algorithm is that it seemed to slowly increase each of the terms per iteration. It was observed that various combinations of these terms can produce a successful result of keeping the vehicle on the road surface, but some can cause the vehicle to operate on the ragged edge of instability. It was found that increasing the P, I, and D terms together would reduce the overall error, but would create a large amount of jerk as the vehicle rapidly and frequently made corrections to its steering angle. As the Twiddle algorithm continued to tune, the error value and visually observed ride comfort for passengers began to show an inverse relationship.

A final attempt at automatically tuning the PID parameters was to automatically adjust the gain parameters by fixed increments, one at a time, and then checking to see if the error was improved. (PID.cpp, code lines 147-179). This differed from the Twiddle algorithm because if an error reduction was found by increasing or decreasing the term, that same term was incremented in the same direction until it stopped producing an improvement. At this point, the subsequent term was adjusted in the same manner, and this continued until manually concluded.

The latter algorithm produced similar results to Twiddle but the error did not seem to converge as rapidly in the early laps. Although, after over 30 laps of simulation, the error was nearly identical.

The final parameters were:

<strong>Manually Tuned:</strong>
 - P: 0.090, I: 0.009, D: 0.100
 - Notes: Smoother steering, tends to drive towards the outside edge of the lane on turn entry, lower peak steering angles

<strong>Twiddle:</strong>
 - P: 0.313866, I: 0.0194341, D: 0.29289
 - Notes: Finds lane center better than the manual parameters, but steering movements are frequent and have high jerk

<strong>Algorithm B:</strong>
 - P: 0.165, I: 0.039, D: 0.225
 - Notes: Finds lane center well but tends to oscillate around zero error, has frequent high angle (and high jerk) steering movements which are ramped out rapidly.

** Potential Improvements

Since the error reference for the automatic parameter tuning algorithms was based upon a total calculated error over the duration of 1400 updates, the distance that the vehicle travelled in this timeframe was not always identical. This is a result of vehicle instability and associated vehicle speed adjustment. Therefore, the reference lap was not always identical, inducing an additional error into the measurement. If a start/stop waypoint could be established, this may allow for some consistency in error calculated on a per lap basis.

Add throttle control to decrease the vehicle speed as error increases. This may help stabilize the steering PID controller.

Add adjustable gain parameters to account for vehicle speed as tire related vehicle dynamics (i.e. slip angles) will change with velocity and turning angle.