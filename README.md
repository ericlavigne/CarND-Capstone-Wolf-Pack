## Capstone Project - Wolf Pack Team
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the Capstone project for the Udacity Self-Driving Car Nanodegree.
We developed software to guide a real self-driving car around a test track.
Using the Robot Operating System (ROS), we created nodes for traffic light
detection and classification, trajectory planning, and control.

![simulator and rviz](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/raw/readme_sketch_2017-11-18/imgs/car-stopping.gif)

### Beyond the Requirements

* Reloading PID parameters while the car is running for faster tuning
* Trajectory plans with multiple successive behaviors so that trajectory planner works well at low frequency
* RViz visualization of car, traffic lights, and trajectory plan
* Smoother acceleration with lower message frequency by adding linear interpolation to the "pure pursuit" code and replacing speed with acceleration in twist messages
* Automatic, real-time adjustments to throttle gain
* Color added to the traffic light message format for smoother stops
* UNet architecture for traffic light detection
* Automated testing via rostest

*Note: Find the latest version of this project on
[Github](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack).*

---

### Team Members

* [Eric Lavigne](https://github.com/ericlavigne) - team lead
* [William Xu](https://github.com/whathelll) - visualization and controller
* [Alena Kastsiukavets](https://github.com/Helen1987) - traffic light detection
* [Mihir Rajput](https://github.com/mihirrajput) - trajectory planner
* [Sergey Vandyshev](https://github.com/sershev) - traffic light classification

---

### Contents

* [Project Components](#project-components)
  * [Visualization](#visualization)
  * [Traffic Light Detection](#traffic-light-detection)
  * [Traffic Light Classification](#traffic-light-classification)
  * [Trajectory Planner](#trajectory-planner)
  * [Waypoint Follower](#waypoint-follower)
  * [Stability Controller](#stability-controller)
  * [Gain Controller](#gain-controller)
  * [Automated Testing](#automated-testing)
* [Usage](#usage)
  * [Installation](#installation)
  * [Running the Simulator](#running-the-simulator)
  * [Running the Simulator with Visualisation](#running-the-simulator-with-visualisation)
  * [Troubleshooting](#troubleshooting)
  * [Running the Project on a Real Car](#running-the-project-on-a-real-car)
  * [Running Automated Tests](#running-automated-tests)

---

### Project Components

* diagram of nodes and messages

#### Visualization

[![visualization-video](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/raw/readme_sketch_2017-11-18/imgs/visualization-video-thumb.png)](https://www.youtube.com/watch?v=JTuVzKwYBiU)

#### Traffic Light Detection

* [UNet architecture](https://lmb.informatik.uni-freiburg.de/people/ronneber/u-net/)
* NN is pretrained on medical images from [Kaggle competition](https://www.kaggle.com/c/ultrasound-nerve-segmentation/data)
* Trained on black-and-white images
* Custom loss function is used based on [Dice coefficient](https://en.wikipedia.org/wiki/S%C3%B8rensen%E2%80%93Dice_coefficient)
* Separate models for simulator (Dice coefficient is 0.81) and Carla (Dice coefficient is 0.66)
* See training code in [detector](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/tree/master/detector) and inference code in [tl\_detector.py](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/blob/master/ros/src/tl_detector/tl_detector.py).

#### Traffic Light Classification

| Simulator Model         | Carla Model                            |
|:-----------------------:|:--------------------------------------:|
| ![simulator model](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/raw/readme_sketch_2017-11-18/imgs/classifier_model_simulator.png)          | ![carla model](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/raw/readme_sketch_2017-11-18/imgs/classifier_model_carla.png)

* Four output classes: GREEN, YELLOW, RED, NONE.
* Test accuracy was 100% for simulator images and 84.7% for Carla images.
* See model and training code for [Carla](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/blob/master/tl_classifier/TL_Classifier-Carla.ipynb) and [Simulator](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/blob/master/tl_classifier/TL_Classifier-Simulator.ipynb) in iPython notebooks.
* See inference code in [tl\_classifier.py](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/blob/master/ros/src/tl_detector/light_classification/tl_classifier.py).

#### Trajectory Planner

| Decision Function       | Ideal Scenario                         |
|:-----------------------:|:--------------------------------------:|
| ![decision function](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/raw/readme_sketch_2017-11-18/imgs/traffic-light-planning.png)          | ![ideal scenario](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/raw/readme_sketch_2017-11-18/imgs/planner-scenario-ideal.png)

* Acceleration and deceleration profiles are governed by the kinematic equations of motion.
* Safe distance allows deceleration at 10% of maximum deceleration while still stopping in time.
* If further than safe distance from the traffic light, the car ignores the traffic light's color and accelerates up to cruising speed.
* Within safe distance of a traffic light, car decelerates at whatever rate would result in stopping exactly on the stop line.
* When car within stopping distance (3 meters) of yellow or red light, it will decelerate at maximum deceleration.
* When car is less than hysteresis distance (1 meter) past the stop line at a yellow or red light, it will continue max deceleration.
* When car is more than hysteresis distance (1 meter) past the stop line, or if the traffic light turns green, car will accelerate up to cruising speed.
* See code in [waypoint\_updater.py](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/blob/master/ros/src/waypoint_updater/waypoint_updater.py).

#### Waypoint Follower

* Substantial improvement over original "pure pursuit" version provided by Udacity
* Originally determined intended speed based only on first waypoint, which quickly became stale
* Originally reported intended speed to stability controller, which also quickly became stale
* Changed to consider current position and varying lookahead distance depending on speed
* Changed message format from speed to acceleration, which is much more stable
* See code in [pure\_pursuit\_core.cpp#calcAcceleration](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/blob/waypoint_follower_acceleration_2017-11-12/ros/src/waypoint_follower/src/pure_pursuit_core.cpp#L104).

#### Stability Controller

![online PID tuning](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/raw/readme_sketch_2017-11-18/imgs/pid-tuning.png)

* Tuned PID parameters with sliders while the simulator was running.
* Tuned for high speed in simulator.
* Needed for moderating the output of waypoint follower and preventing overshoot.
* See code in [stability_controller.py](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/blob/master/ros/src/twist_controller/stability_controller.py).

#### Gain Controller

* We know that the real car has a much more sensitive throttle and brake than the simulator, but don't have enough information to calculate this difference.
* The effect of throttle on acceleration is also very non-linear depending on car's speed.
* Use a series of linear models for 0-10mph, 10-20mph, 20-30mph, etc.
* For each of these small speed ranges, assume required throttle or brake is linear function of current speed and intended acceleration.
* Collect data from car behavior for on-the-fly calibration of throttle and brake models
* See code in [gain_controller.py](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/blob/gain_controller_2017-10-15/ros/src/twist_controller/gain_controller.py).

#### Automated Testing

[![rostest howto video](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/raw/readme_sketch_2017-11-18/imgs/rostest-youtube.png)](https://www.youtube.com/watch?v=j7-_flWgJ88)

---

### Usage

#### Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

#### Running the simulator

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
./rebuild-all.sh
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

#### Running the Simulator with Visualisation
```bash
cd ros
./rebuild-all.sh
source devel/setup.sh
roslaunch launch/wolfpack.launch use_ground_truth:=true

# running rqt_gui in a different terminal
rqt --perspective-file "$(rospack find wolfpack_visualisation)/launch/rqt.perspective"
```

#### Troubleshooting

If the car does not move or behaves poorly, try applying the "eventlet monkey patch" with this modification to the roslaunch command. This helps on some computers and hurts on others, so try both ways if you're having problems.

```bash
EVENTLET_MONKEY_PATCH=true roslaunch launch/styx.launch
```

#### Running the Project on a Real Car
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

#### Running Automated Tests

To run a single unit test with all the rosout logging in the console
```bash
# rostest from ros dir
rostest --text src/waypoint_updater/test/waypoint_updater.test

# nosetest from ros dir
nosetests -s src/twist_controller/test/test_stability_controller.py
```

To run all ROS unit tests
```bash
# from ros dir
catkin_make run_tests
```

To watch for file changes and run unit tests on any file modifications.
If the below commands don't work on windows then change the watch-run.sh,
uncomment the inotify command and comment out the existing inotify command
```bash
# from ros dir watch src dir and run single test
./watch-run.sh src "rostest --text src/waypoint_updater/test/waypoint_updater.test"

# from ros dir watch src dir and run nosetest
./watch-run.sh src "nosetests -s src/twist_controller/test/test_stability_controller.py"

# from ros dir watch src dir and run all unit tests
./watch-run.sh src "catkin_make run_tests"
```

##### Adding ros node tests to new package
read http://wiki.ros.org/rostest/Writing
1. Create a ROS test file
e.g. `ros/src/waypoint_updater/test/test_waypoint_updater.py`

2. Create a launch file with the node to test. Add a node pointing to the test file
e.g. `ros/src/waypoint_updater/test/waypoint_updater.test`
```
<launch>
  <node pkg="mypkg" type="mynode" name="mynode" />
  <test test-name="test_mynode" pkg="mypkg" type="test_mynode" />
</launch>
```

3. add to CmakeLists.txt in the module
```
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/waypoint_updater.test)
endif()
```

4. Add the below to the package.xml
```
  <build_depend>rostest</build_depend>
```

##### Adding ros nosetests to new package
1. Create plain Python unit test file
2. Add to CmakeLists.txt in the Testing section
```
## Add folders to be run by python nosetests
  catkin_add_nosetests(test/test_stability_controller.py)
```
