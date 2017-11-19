## Capstone Project - Wolf Pack Team
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the Capstone project for the Udacity Self-Driving Car Nanodegree.
We developed software to guide a real self-driving car around a test track.
Using the Robot Operating System (ROS), we created nodes for traffic light
detection and classification, trajectory planning, and control.

TODO || Side-by-side animated GIFs: Driving in Simulator, RViz Visualization

We went beyond the project requirements in several ways:

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
  * [Running the Project on a Real Car](#running-the-project-on-a-real-car)
  * [Running Automated Tests](#running-automated-tests)

---

### Project Components

* diagram of nodes and messages

#### Visualization

* animated GIF of visualization with link to full video
* describe purpose and details of visualization

#### Traffic Light Detection

* [UNet architecture](https://lmb.informatik.uni-freiburg.de/people/ronneber/u-net/)
* NN is pretrained on medical images from [Kaggle competition](https://www.kaggle.com/c/ultrasound-nerve-segmentation/data)
* Trained on black-and-white images
* Separate models for simulator and Carla

#### Traffic Light Classification

* describe network architecture
* validation accuracy for each class

#### Trajectory Planner

![decision function](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/raw/readme_sketch_2017-11-18/imgs/traffic-light-planning.png)

* graphic for decision function
* describe scenarios

#### Waypoint Follower

* original version including problems
* how we solved problems
* link to branch until merged

#### Stability Controller

* Online PID tuning
* Tuned for high speed in simulator

#### Gain Controller

* why needed
* complications in throttle gain model
* simplification: series of linearized models
* exploration
* link to branch until merged

#### Automated Testing

[![rostest howto video](https://github.com/ericlavigne/CarND-Capstone-Wolf-Pack/raw/readme_sketch_2017-11-18/imgs/rostest-youtube.png)](https://www.youtube.com/watch?v=j7-_flWgJ88&feature=youtu.be)

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
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

#### Running the Simulator with Visualisation
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/wolfpack.launch use_ground_truth:=true

# running rqt_gui in a different terminal
rqt --perspective-file "$(rospack find wolfpack_visualisation)/launch/rqt.perspective"
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
