# bag image exporter

## 1. Description:
The provoded launchfile exports images from a selected *.bag file to a directory of your choice.

## 2. Requirements
* image_view 
* rosbag

## 3. Usage:
from bag_image_exporter_dir call:
`roslaunch launch/bag_image_exporter.launch bagdir:=/path/to/directory/with/bagfile bagfile:=name_of_file.bag targetdir:=/directory/where/to/save/images`
###### Args
* bagdir - location directory of the .bag file (without slash(/) in the end)
* bagfile - filename of the .bag file (e.g. file.bag) (default: just_traffic_light.bag)
* targetdir - Destination directory to save all the images from the .bag file (default: ROS_HOME)