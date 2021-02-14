# Self-Driving-Car_System-Integration
Using ROS to program a self-driving car

## Note
This project does not include a neural network traffic light detection. The traffic light detection is using simulator ground truth detected.

## System Architecture Diagram
The following is a system architecture diagram showing the ROS nodes and topics used in the project. They are described briefly in the Code Structure section below.
![](/readme_data/ros-graph.png)

## Code Structure
The code is contained entirely within the /ros/src/ directory. Within this directory, you will find the following ROS packages:

### /ros/src/tl_detector/
This package contains the traffic light detection node: tl_detector.py. This node takes in data from the `/image_color`, `/current_pose`, and `/base_waypoints` topics and publishes the locations to stop for red traffic lights to the `/traffic_waypoint` topic.

The `/current_pose topic` provides the vehicle's current position, and `/base_waypoints` provides a complete list of waypoints the car will be following.

I built both a traffic light detection node and a traffic light classification node. Traffic light detection takes place within tl_detector.py, whereas traffic light classification takes place within `../tl_detector/light_classification_model/tl_classfier.py.`
![](/readme_data/tl-detector-ros-graph.png)


### /ros/src/waypoint_updater/
This package contains the waypoint updater node: `waypoint_updater.py`. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the `/base_waypoints`,` /current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint topics`, and publish a list of waypoints ahead of the car with target velocities to the `/final_waypoints` topic.

![](/readme_data/waypoint-updater-ros-graph.png)

### /ros/src/twist_controller/
Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node dbw_node.py and the file twist_controller.py, along with a pid and lowpass filter that you can use in your implementation. The dbw_node subscribes to the `/current_velocity` topic along with the `/twist_cmd topic` to receive target linear and angular velocities. Additionally, this node will subscribe to `/vehicle/dbw_enabled`, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd topics`.
![](/readme_data/dbw-node-ros-graph.png)


In addition to these packages you will find the following. The styx and styx_msgs packages are used to provide a link between the simulator and ROS, and to provide custom ROS message types:

* /ros/src/styx/
  A package that contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.
* /ros/src/styx_msgs/
  A package which includes definitions of the custom ROS message types used in the project.
* /ros/src/waypoint_loader/
  A package which loads the static waypoint data and publishes to /base_waypoints.
* /ros/src/waypoint_follower/
  A package containing code from Autoware which subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities in the form of twist commands to the `/twist_cmd topic`.
  

## Installation
Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* To improve performance while using a VM, we recommend downloading the simulator for your host operating system and using this outside of the VM. You will be able to run project code within the VM while running the simulator natively in the host using port forwarding on port 4567. You will find more information on how to set up port forwarding later (see below).
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
Port forwarding is required when running code on VM and simulator on local host system. You will be able to run project code within the VM while running the simulator natively in the host using port forwarding on port 4567. For more information on how to set up port forwarding, see [this file](https://github.com/wolfgang-stefani/Self-Driving-Car_System-Integration/blob/main/readme_data/Port%2BForwarding.pdf).

## Usage

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

## Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

## Other library/driver information
Outside of `requirements.txt`, here is information on other driver/library versions used in the simulator and Carla:

Specific to these libraries, the simulator grader and Carla use the following:

|        | Simulator | Carla  |
| :-----------: |:-------------:| :-----:|
| Nvidia driver | 384.130 | 384.130 |
| CUDA | 8.0.61 | 8.0.61 |
| cuDNN | 6.0.21 | 6.0.21 |
| TensorRT | N/A | N/A |
| OpenCV | 3.2.0-dev | 2.4.8 |
| OpenMP | N/A | N/A |
