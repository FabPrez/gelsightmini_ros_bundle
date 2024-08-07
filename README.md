# Gelsight Mini ROS Bundle

Repository containing ROS packages for interfacing with the Gelsight Mini tactile sensor, enabling integration into robotic systems and applications.

## Overview

This repository contains all the necessary ROS packages and dependencies to interface with the Gelsight Mini tactile sensor. It allows for reading images from both a physical sensor and a simulator and utilizes the Gelsight SDK with its neural network to convert images into point clouds.

## Included Packages

- **Gelsight SDK**: Forked and modified version of the Gelsight SDK for processing tactile images.
  - [Gelsight SDK Repository](https://github.com/FabPrez/Gelsight-SDK)
- **Taxim**: Forked and modified version of the Taxim simulator for generating synthetic tactile data.
  - [Taxim Repository](https://github.com/FabPrez/Taxim)

> Note: The packages required for the project will be automatically installed following the steps outlined in the Setting up your environment section. Direct modifications or pushes to packages other than gelsightmini_ros_bundle should be avoided.

## Environment Structure

The repository is organized into the following main directories:

- `rosinstall`: Contains ROS installation files.
- `Gelsight-SDK`: Contains the Gelsight SDK for image processing used with the real sensor and the simulated one.
- `Taxim`: Contains the Taxim simulator for generating synthetic tactile data.
- `gelsightmini_ros_bundle`: The main ROS package for interfacing with the Gelsight Mini sensor, serving both to simulate tactile images intuitively and to use the real sensor more effectively, extracting fundamental data.

### Detailed Structure

Here is the detailed structure of the repository:

```
gelsightmini_ros_bundle/
│
├── gelsightmini_venv/           # Virtual environment for dependencies
│
├── gsmini_to_pc/                # Code to convert real Gelsight images to point clouds
│
├── data_folder/                 # Contains data used for simulation and processing
│
├── simulated_gmini/             # Contains packages for simulating tactile images
│   ├── tactile_image_simulator/ # Converts point clouds to tactile images
│   ├── interactive_object_controller/ # Interface for simulating single or dual contact with a parallel gripper
│
├── gelsightmini_ros_bundle.repos # File specifying ROS repositories
├── requirements.txt             # Python dependencies
├── README.md                    # This README file
```

## Setting up Your Environment

Create a ROS workspace (if you already have one, you can skip this step):

```bash
mkdir -p ~/gsmini_ws/src
cd ~/gsmini_ws
catkin init
```

Then:

```bash
cd ~/gsmini_ws/src
git clone https://github.com/FabPrez/gelsightmini_ros_bundle.git
mkdir rosinstall
cp gelsightmini_ros_bundle/gelsightmini_ros_bundle.repos rosinstall/gelsightmini_ros_bundle.repos 
vcs import < rosinstall/gelsightmini_ros_bundle.repos
cd ..
vcs pull src
sudo apt update && sudo apt upgrade -y
rosdep install --from-paths src --ignore-src -r -y
catkin config -j $(nproc --ignore=2)
catkin build -cs --mem-limit 50%
source devel/setup.bash
```

Create the virtual environment and install the requirements:

```bash
cd src/gelsightmini_ros_bundle/
python3 -m venv gsmini_venv
source gsmini_venv/bin/activate
cd gelsightmini_ros_bundle/
pip install -r requirements.txt
```

### Other Dependencies

```bash
sudo apt-get install ros-noetic-cv-bridge
```

---

## HowTo

### Real tactile images
Obtain tactile images as for the gelsgith-sdk package, generating the respective pointcloud:

```bash
    roslaunch gsmini_to_pc gsmini_to_pc.launch image_topic:=/new_image_topic pointcloud_image_topic:=/new_pointcloud_topic is_simulated:="False"
```
### Simulate Tactile Images with Simulated_gmini

This package uses the Taxim simulator to render images and provides a ROS-based interface to intuitively move the object to obtain contact. In particular:

This ROS interface for Taxim allows for obtaining tactile images from a PLY object file positioned in the `data_folder` directory. Specify the object name when launching the package with `obj_name`:

1. **Single Contact Simulation:**

    ```sh
    roslaunch tactile_image_simulator singleContact_tactile_simulator.launch obj_name:=Simple_Pin
    ```

2. **Parallel Gripper Simulation:**

    ```sh
    roslaunch tactile_image_simulator parallelGripper_tactile_simulator.launch obj_name:=Simple_Pin
    ```

![Simulator Screen](data_folder/simulator_screen.png)

#### Moving the Object

To move the object, use the dynamic reconfigure interface as shown in the provided screenshot. This interface allows you to adjust the position and orientation of the sensor and the object.

#### Output Data

The package is designed to manage multiple sensors. Simply specify the topic on which you want to publish. In the launch files mentioned above, by default, it publishes to:
- `/first_finger_simulated_image`
- `/second_finger_simulated_image`

### Generate Point Clouds from Tactile Images

You can generate point clouds from tactile images (real or simulated) using the `gsmini_to_pc` package, which interfaces with the Gelsight SDK. By choosing the topic from which the tactile image originates and specifying whether it is simulated or not via the launch parameters, you can manage this for different sensors.

There are two launch files that interface with Simulated_gmini:

- **Single Finger Simulation to Point Cloud**:

    ```sh
    roslaunch gsmini_to_pc simulate_1fing_gsmini_to_pc.launch
    ```

- **Dual Finger Simulation to Point Cloud**:

    ```sh
    roslaunch gsmini_to_pc simulate_2fing_gsmini_to_pc.launch
    ```

### Custom Launch

In general, if you want to launch `gsmini` with custom parameters, you can:

```sh
roslaunch gsmini_to_pc gsmini_to_pc.launch image_topic:=/new_image_topic pointcloud_image_topic:=/new_pointcloud_topic is_simulated:=false
```

---

## ⚠️ Important Note

<div style="background-color: #f8d7da; padding: 10px; border-radius: 5px;">
  <strong>IMPORTANT NOTE:</strong> DO NOT RUN OR LAUNCH INSIDE THE VENV!
</div>

**⚠️ Please read this important note before using the project.**

---
