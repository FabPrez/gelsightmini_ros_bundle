# gelsightmini_ros_bundle
Repository containing ROS packages for interfacing with the Gelsight Mini tactile sensor, enabling integration into robotic systems and applications.

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
cd src/
python3 -m venv gsmini_venv
source gsmini_venv/bin/activate
cd gelsightmini_ros_bundle/
pip install -r requirements.txt
```
