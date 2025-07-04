mkdir src
cd src
git clone this
cd ..
colcon build --packages-select tuto_package
source install/setup.bash
ros2 launch tuto_package tuto.launch.py
