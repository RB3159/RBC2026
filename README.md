mkdir src
cd src
git clone https://github.com/RB3159/RBC2026.git
cd ../
colcon build --symlink-install
source install/setup.bash
ros2 launch my_robot_sim simple.launch.py
