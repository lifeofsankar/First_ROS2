# --- TASK 01 --- #

**RViz (Robot Visualization)**

Terminal


#1 colcon build

#2 source install/setup.bash

#3 ros2 launch tortoise_bot tortoise_launch.py

**Gazebo Simulation with Teleop**

Terminal 1


#1 colcon build

#2 source install/setup.bash

#3 ros2 launch tortoise_bot teleop_launch.py

Terminal 2


#1 colcon build

#2 source install/setup.bash

#3 ros2 run teleop_twist_keyboard teleop_twist_keyboard

# --- Task 02 --- #

Terminal 1


#1 cd ~/my_workspace/tortoise_ws

#2 colcon build

#3 source /opt/ros/jazzy/setup.bash

#4 source install/setup.bash

#5 ros2 run tortoise_bot lidar_processor

Terminal 2


#1 cd ~/my_workspace/tortoise_ws

#2 source /opt/ros/jazzy/setup.bash

#3 source install/setup.bash

#4 ros2 launch tortoise_bot teleop_launch.py
    (add visualization Lidar panel to see lidar rays and select /scan)

# --- Task 03 --- #

Terminal 1


#1 cd ~/my_workspace/tortoise_ws

#2 source /opt/ros/jazzy/setup.bash

#3 source install/setup.bash

#4 ros2 run tortoise_bot lidar_processor

Terminal 2


#1 cd ~/my_workspace/tortoise_ws

#2 source install/setup.bash

#3 ros2 launch tortoise_bot teleop_launch.py
	(add visualization Lidar panel to see lidar rays and select /scan)
 
Terminal 3


#1 cd ~/my_workspace/tortoise_ws

#2 source install/setup.bash

#3 ros2 launch tortoise_bot ball_follower.py

# --- Task 04 --- #

# --- Task 15 --- #

Terminal 1


#1 cd ~/my_workspace/face_detect_ws

#2 source /opt/ros/jazzy/setup.bash

#3 source install/setup.bash

#4 ros2 run face_detect face_detect

Terminal 2


#1 cd ~/my_workspace/face_detect_ws

#2 source install/setup.bash

#3 ros2 launch face_detect web_processor

# --- Task 16 --- #

# --- Task 23 --- #

Terminal 1


#1 cd ~/my_workspace/tortoise_ws

#2 colcon build

#3 source install/setup.bash

#4 ros2 launch tortoise_bot teleop_launch.py

Terminal 2


#1 cd ~/my_workspace/tortoise_ws

#2 python3 -m html.server
	(open "localhost:8000/web.html" in any browser)
 
Terminal 3


#1 cd ~/my_workspace/tortoise_ws

#2 python3 simple_bridge.py


##
**💡 BONUS: If something doesn’t work, rebuild the package**


**cd ~/my_workspace/tortoise_ws**

**colcon build --packages-select tortoise_bot**

**source install/setup.bash**

