# ROS packages for the ABB YuMi (IRB14000) robot, Noetic version

- This fork is modified from [kth-ros-pkg/yumi](https://github.com/kth-ros-pkg/yumi) for controlling Yumi with ROS Noetic.
- For the original document and instructions, please check the [wiki](https://github.com/kth-ros-pkg/yumi/wiki).
- Currently, this version is configured to use RWS to communicate. A tested backup firmware is under `yumi_firmware_backup`. You can use it to restore the system configuration.

## Configuration
- Install the following packages
	```
	sudo apt-get install \
	        python3-pip \
	        protobuf-compiler \
	        protobuf-c-compiler \
	        ros-$ROS_DISTRO-control-toolbox \
	        ros-$ROS_DISTRO-controller-interface \
	        ros-$ROS_DISTRO-controller-manager \
	        ros-$ROS_DISTRO-effort-controllers \
	        ros-$ROS_DISTRO-force-torque-sensor-controller \
	        ros-$ROS_DISTRO-gazebo-ros-control \
	        ros-$ROS_DISTRO-joint-limits-interface \
	        ros-$ROS_DISTRO-joint-state-publisher \
	        ros-$ROS_DISTRO-joint-state-controller \
	        ros-$ROS_DISTRO-joint-trajectory-controller \
	        ros-$ROS_DISTRO-moveit-commander \
	        ros-$ROS_DISTRO-moveit-core \
	        ros-$ROS_DISTRO-moveit-planners \
	        ros-$ROS_DISTRO-moveit-ros-move-group \
	        ros-$ROS_DISTRO-moveit-ros-planning \
	        ros-$ROS_DISTRO-moveit-ros-visualization \
	        ros-$ROS_DISTRO-moveit-simple-controller-manager \
	        ros-$ROS_DISTRO-position-controllers \
	        ros-$ROS_DISTRO-rqt-joint-trajectory-controller \
	        ros-$ROS_DISTRO-transmission-interface \
	        ros-$ROS_DISTRO-velocity-controllers 

	pip3 install --user pyftpdlib
	pip3 install --user --upgrade pyassimp

	sudo apt install ros-$ROS_DISTRO-hector-xacro-tools
	```

- Compile abb_driver
	```
	cd $HOME/catkin_ws

	git clone -b melodic-devel https://github.com/ros-industrial/abb_driver.git src/abb_driver

	rosdep update

	rosdep install --from-paths src/ --ignore-src --rosdistro noetic

	catkin build
	```
- Compile industrial_core
	```
	$ cd $HOME/catkin_ws

	$ git clone -b melodic-devel https://github.com/ros-industrial/industrial_core.git src/industrial_core

	$ rosdep update

	$ rosdep install --from-paths src/ --ignore-src --rosdistro noetic

	$ catkin build
	```
- If you encounter Gazebo error:
	```
	RLException: Invalid <param> tag: Cannot load command parameter [robot_description]: no such command [['/opt/ros/noetic/share/xacro/xacro.py'
	```
	Modify your launch file (e.g.,`yumi_gazebo_pos_control.launch`)

    Change `<param name="robot_description" command="$(find xacro)/xacro.py` to `<param name="robot_description" command="$(find xacro)/xacro`

    Reference: （[ROSの勉強　第23弾：チェスセットのモデル作成](https://qiita.com/Yuya-Shimizu/items/f1a22d430a3f6343b3e7)）
