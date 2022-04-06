catkin create pkg custom_msgs --catkin-deps std_msgs
rosrun rosserial_python serial_node.py /dev/ttyACM0
sudo apt install ros-melodic-rosserial
sudo apt install ros-melodic-rosserial-arduino
catkin build
catkin build package_name
rosrun rosserial_arduino make_libraries.py .
rosrun lifting_disks_joy_sub lifting_disks_joy_sub_node
rosparam joy_node "/dev/input/js0"
rosrun joy joy_node


Custom Messageをつくる時にはCMakeListsのadd_message_filesの書き換えを忘れずに！