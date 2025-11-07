# ROS2 python library (rclpy) for Lightning Rod
In order to have the **rclpy** module avaiable for the usage in Lightning Rod, these two operations are needed (the reinstall operation can be skipped if **rclpy** is already installed, it is advised to look for it first):

``` bash
sudo apt install --reinstall ros-<ROS_DISTRO>-rclpy
echo "/opt/ros/<ROS_DISTRO>/lib/python3.XX/site-packages" | sudo tee /usr/lib/python3.XX/dist-packages/ros2_rclpy.pth
```
This creates a symlink to where the **rclpy** module is installed: it is otherwise not seen by Python.
Then you have to make sure that the necessary libraries are linked properly for every user in the system (Lightning Rod runs as a different user):

``` bash
echo /opt/ros/humble/lib | sudo tee /etc/ld.so.conf.d/ros2-humble.conf
sudo ldconfig
```
Then you must be sure that the envirronment variables like ROS_ are set for all the users: modify the */etc/environment* file with those information. Here follows an example
```bash
CYCLONEDDS_URI="/etc/turtlebot4/cyclonedds_rpi.xml"
FASTRTPS_DEFAULT_PROFILES_FILE="/etc/turtlebot4/fastdds_rpi.xml"
ROBOT_NAMESPACE=""
ROS_DOMAIN_ID="0"
ROS_DISCOVERY_SERVER=""
RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
TURTLEBOT4_DIAGNOSTICS="1"
WORKSPACE_SETUP="/opt/ros/humble/setup.bash"
ROS_SUPER_CLIENT="False"
ROS_LOCALHOST_ONLY="0"
```
For the **Turtlebot 4**, you can find those information in the */etc/turtlebot4/setup.bash* file (you must remove the export keyword).
