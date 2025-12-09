# Pervasive-Laboratory-Internship
In this repository you will find configurations useful to control many devices (such as Raspberry PIs) and robots (such as the Turtlebot 4)ng the Stack4Things infrastructure. 
Here you'll find guides in other to configure the devices in optimal manner along with some examples either to test the connection between the device and S4T or to get some inspiration to create something else.

## ROSbotXL
The goal for the **Husarion ROSbotXL** is being able to control it remotely using a WebUI. This requires not just the capacity to make it move but also seeing where it is going. Right now, there is just the possibility to make it move (the vision will be later added). 

### Moving the ROSbotXL remotely
In order to be able to make the **ROSbotXL** move using *Stack4Things* and *Lightning-Rod* it is necessary to create a *plugin* that mimics the *Robot Control* operation found, for example, in the **ROSbotXL** <a href="https://husarion.com/tutorials/howtostart/rosbotxl-quick-start/#robot-control">quick start guide</a>. This has been done tewking the *teleop_twist_keyboard* found both in the *ROSbot XL firmware* and in the <a href "https://github.com/ros-teleop/teleop_twist_keyboard"> Official repository </a>. A *Web UI* has been designed to control the *ROSbot XL*. Both the *plugin* and the *Web UI* will be further analyzed in the following sections.

#### WAMP_ROSbotXL_teleop.py
This plugin contains the interfaces used to control the *ROSbotXL* both via keyboard and virtual joypad. The building block from where the plugin starts is, as previously said, the *teleop_twist_keyboard* script. This script's main highlights are the fact that it binds specific keys to specific values that will be published onto the *cmd_vel* **ROS2** topic which can be found among the topics present in the *ROSbotXL*. In order to publish these values a **rclpy** node is spawned in a separate thread: the values that it must publish are all handled inside the new thread. 
Being able to control the *ROSbotXL* using *Stack4Things* and *Lightning Rod* requires the publishing of a **WAMP RPC** whose purpose is capturing the inputs from the *Web UI* (that replaces the actual keyboard the *teleop_twist_keyboard* script expects) and sending it to the *rclpy thread*. Given the implementation of *Lighting Rod*, the *RPC publisher* must be spawned in another separate thread from the main. In this implementation, there is a total of three threads:

- Main thread
- "*WAMP server* thread which exposes the *RPCs*
- *rclpy* thread, which publishes the values in the *ROS2 topic*

The same discussion can be made for the *virtual joypad* version: the difference is only the structure of the published message.

#### Performance
The main challenge in this kind of application is reducing the latency the most. In order to being able to use the *Web UI* on even the lowest end devices, the key bindings are still made by the *ROSbotXL* board (mimicking the behaviour of the original *teleop* script). 
