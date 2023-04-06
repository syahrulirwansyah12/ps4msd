# ps4msd
ROS package to read PS4 DualShock controller that uses a Sub-GHz LPWA 920MHz wireless module that enables long-distance communication (IM920sL Interplan with XIAO-RP2040 board from Seeed Studio).
This takes [naoki-mizumi/ds4_driver](https://github.com/naoki-mizuno/ds4_driver.git) as message reference.

# Installation
1. Clone this project to your catkin's workspace src folder.
```bash
cd ~/<catkin-workspace>/src
git clone https://github.com/syahrulirwansyah12/ps4msd.git -b devel
```
2. Run catkin_make in your workspace folder.
```bash
cd ~/<catkin-workspace> && catkin_make
```
3. In case you haven't installed pyserial yet, please install it first.
```bash
pip install pyserial
```
or
```bash
pip3 install pyserial
```
if you're using Python3.

# Running Program
1. Check the authority of XIAO-RP2040's serial-port:
```bash
ls -l /dev/ttyACM*
```
2. Add the authority of write: (such as /dev/ttyACM0):
```bash
sudo chmod 666 /dev/ttyACM0
```
3. You can set the permission permanently by creating a udev rule.
```bash
cd /etc/udev/rules.d/
sudo touch local.rules
```
then add:
```bash
ACTION=="add", KERNEL=="/dev/ttyACM0", MODE="0666"
```
to the file you have been made.\

4. After setting up the serial port you can run the node by running roscore then rosrun:
```bash
roscore
rosrun ps4msd ps4msd_node.py
```

# ROS Nodes
1. Published Topics\
`/joystick/joystate` (ps4msd/Joystate)\
&emsp;&emsp; publishes topics from the joystick state\
`/joystick/cmd_vel` ([geometry_msgs/Twist](https://docs.ros.org/en/jade/api/geometry_msgs/html/msg/Twist.html))\
&emsp;&emsp; publishes topic of command velocity from the right analog hat
2. Parameters\
`serial_port`\
&emsp;&emsp; serial port name used in your system. (`string`, default: `/dev/ttyACM0`)\
`baudrate`\
&emsp;&emsp; serial port baud rate. (`int`, default: `115200`)\
`max_speed`\
&emsp;&emsp; maximum linear speed in m/s. (`float`, default: `0.4`)\
`max_turn`\
&emsp;&emsp; maximum angular speed in rad/s. (`float`, default: `0.8`)
