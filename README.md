# iiwa_hw_grl


This repository contains the application code used to communicate with
the KUKA iiwa.  Communicating with the arm requires two
pieces, a Java application to run on the KUKA Sunrise cabinet, and an
local ROS/C++ application communicating with the cabinet using the FRI
protocol.

The KUKA control cabinet has two network interfaces which are
configured with static IP addresses.  Both will need to be connected
to communicate with the arm.

 * X66: 172.31.1.147/16 -- This interface is used by Sunrise Workbench to load new software onto the controller.
 * KONI: 192.170.10.2/24 -- This is the interface which FRI clients communicate over.  That's not in the reserved IP space, so it could potentially cause a conflict if you happen to want to contact a host in that subnet.

Build the driver program to communicate with the iiwa arm using
FRI, and with the controlling application using ROS.  

You will need KUKA FRI source code. The current tested version is 1.11

```
mkdir -p ~/ws_kuka/src
cd ~/ws_kuka/src
git clone https://github.com/IFL-CAMP/iiwa_stack.git
cd iiwa_stack
git checkout 3515f00c8b7614b8944bfd5306cf7b42c5dd334f
git clone https://github.com/tdinesh/iiwa_hw_fri
cd iiwa_hw_fri
git checkout devel_velocity_control
mkdir external
```
Copy the FRI zip file to the ROS package `external` folder
`cp /path_to_folder/FRI-Client-SDK_Cpp.zip ~/ws_kuka/src/iiwa_hw_fri/external`

```
catkin build
```

ROS dependencies to install. Choose `kinetic/melodic` according to system
```
sudo apt install ros-melodic-ros-control ros-melodic-octomap-msgs ros-melodic-object-recognition-msgs ros-melodic-realtime-tools ros-melodic-pcl-ros ros-melodic-control-toolbox ros-melodic-controller-manager ros-melodic-controller-interface ros-melodic-controller-manager-msgs ros-melodic-moveit ros-melodic-joint-state-controller ros-melodic-joint-trajectory-controller ros-melodic-position-controllers ros-melodic-hardware-interface ros-melodic-ros-controllers ros-melodic-controller-manager
```

You'll need to configure the system which will communicate directly with the KONI interface.  This
system must be configured for the IP address 192.170.10.100 (netmask
/24, or 255.255.255.0) (this can be changed in the Java applications).
KUKA recommends directly attaching the computer to the KONI port
instead of using a switch.  Some network interfaces (particularly some
Intel models) have issues when cabled directly to the KONI port (problems
include link flapping up/down repeatedly).

Connect ethernet from KONI to your ROS computer. Set IP of the network interface to be `192.170.10.100`

## Running the system 

On the SmartPad, turn the key switch, and choose the "AUT" mode, then
turn the key switch back. Choose "FRIPositionDriver" from "Application". Press the green "Play"
button on the left sidebar of the SmartPad.

```
roslauch iiwa_hw_fri iiwa_hw_fri.launch
```

Once connection is successfully established

```
roslaunch iiwa_hw_fri moveit_grl.launch
```
