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
 
Connect ethernet from KONI to your ROS computer. Set IP of the network interface to be `192.170.10.100`

ROS dependencies to install
```
sudo apt install ros-melodic-control-toolbox ros-melodic-joint-state-controller
```

After cloning and compiling the package

Run the Java application on the KUKA Smartpad. Then 
```
roslauch iiwa_hw_fri iiwa_hw_fri.launch
```

Once connection is successfully established

```
roslaunch iiwa_hw_fri moveit_grl.launch
```
