# Generic_ur5_controller
## Description:

simple general purpose ur5 controller

contains python to robot connections and example setup

connection is done with the PC running python as a host, the robot will connect to the pc through ethernet(connected on the underside of the robot control box), allowing commmands to be sent over socket. the robot is controlled by calling functions from the kg_robot.py class e.g. 

robot=kg_robot.kg_robot()        creates a kg_robot object called robot

robot.translatel_rel([0,0,-0.1]) move the robot in the z direction by -0.1m

in addition to this controller, the PC can connect to the robot 'dashboard server' using the module kg_robot_dashboard.py, where the robot acts as the host. predefined commands can be sent over this connection, such as "load kg_client.urp\n" and "play\n". this is just for quality of life and automates the initialisation of the robot on startup and resets the program everytime the python program is reset. more commands are possible if needed (contact me or see https://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/dashboard-server-cb-series-port-29999-15690/)

the teach_mode.py is a seperate set of functions which allows you to manually move the robot, record the trajectory and replay it. this is an example of a more specialised set of functions which is initialised with kg_robot by adding self.teach_mode = teach_mode.teach_mode(self) to the \__init__ of kg_robot.

 ## Connection Setup:

### Load Client Program onto robot
-copy contents of ur5_client folder to a memory stick and plug into robot teach pendant
-save kg_client.urp to ur5 by loading from the loading tab and pressing 'save as' from the 'file' drop down menu, save in the \programs directory.


### Setup ip and ports
-this version uses two connections.
#### kg_robot
-the general connection from kg_client will use the ip address of the computers ethernet connection. for windows open command prompt and enter ipconfig and find the ipv4 address of the ethernet adapter. set self.host in kg_robot.py line 27 and address of var_1:=socket_open('address',port) in kg_client.urp line 8, to this. 

-this connection uses self.port, this is set in python when initialising the kg_robot object, see example in generic_ur5_controller\generic_ur5_controller.py. this must match the port in var_1:=socket_open('address',port) kg_client.urp line 8. use 30000 or >=30010 (29999 reserved for dashboard, 30001-30004 reserved for data exchange)
#### kg_robot_dashboard
-the dashboard port uses the ip address of the robot, the ip address of the robot is set by returning to the home page of the pedant, selecting 'setup robot' then 'network', selecting static address, inputting the new ip and applying. this is set in python when initialising kg_robot. the port of this connection is 29999 and should not be changed.

-this address should be set to match the ip of the host computer with the last number different. e.g. if the host is 192.168.1.100 set the robot static address to 192.168.1.x where x!=100
#### end effector
-some support for serial connections, you may need to setup your own communications protocol
-ee_port: e.g. 'COM20' of any connected end effectors


## Troubleshooting:

-have done everything above, and robot still not connecting - try turning off wifi and/or disable other network connections by going to 'network and sharing center'->'change adapter settings' then select a device and disabling
    

## Getting Started:
-main loop in \Generic_ur5_controller\Generic_ur5_controller.py, contains examples of using kg_robot and teach mode

-control the robot using the functions defined in kg_robot.py in the ur5 commands section, if the desired robot function doesn't exist and cannot be created through a combination of existing functions, it can be added by modifying kg_robot.py and kg_client.urp (contact me for more details). complete capabilities are described in https://s3-eu-west-1.amazonaws.com/ur-support-site/18679/scriptmanual_en.pdf (if link doesn't work search for ur5 script api)

-create specialised robot fns and attach in same format as teach_mode.py by adding an init to kg_robot.py, e.g. complex move sequences, cameras, calibrations and robot workspaces

-use waypoint.py for global robot poses, joints and tool centre points (tcp)
