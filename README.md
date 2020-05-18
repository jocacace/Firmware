# Multi sensorized quadcopter for PX4 Gazebo simulator 

This repository contains all necessary files to run the PX4 firmware on ROS-Gazebo simulation, usisng an IRIS based quadrotor equipped with a camera, a depth camera, a laser scanner and a sonsar sensor. 

To start the simulation clone the repository including all the submodules:
		
        $ git clone https://github.com/jocacace/Firmware --recursive

Could be convenient to switch on the devel branch

				$ cd Firmware/Tools/sitl_gazebo        
				$ git checkout prisma-devel

Compile and launch it!
		
        $ cd Firmware && make px4_sitl_default 
				$ make px4_sitl_defaul gazebo 

    During the installation you should install some additional dependencies (follow the instruction in the compilation shell):
    
				$ sudo apt install python3-pip
				$ pip3 install --user empy
				$ pip3 install --user toml
				$ pip3 install --user numpy
				$ pip3 install --user packaging
				$ sudo apt-get install libgstreamer-plugins-base1.0-dev
				$ pip3 install --user jinja2


    To allow ROS/FCU communication you should install Mavros package:

				$ sudo apt-get install ros-melodic-mavros ros-melodic-mavros-msgs 

    
    Now you can install the geographic dataset

				$ sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh
        

To launch the Iris version equipped with the camera, you should load the gazebo configuration. You could use the __load_sitl_conf.sh__ script placed in the sitl_gazebo folder

				$ roscd px4
				$ cd Tools/sitl_gazebo && source load_sitl_conf.sh
			 


##### Launch the simualtion scene
After properly configured the environment, launch the simulation use this command:

		$ roscd px4
		$ cd Tools/sitl_gazebo && source load_sitl_conf.sh
		$ roslaunch px4 iris_camera.launch

You can check the active topics and services to test the correct working of MAVROS bridge. 

##### Arm the quadrotor

To arm the quadrotor we can use the MAVROS service:

		rosservice  call  /mavros/cmd/arming "value: true"
 
##### Take-off
We will use the same Geo-location specified in the load_sitl_conf.sh file to take off:

		rosservice call  /mavros/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 44.4928130, longitude: 11.3299817, altitude: 2.0}" 

In this simulation the Radio controller is not considered. For this reason, if you want to fly in the scene, you should change the default bahviour for the RC loss failsafe. You can use qgroundcontrol ground station to set the default value of NAV_RCL_ACT parameter to: Disabled.

##### Visualize ROS camera stream

To visualize the camera stream, you can use the _rqt_image_view_ tool from ROS selecting the correct camera topic: _/iris_with_camera/camera_raw/image_raw_ 


## Supported robots:
     
     * iris_with_camera
     * iris_with_sensors



