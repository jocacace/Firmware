sudo apt install python3-pip
pip3 install --user empy
pip3 install --user toml
pip3 install --user numpy
pip3 install --user packaging
sudo apt-get install libgstreamer-plugins-base1.0-dev
pip3 install --user jinja2
make px4_sitl_default 
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-msgs 
sudo /opt/ros/melodic/lib/mavros/install_geographiclib_datasets.sh

Compile:

make px4_sitl_default
make px4_sitl_default gazebo
