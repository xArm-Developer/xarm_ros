# Setting up ROS with virtualenv
We set up ROS with virtualenv because it is a better way to contain the python packages needed when running ROS.

# Installing virtualenv
You can install virtualenv by opening a terminal and running:
```sh
$ pip install virtualenv
```

# Setting up the virtual environment
Make and navigate to a directory folder where you will keep your virtual environment folders, then run:
```sh
$ virtualenv -p /usr/bin/python2.7 ros_py27
$ source ros_py27/bin/activate
```
[Please note for the following steps we assume that you already have the xarm_ros repo cloned]
Navigate to xarm_ros/examples/xarm7_redundancy_res, and run:
```sh
(your-venv)$ pip install -r requirements.txt
```
This will install the python dependencies needed for the robot_jogging.py script.

You can check that the requirements are installed properly by running:
```sh
(your-venv)$ pip freeze
```

# Running Instructions
In the below steps we assume: (1) that you’ve set up a loadable virtual env and (2) that you’ve built the xarm_ros package using catkin_make

In a terminal, run:
```sh
$ source /opt/ros/your_ros_version/setup.bash
$ source your_ws/devel/setup.bash
$ roslaunch xarm_gazebo xarm7_beside_table.launch
```

In a second terminal, run
```sh
$ source /opt/ros/your_ros_version/setup.bash
$ source your_ws/devel/setup.bash
$ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
```

Using the Rviz move_it interface, move the xarm to a position you are happy with. Then, in a third terminal, run:
```sh
$ source /opt/ros/your_ros_version/setup.bash
$ source your_ws/devel/setup.bash
$ source  ros_py27/bin/activate
(your-venv)$ rosrun xarm7_redundancy_res robot_jogging.py -q 30.0 -i 1 -a -1.1
```
In the above command line, the -q option is the desired angle change, the -i option indicates the ith joint, and the -a option is the stepsize alpha.

Alternatively, robot_jogging.py can be run within a python shell, allowing you to make function calls from the shell to the jog and redundancy_resolution functions defined within robot_jogging.py. To do this alternative method, open the third terminal and run:
```sh
$ source /opt/ros/your_ros_version/setup.bash
$ source your_ws/devel/setup.bash
$ source  ros_py27/bin/activate
(your-venv)$ python
>>> execfile('PATH_TO_YOUR_WS/src/xarm_ros/examples/xarm7_redundancy_res/scripts/robot_jogging.py')
>>> jog(0.05, 0, 0, 0, 0, 0, client)
>>> redundancy_resolution(3.14/6, 1, -1.0, client)
```

# Creating the requirements.txt file for new packages
All required python libraries should be installed using pip while the virtual environment is activated.

With the virtual environment created and activated, run:
```sh
(your-venv)$ pip install pip --upgrade
(your-venv)$ pip install -U rosdep rosinstall_generator wstool rosinstall
(your-venv)$ pip install --upgrade setuptools
(your-venv)$ pip install defusedxml
```
Open a new terminal without activating the ros and catkin setup.bash files. Only activate the virtual environment and then navigate to your project’s directory. Run:
```sh
(your-venv)$ pip freeze > requirements.txt
```
The requirements.txt file should be created with all the python dependencies.
