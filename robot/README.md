# COViRondelle2021 robot

Robot Python application for COViRondelle2021

 - Communicates with [`station`](../station) using ROS

## Dependencies

Using Docker prevents having to install the needed dependencies on the system. For testing purposes, it's really simpler.

Otherwise, here are some links for UNIX-based OS :

- [ROS](http://wiki.ros.org/Installation/Ubuntu)
- [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)
- [Tesseract](https://www.pyimagesearch.com/2017/07/03/installing-tesseract-for-ocr/)

## Installation

With Docker : 
```shell
docker build -t robot .
```

Without Docker : 
```shell
pip install -r requirements.txt
```

## Usage

With Docker :
```shell
docker run robot
```

Without Docker : 
```shell
python script_to_execute.py
```

With Docker Compose, from project's root : 
Connect after launching Docker-Compose (covirondelle2021_robot_1 is the name of running robot container, can vary, use `docker ps` to find container name)
```shell
docker exec -it covirondelle2021_robot_1 /bin/bash
```

## Contributing

Before contributing to the project, please read our [contribution guide](../CONTRIBUTING.md).

Create new ros package (If you forget a dependency you should create a new package with the good dependencies and copy the source code instead of trying to modify CMakeLists.txt and package.xml)
```shell
cd /root/catkin_ws/src
catkin_create_pkg <package-name> std_msgs rospy <other-dependencies>
```

Check code style of a single file
```shell
pylint module/script_to_check.py
```

Check code style of a module
```shell
pylint module
```

Run single test file
```shell
pytest tests/test_file.py
```

Run all test files
```shell
pytest tests
```