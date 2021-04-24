# COViRondelle2021 station

Station Python application for COViRondelle2021

 - Communicates with [`robot`](../robot) using ROS
 - Communicates with [`frontend`](../frontend) using websockets

## Dependencies

Using Docker prevents having to install the needed dependencies on the system. For testing purposes, it's really simpler.

Otherwise, here are some links for UNIX-based OS :

- [ROS](http://wiki.ros.org/Installation/Ubuntu)
- [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html)

## Installation

With Docker :
```shell
docker build -t station .
```

Without Docker :
```shell
pip install -r requirements.txt
```

## Usage

With Docker :
```shell
docker run station
```

Without Docker : 
```shell
python script_to_execute.py
```

With Docker Compose, from project's root : 
Connect after launching Docker-Compose (covirondelle2021_station_1 is the name of running station container, can vary, use `docker ps` to find container name)
```shell
docker exec -it covirondelle2021_station_1 /bin/bash
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
pylint module/file_to_check.py
```

Check code style of a module
```shell
pylint module
```

Run single test file within a module
```shell
pytest module/tests/test_file.py
```

Run all test files within a module
```shell
pytest module
```

ROS isn't suited for unit testing. To test individual handlers that subscribe or publish thing, the best strategy is to start the chain of commands and mock the required nodes. For instance, this is how we test "ready" and "start_cycle" topics reception : 

Terminal 1
```shell
docker-compose up
# Open frontend in browser
```

Terminal 2
```shell
docker exec -it covirondelle2021_station_1 /bin/bash
roscore
```

Terminal 3
```shell
docker exec -it covirondelle2021_station_1 /bin/bash
cd src/websockets
python3 main
```

Terminal 4
```shell
docker exec -it covirondelle2021_station_1 /bin/bash
cd src/controller
python main
```

Terminal 5
```shell
docker exec -it covirondelle2021_station_1 /bin/bash
cd src/controller
python mock_robot_consumption.py
```

Then, you can print in the required handler that the topic was received. You can also test "start_cycle" reception by pressing the button in the frontend, which will be handled by the websocket and will publish the correct topic.