# [Python 範例程式](https://github.com/HsinChiaChen/Python_pub_sub) - Publisher 與 Subscriber
## Install ROS 2 Package
Open the new terminal and do these commands:
```shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/
git clone https://github.com/HsinChiaChen/Python_pub_sub.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=160" >> ~/.bashrc
```
## Run `publisher` Node
Open the new terminal and do this commands:
```shell
ros2 launch easy_pub_sub my_publisher.launch.py
```

## Run `subscriber` Node
Open the new terminal and do this commands:
```shell
ros2 launch easy_pub_sub my_subscriber.launch.py
```

## Else
1. Tree structure
    ```
    ros2_ws
    ├── build
    ├── install
    ├── log
    └── src
        └── Python_pub_sub
            ├── easy_pub_sub
            │   ├── CMakeLists.txt
            │   ├── config
            │   │   └── params.yaml
            │   ├── easy_pub_sub
            │   │   └── __init__.py
            │   ├── launch
            │   │   ├── my_publisher.launch.py
            │   │   └── my_subscriber.launch.py
            │   ├── package.xml
            │   └── scripts
            │       ├── publisher.py
            │       └── subscriber.py
            └── README.md
    ```
2. RQT
    ```
    rqt_grath
    ```
    

# Control
## P controller
Open the new terminal and do this commands:
```shell
ros2 launch easy_pub_sub P_controller.launch.py
```

## PID controller & Get Parameter
Open the new terminal and do these commands:
- send parameter
```shell
ros2 launch easy_pub_sub publisher_parameter.launch.py
```
- PID controller
```shell
ros2 launch easy_pub_sub PID_controller.launch.py
```
