> NOTE: This is the `ros-humble` implementation. It contains how the `smc_ros_hw_plugin` is used with a `differntial drive robot`.
>
> It also shows how to set up `ros2-control`, `ros2-controllers` and the `controller-manager` to work with the smc driver module (check the `config` and `launch` folder).  

## How to quickly setup the `smc driver module` with the `smc_test_bot` using the `smc_ros2_hw_plugin` pkg
- ensure the **`smc_l298n_pid_driver`** module (with the motors connected and fully set up for velocity PID) is connected to the microcomputer or PC via USB.

- ensure you have successfullly install and build the [**`smc_ros2_hw_plugin`**](https://github.com/samuko-things-company/smc_ros_hw_plugin/tree/humble) package in your ros workspace and also sourced it.

- In the src/ folder of your ros workspace, clone the repo (or you can download and add it manually to the src/ folder)
  > ```git clone -b humble https://github.com/samuko-things-company/smc_test_bot.git```

- cd into the package folder (i.e `smc_test_bot`) and run rosdep to install any necessary ros dependencies
  > ```cd smc_test_bot```
  >
  > ```rosdep install --from-paths src --ignore-src -r -y```

- check the serial port the driver is connected to:
  > The best way to select the right serial port (if you are using multiple serial device) is to select by path
  >
  > ```ls /dev/serial/by-path```
  >
  > you should see a value (if the driver is connected and seen by the computer), your serial port would be -> /dev/serial/by-path/[value]. for more info visit this tutorial from [ArticulatedRobotics](https://www.youtube.com/watch?v=eJZXRncGaGM&list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT&index=8)

  - OR you can also try this:
  > ```ls /dev/ttyU*```
  >
  > you should see /dev/ttyUSB0 or /dev/ttyUSB1 and so on

- once you have gotten the port, update the port parameter in the `ros2_control` tag in the `robot_urdf.xacro` file with the discovered port in the previous step

- build the packages with colcon (in your ros workspace root folder):
  > ```colcon build --packages-select smc_test_bot --symlink-install``


## Test the smc_test_bot (with the smc_driver connected)
- ensure you source your ros workspace in the terminal

- view the robot in rviz with this command
```shell
$ ros2 launch smc_test_bot robot_test_view.launch.py
```

- start the smc_test_bot with the smc_ros_hw_plugin (using the controller manager package)
```shell
$ ros2 launch smc_test_bot robot_server_control.launch.py
```

- while the control server is running, you can view the robot with odometery data by runing the following in a new terminal
```shell
$ ros2 launch smc_test_bot robot_client_view.launch.py
```

- you can now control the motors via teleop and notice how the robot changes position in rviz (as in the video above)
> you can also use my [arrow_key_teleop_package](https://github.com/samuko-things/arrow_key_teleop_drive)
