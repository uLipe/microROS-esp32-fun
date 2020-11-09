# Fun with micro-ROS on ESP32 devices

This <b>WIP-project</b> is a base project on how to create an ESP32 device application compatible with micro-ROS aka "ROS2 for microcontrollers". Different from 
regular micro-ROS applications, this project uses the <b>micro_ros_espidf_component</b> allowing to build micro-ROS firmware 
without having the micro-ROS setup project, that is it you don't need to run the build from the ROS2 side but from ESP-IDF side. 

---


# Getting started:

Before to use this example, you need to get the ESP-IDF running and install micro-ROS component for ESP-IDF depencies in your system, these are one-time process, please see the links below:

* ESP-IDF: https://github.com/espressif/esp-idf
* micro-ROS for ESP-IDF: https://github.com/micro-ROS/micro_ros_espidf_component

This project is out-of-tree, so the IDF and micro-ROS components are abstracted from here and you user, just needs to add your amazing micro-ROS compatible design inside the <b>main</b> component and edit the CMakeLists.txt there. 

Once you get the components just configure micro-ROS under micro-ROS menu settings by typing: 

```
$ idf.py menuconfig
```

You should see the menus below:

![Alt text](images/menuconfig.png?raw=true "menuconfig")
![Alt text](images/microros-menu.png?raw=true "micror-ros menu")

After to select the transport, over WiFi or serial-port, build:

```
$ idf.py build 
```

Since there is the micro-ros library build after regular idf build the first build process may take a while, since the build system will download the sources first.

---


# Testing :

For fast verification you can use one of the micro-ROS docker images, the microros/micro-ros-agent image has the micro-ROS Agent installed and ready to use. After get it from here: https://github.com/micro-ROS/docker , just run the micro-ROS Agent by using the command below (note: you need to have docker installed): 

```
$ docker run -it --rm -d /dev:/dev --privileged --net=host microros/micro-ros-agent:foxy
```

After that flash the firmware to the ESP32 using the commands from IDF:

```
$ idf.py flash monitor
```
This will also trigger the monitor, where you should see the messages received from Agent: 

![Alt text](images/microros-agent.png?raw=true "microros-agent")

---


# Interacting with ROS2:

The most interesting part start here, after micro-ROS agent gets running you can visualize the device's topic directly from ROS2, just run your ROS2 instance (and make sure the ROS2 instances are running in the same network), and use the popular <b>ros2 topic</b> tool, just type the command:

```
$ ros2 topic echo /esp32_talker_topic
```

This will print the esp32 publishing data in the topic directly in your ROS2 instance! 

![Alt text](images/rostopic.png?raw=true "rostopic")

From here you could move your amazing data read from sensors and forward to your ROS2 nodes and build some awesome robot application.

---


# Known limitations:

This code is not production-ready, and this project is in very WIP stage, so expect some breaking as long new features got added.

---