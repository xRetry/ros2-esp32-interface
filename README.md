# Generic ROS2 Interface for ESP32

This program exposes the hardware components of an ESP32 microcontroller to the ROS2 Ecosystem.
It allows a direct mapping from topic to pin, which can be dynamically configured at runtime.

## Feature List

- [x] Runtime configuration of the boards pins
- [x] Lightweight read and write operations
- [x] Simple addtion of new read/write modes
- [x] Robust error handling
- [x] Configuration via file

## Overview

This program is intended to be run on an ESP32 microcontroller.
On the ROS2 side, an Micro-ROS Agent has to be running to make the microcontroller visible to the ROS2 ecosystem.
The communicaton between program and agent is done via Wifi (UDP).

<img src="https://drive.google.com/uc?export=view&id=19eBBOYYDD7vcoYiBOruf9suX36mbpyZK" height="500">

## Usage

### Building from Source

The only way to use the interface is to build it from source.
This section describes the entire process for Ubuntu.
Some parts may vary depending on the distribution.

Fistly, the toolchain for ESP32 needs to be installed.
This part follows the official instructions, which can be found [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html)

Install the required packages:

    sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

Clone the ESP-IDF repository:

    mkdir -p ~/esp
    cd ~/esp
    git clone -b release/v4.4 --recursive https://github.com/espressif/esp-idf.git

Install additional tools:

    cd ~/esp/esp-idf
    ./install.sh esp32

Set up the environment variables:

    . $HOME/esp/esp-idf/export.sh

In the next part, the micro-ROS component for ESP-IDF needs to be set up.

Install additional dependencies:

    pip3 install catkin_pkg lark-parser empy colcon-common-extensions

Clone the micro-ROS component for ESP-IDF repository:

    git clone https://github.com/micro-ROS/micro_ros_espidf_component.git ~/esp/esp-idf/components/micro_ros_espidf_component

Next, the repository containting the ROS2 messsages need to cloned:

    git clone https://github.com/xRetry/ros2-esp32-interfaces.git ~/esp/idf/components/extra_packages/ros2-esp32-interfaces

Finally, this project can be cloned.

Clone the GitHub repository:

    cd ~/esp/esp-idf/components
    git clone https://github.com/xRetry/ros2-esp32.git ~/esp/esp-idf/components/ros2-esp32
    cd ros2-esp32

To configure the compilation settings use:

    idf.py menuconfig

Then, to build the interface and flash it to the device use:

    idf.py build flash

If you also want to monitor the console output after flashing it the microcontroller, use:

    idf.py build flash monitor

### Building with Docker 

To simplify this, the entire process can be done using Docker.

Clone the GitHub repository:

    git clone https://github.com/xRetry/ros2-esp32.git

Build the docker image using the Dockerfile inside the respository:

    cd ros2-esp32
    docker build -t ros2-esp32 .

Run the docker image:

    docker run --rm -v .:/ws -v /dev:/dev --net=host ros2-esp32 /bin/bash -c "idf.py menuconfig build flash monitor"

## Program Structure

The program consists of two layers, one for running the ROS2 node and one for managing the hardware access.

The ROS node is containing a publisher and subscriber for reading and writing to the pins of the microcrontroller, as well as a service for runtime configuration changes.

The board layer is managing the access and state of the hardware components and exposes functions to read/write the values from the pins and change the board configuration.

### Runtime Configuration

When a new configuraion is received via the ROS service, depending on the mode number, the corresponding mode-activation function is called.
The mode-activation function is responsible for correctly initializing the hardware and updating the board state.
Specifically, it defines the direction of the mode (input or output) as well as adding the function pointer for the read/write function.

### Reading and Writing

Reading and writing from and to the pins of the microcontroller is done at the refresh rate of the ROS node.
A loop goes over all pins and calls the correct read/write function depending on the configured pin direction.

## Adding new read/write modes

So far, the program contains modes for reading and writing digital and analog signals.
The functionality can be easily expanded thrugh the following steps:

1. Provide a new function inside the `modes.c` file, which writes or reads the values of a specific pin. 
It is called at the refresh rate of the ROS node and should therefore be lightweight.
From inside the function it is also possible to access the global `board` struct, which can be used to store handles or other stateful information.
For concurrency safety, only read access to the struct is advised.

2. Provide a function to activate the new mode inside the `modes.c` file. 
It gets called whenever this mode is selected for a specific pin. 
As with read/write function, it has access to the `board` struct but is also allowed to make mutable changes.

3. Add the new mode to the `pin_mode_t` enum inside the `modes.h` file

4. Add a pointer to the function which activates the new mode (from point 2) to the `PIN_MODE_FUNCTIONS` array inside `modes.c`

5. Add the direction (Input or Output) to the `PIN_DIRECTIONS` array in `modes.c`

## Configuration File

The `esp32-config-tool` can be used to change the configuration of the board.
It can be executed in the command line with the addition of a file path.
    
    esp32-config-tool config.yml

The configuration file itself is formatted as YAML.
This is what an example configuration looks like:

    transport: 
        type: udp
        agent_ip: 127.0.0.1
        agent_port: 0.0.0.0
        wifi_ssid: abcd
        wifi_password: 1234
        
    pins:
      - number: 1
        mode: analog_output
      - number: 2
        mode: digital_input

    topics:
        service: /esp32_service
        publisher: /esp32_pub
        subscriber: /esp32_sub
        
## Testing Framework

Unit tests are implemented using the Ceedling framework by mocking hardware components.
To run all tests, execute `ceedling test` in the command line.
