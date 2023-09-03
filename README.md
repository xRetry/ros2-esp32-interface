# Generic ROS2 Interface for the ESP32 microcontroller

This program exposes the GPIO pins of an ESP32 microcontroller to the ROS2 ecosystem.
A ROS2 client can be used to read and write the values of the pins in real-time.
This is done by mapping the pin values to ROS2 topics, which can be dynamically configured at runtime.

The program is intended to be run on an ESP-WROOM-32 microcontroller.
On the ROS2 side, a Micro-ROS agent has to be running to make the microcontroller visible to the ROS2 ecosystem.
The communication between program and agent is done via Wi-Fi (UDP).

## Usage

### Building from Source

The interface does not come in a pre-compiled version, so it has to be build from source.
This section describes the entire build process for Ubuntu 20.04.
For other distributions, some steps may vary.

#### Espressif IDF

Fistly, the toolchain for ESP32 needs to be installed.
This part follows the official instructions, which can be found [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html)

To install the required system packages run the following terminal command:

    sudo apt update && apt install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

Then, clone the Espressif IDF repository:

    mkdir -p ~/esp
    cd ~/esp
    git clone -b v4.4.5 --recursive https://github.com/espressif/esp-idf.git

Run the following shell script to install additional tools:

    cd ~/esp/esp-idf
    ./install.sh esp32

Source the new environment variables:

    . $HOME/esp/esp-idf/export.sh

#### Micro-ROS

In the next part, the Micro-ROS component needs to be added.

Install required python packages:

    pip3 install catkin_pkg lark-parser empy colcon-common-extensions

Clone the Micro-ROS component repository:

    git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git ~/esp/esp-idf/components/micro_ros_espidf_component

Next, add the repository containing the required ROS2 messages:

    git clone https://github.com/xRetry/ros2-esp32-messages.git ~/esp/esp-idf/components/micro_ros_espidf_component/extra_packages/ros2-esp32-messages

#### ROS2-ESP32-Interface

Finally, this project can be cloned and compiled.

Clone the GitHub repository:

    git clone https://github.com/xRetry/ros2-esp32-interface.git
    cd ros2-esp32-interface

To configure various settings for the compilation use:

    idf.py menuconfig

Then, to build the interface and flash it to the device use:

    idf.py build flash

If you also want to monitor the console output after flashing it the microcontroller, use:

    idf.py build flash monitor

### Building with Docker 

To simplify this process, everything can be done using Docker and the provided Dockerfile.

Clone the GitHub repository:

    git clone https://github.com/xRetry/ros2-esp32-interface.git

Build the docker image using the Dockerfile inside the repository:

    cd ros2-esp32-interface
    docker build -t ros2-esp32 .

Then, to run the docker image, use:

    docker run --rm -it -v .:/ws -v /dev:/dev --net=host --privileged ros2-esp32 /bin/bash -c "idf.py menuconfig build flash"


## Communication

A ROS2 client is needed to interact with the ROS2-ESP32 Interface.
The configuration of the pins is done using a ROS2 service.
Alternatively, the [configuration tool](https://github.com/xRetry/esp32-config-tool) can be used.
Furthermore, a ROS2 publisher is required to send pin values to the microcontroller and a ROS2 subscription to receive values.
All message definitions can be found [here](https://github.com/xRetry/ros2-esp32-messages).

### Changing the Pin Configuration

When a new configuration is received via the ROS2 service, the currently active operating mode of a pin is changed based on the mode number in the service request.
The request contains an array of mode numbers, where the array index corresponds to GPIO pin number of the microcontroller.
The currently available pin operating modes are:

0: Disabled: The pin is not used. Received values are ignored and published values set to zero.
1: Digital Input: A pin is read as digital signal, with the value zero or one, and sent to other ROS2 clients.
2: Digital Output: The values received by the microcontroller is written as digital signal to the pins.
3: Analog Input: The program uses the ADC of the microcontroller to read the raw voltage and publishes the value.
4: Analog Output: Received values are converted to an analog pin signal using the microcontrollers DAC.

### Reading and Writing Pin Values

During normal operation, the ROS2-ESP32 Interface performs the read and write operations at a frequency defined by the `MICRO_ROS_REFRESH_RATE`, which can be changed before compilation.

