# Generic ROS2 Interface for ESP32

This program exposes the hardware components of an ESP32 microcontroller to the ROS2 Ecosystem.
It allows a direct mapping from topic to pin, which can be dynamically configured at runtime.

## Overview

<img src="https://drive.google.com/uc?export=view&id=19eBBOYYDD7vcoYiBOruf9suX36mbpyZK" height="500">

This program is intended to be run on an ESP32 microcontroller.
On the ROS2 side, an Micro-ROS Agent has to be running to make the Microcontroller visible to the ROS2 Ecosystem.
The communication between Program and Agent is possible either via Wifi (UDP) or Serial connection (USB).

## Program Structure

The program consists of two layers, one for running the ROS2 node and one for managing the hardware access.

The ROS node is containing a publisher and subscriber for reading and writing to the pins of the microcrontroller, as well as a service for runtime configuration changes.

The board layer is managing the access and state of the hardware components and exposes functions to read/write the values from the pins and change the board configuration.
