# Generic ROS2 Interface for ESP32

This program exposes the hardware components of an ESP32 microcontroller to the ROS2 Ecosystem.
It allows a direct mapping from topic to pin, which can be dynamically configured at runtime.

## Overview
<img src="https://drive.google.com/uc?export=view&id=19eBBOYYDD7vcoYiBOruf9suX36mbpyZK" height="500">

This program is intended to be run on an ESP32 microcontroller.
It communicates with an Micro-ROS Agent either via Wifi (UDP) or Serial connection (USB).
On the ROS2 side, an Micro-ROS Agent has to be running to make the Microcontroller visible to the ROS2 Ecosystem.
