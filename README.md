# Generic ROS2 Interface for ESP32

This program exposes the hardware components of an ESP32 microcontroller to the ROS2 Ecosystem.
It allows a direct mapping from topic to pin, which can be dynamically configured at runtime.
It is currently work-in-progress and far from finished.

## Feature List

- [ ] Runtime configuration of:
    - [x] Pin configuration
    - [ ] UDP and serial connection 
    - [ ] Sample rate
    - [ ] ROS2 topics
- [x] Lightweight read and write operations
- [x] Simple addtion of new read/write modes
- [ ] Robust error handling
- [ ] Configuration via file
- [ ] Configuration persistence after reboot

## Overview

This program is intended to be run on an ESP32 microcontroller.
On the ROS2 side, an Micro-ROS Agent has to be running to make the Microcontroller visible to the ROS2 Ecosystem.
The communication between Program and Agent is possible either via Wifi (UDP) or Serial connection (USB).

<img src="https://drive.google.com/uc?export=view&id=19eBBOYYDD7vcoYiBOruf9suX36mbpyZK" height="500">

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





