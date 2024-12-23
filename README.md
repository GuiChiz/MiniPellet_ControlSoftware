# MiniPellet_ControlSoftware
Mini Pellet Extruder for a 5Axis Desktop 3D Printer system - control software

The provided code is intended to be installed on the Arduino board responsible for controlling the extruder hardware. This code interfaces with a real-time program running on the PC, which handles the trajectory planning for the five axes and synchronizes the axis movements with the extruder control. The extruder dynamics, which represent the bottleneck in the overall motion system, are accounted for in the PC-side software.
