# Drone Apriltag code positioning and landing

## Tool

Onboard computer ubuntu18.04/20.04 

px4 firmware flight control

Flight platform

The on-board computer needs to be configured with ROS+mavros+apriltag_ros

mavros installation reference:

The configuration process is divided into source code installation and binary (apt) installation

Mainly install the camera driver and apriiltag recommended binary installation, simple and time-saving!



# Explanation of control code

config:
//Set the amount and several parameters of the remote control elevator and rudder lever
Set the search mode points x_move, y_move, that is, the points near the apriltag code, and modify the speed PID according to the actual size of the QR code and the flight altitude


//Process: Take off and rise to the flight altitude and then enter the search mode, relying on the elevator and rudder for movement control, if the identified apriltag code enters the automatic positioning and landing, the manual is invalid.

Process: The aircraft first flies to the search point, and then performs QR code tracking. If the apriltag code is not recognized for a long time, it enters the landing mode.


The default camera is installed in the geometric center of the plane of the aircraft, and the camera coordinate system points to the nose in the negative direction of y



2025.11.21

Added mode switching, follow mode and landing mode
