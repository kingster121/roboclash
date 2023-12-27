## Manual control
Go to src and run main.cpp
- helper.h contains functinos that convert joystick square cartesian coordinates to circular polar coordinates
- Movement_Polar.h contains *Movement* class
    - move() method takes in raw input from joystick's 2 channels and return the magnitude and angle (after conversion to circular polar)
- main.cpp contains the overall logic

## Automation
There will be 2 ESPs being ran using UART
- esp_brain.cpp will receive information from the sensors
    - 4 ultrasonic, 1 on each side
    - 1 compass
    - (Testing) 1 IMU for checking if robot is moving straight using PID
- esp_drivers.cpp will use the information from the sensors to tell the motors what to do
    - Wheels (2 motors)
    - Door for letting small white rocks out (1 motor)