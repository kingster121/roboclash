#include <Arduino.h>
#include <array>
#include <iostream>
#include <stdlib.h>
#include <helper.h>

const float SPEED_SENSITIVITY = 0.2;
const float TURN_SENSITIVTY = 0.2;
const float TURN_RATIO = 0.2;

class Movement
{
public:
    float turn, speed;
    float l_motor_duty_cycle, r_motor_duty_cycle;

    // Speed is controlled by the forwardness/backwardness, turning is controlled by ratio determined by the left/rightness
    // Returns duty cycle of L_MOTOR and R_MOTOR
    std::array<float, 2> move(int x, int y)
    {
        x = map(x, 1000, 2000, 100, -100);
        y = map(y, 1000, 2000, 100, -100);

        float magnitude, angle;
        cartesianToPolar(x, y, magnitude, angle); // Edits magnitude and angle by pointer

        // std::cout << "x: " << x << "\n"
        //           << std::flush;
        // std::cout << "y: " << y << "\n"
        //           << std::flush;
        // std::cout << "magnitude: " << magnitude << "\n"
        //           << std::flush;
        // std::cout << "angle: " << angle << "\n"
        //           << std::flush;

        // Split the joystick into 4 quadrants to determine motor spin speed
        if (angle <= 90 && angle > 0)
        {
            l_motor_duty_cycle = 1.0;
            r_motor_duty_cycle = sin(angle / 180.0 * M_PI);
        }
        else if (angle <= 180 && angle > 90)
        {
            l_motor_duty_cycle = sin(angle / 180.0 * M_PI);
            r_motor_duty_cycle = 1.0;
        }
        else if (angle <= -90 && angle > -180)
        {
            l_motor_duty_cycle = sin(angle / 180.0 * M_PI);
            r_motor_duty_cycle = -1.0;
        }
        else
        {
            l_motor_duty_cycle = -1.0;
            r_motor_duty_cycle = sin(angle / 180.0 * M_PI);
        }

        l_motor_duty_cycle = l_motor_duty_cycle * magnitude / 100.0;
        r_motor_duty_cycle = r_motor_duty_cycle * magnitude / 100.0;
        return {l_motor_duty_cycle, r_motor_duty_cycle};
    }
};