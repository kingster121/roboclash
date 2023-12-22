#include <array>
#include <iostream>
#include <stdlib.h>
#include <Normalise.h>

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
        turn = -normalise(x);
        speed = -normalise(y);

        // std::cout << "turn: " << turn << "\n"
        //           << std::flush;
        // std::cout << "speed: " << speed << "\n"
        //           << std::flush;

        // If the speed is near zero AND is turning, set speed to 0.5 (for spot-turn)
        if ((abs(speed) <= SPEED_SENSITIVITY) && (abs(turn) >= TURN_SENSITIVTY))
            speed = 0.5;

        // If turning joystick is on left side
        if (turn <= -TURN_SENSITIVTY)
        {
            l_motor_duty_cycle = speed * (1 - TURN_RATIO);
            r_motor_duty_cycle = speed;
        }
        // If turning joystick is on right
        else if (turn > TURN_SENSITIVTY)
        {
            l_motor_duty_cycle = speed;
            r_motor_duty_cycle = speed * (1 - TURN_RATIO);
        }
        // STRAIGHT AHEAD
        else
        {
            l_motor_duty_cycle = speed;
            r_motor_duty_cycle = speed;
        }

        return {l_motor_duty_cycle, r_motor_duty_cycle};
    }
};