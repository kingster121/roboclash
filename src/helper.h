#include <math.h>

void cartesianToPolar(int xSquare, int ySquare, float &magnitude, float &angle)
{

    // Convert square to circle cartesian
    float xCircle = xSquare * sqrt(1 - 0.5 * pow((ySquare / 100.0), 2));
    float yCircle = ySquare * sqrt(1 - 0.5 * pow((xSquare / 100.0), 2));

    // Calculate magnitude using Pythagorean theorem
    // std::cout << "xCircle: " << xCircle << "\n"
    //           << std::flush;
    // std::cout << "yCricle: " << yCircle << "\n"
    //           << std::flush;
    magnitude = sqrt(xCircle * xCircle + yCircle * yCircle);
    // std::cout << "magnitude: " << magnitude << "\n"
    //           << std::flush;

    // Calculate angle using arctangent (atan2) function
    angle = atan2(yCircle, xCircle);

    // Convert angle from radians to degrees if needed
    angle = angle * 180.0 / M_PI;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}