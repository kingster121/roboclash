#include <math.h>

float normalise(int value)
{
    float norm = (value - 1500) / 500.0;
    return norm;
}

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