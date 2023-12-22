#include <Movement.h>
#include <Arduino.h>

Movement myMovement;

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    std::array<float, 2> motors = myMovement.move(1000, 1500);

    // Print the motor values to the Serial monitor
    Serial.print("Left Motor Duty Cycle: ");
    Serial.println(motors[0]);

    Serial.print("Right Motor Duty Cycle: ");
    Serial.println(motors[1]);

    delay(2000);
}
