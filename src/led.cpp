#include <Arduino.h>

int pin1 = 12;
int pin2 = 13;
int pin3 = 14;
int longRand;

const int R_MOTOR_CH = 0;

const int PWM_FREQ = 70;       // PWM frequency in Hz
const int PWM_RESOLUTION = 20; // Allows control over the granularity of the speed
const int MAX_PWM = pow(2, PWM_RESOLUTION);

void setup()
{
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin3, OUTPUT);

    ledcAttachPin(pin1, R_MOTOR_CH);
    digitalWrite(pin2, 1);
    digitalWrite(pin3, 1);
}

void loop()
{
    ledcSetup(R_MOTOR_CH, PWM_FREQ, PWM_RESOLUTION);
    longRand = random(0, 1000) / 1000.0;

    ledcWrite(R_MOTOR_CH, longRand);
}