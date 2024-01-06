#include <Arduino.h>
#include <driver/ledc.h>

const int D0 = 27;
const int D1 = 26;

const int L_MOTOR_CH = 0;      // Choose a PWM channel (0-15)
const int PWM_FREQ = 20000;    // PWM frequency in Hz
const int PWM_RESOLUTION = 11; // Allows control over the granularity of the speed
const int MAX_PWM = pow(2, PWM_RESOLUTION);

// Note that duty_cycle comes in percentage
void moveMotor(int ch, long duty_cycle, bool reverse);

void setup()
{
    // ledcSetup(L_MOTOR_CH, PWM_FREQ, PWM_RESOLUTION); // Configure PWM parameters for channel 0
    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    Serial.begin(115200);

    // float t = 0;
    // const float f = 0.5;
    // while (1)
    // {
    //     moveMotor(L_MOTOR_CH, 100 * sin(2 * PI * f * t), false);
    //     delay(1);
    //     t += 0.001;
    // }
}

void loop()
{
    int a = 0;
    Serial.print(a);
    digitalWrite(D0, HIGH);
    digitalWrite(D1, LOW);
}

// ch: PWM Channel
// duty_cycle: -100 to 100
// reverse: if true, reverse
void moveMotor(int ch, long duty_cycle, bool reverse)
{
    bool s = duty_cycle > 0; // Checks the sign of duty_cycle

    // Using XOR, toggle direction with boolean reverse
    if (s ^ reverse)
    {
        ledcDetachPin(D0); // D0 is no longer connected to the PWM
        ledcAttachPin(D1, L_MOTOR_CH);
        digitalWrite(D0, 0);
    }
    else
    {
        ledcDetachPin(D1);
        ledcAttachPin(D0, L_MOTOR_CH);
        digitalWrite(D1, 0);
    }

    duty_cycle = abs(duty_cycle);
    if (duty_cycle > 100)
        duty_cycle = 100;

    // Map the duty cycle between the (0 to 2047)
    duty_cycle *= MAX_PWM;
    duty_cycle /= 100;

    // Apply finalized duty_cycle value to the PWM channel
    ledcWrite(ch, duty_cycle);
}
