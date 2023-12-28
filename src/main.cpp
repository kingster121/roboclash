#include <PPMReader.h>
#include <Movement_Polar.h>

// Movement object - contains move method
Movement myMovement;

// Initialize a PPMReader on digital pin 3 with 6 expected channels.
int channel_arr[10] = {};
byte interruptPin = 13;
byte channelAmount = 10;
PPMReader ppm(interruptPin, channelAmount);

const int D0 = 33;
const int D1 = 25;
const int D2 = 26;
const int D3 = 27;

const int L_MOTOR_CH = 0; // Choose a PWM channel (0-15)
const int R_MOTOR_CH = 1;

const int PWM_FREQ = 70;       // PWM frequency in Hz
const int PWM_RESOLUTION = 20; // Allows control over the granularity of the speed
const int MAX_PWM = pow(2, PWM_RESOLUTION);

void moveMotors(float l_duty_cycle, float r_duty_cycle);

void setup()
{
    ledcSetup(L_MOTOR_CH, PWM_FREQ, PWM_RESOLUTION); // Configure PWM parameters for channel 0
    ledcSetup(R_MOTOR_CH, PWM_FREQ, PWM_RESOLUTION);
    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);
    Serial.begin(115200);
}

void loop()
{
    // Print latest valid values from all channels
    for (byte channel = 1; channel <= channelAmount; ++channel)
    {
        unsigned value = ppm.latestValidChannelValue(channel, 0);
        // Serial.print("C" + String(channel) + ": " + String(value) + "\t");
        channel_arr[channel - 1] = value;
    }
    Serial.println();
    delay(20);

    int speed = channel_arr[1];
    int turn = channel_arr[0];
    // Serial.println("y: " + String(speed));
    // Serial.println("x: " + String(turn));

    if (channel_arr[0] != 0)
    {
        std::array<float, 2> motor_duty_cycles = myMovement.move(turn, speed);
        float l_duty_cycle = motor_duty_cycles[0];
        float r_duty_cycle = motor_duty_cycles[1];
        moveMotors(l_duty_cycle, r_duty_cycle);
    }
}

// Powers the L_MOTOR and R_MOTOR.
// duty_cycle inputs go from -1 to 1
void moveMotors(float l_duty_cycle, float r_duty_cycle)
{
    if (l_duty_cycle < 0)
    {
        ledcDetachPin(D1);
        ledcAttachPin(D0, L_MOTOR_CH);
        digitalWrite(D1, 0);
    }
    else
    {
        ledcDetachPin(D0);
        ledcAttachPin(D1, L_MOTOR_CH);
        digitalWrite(D0, 0);
    }

    if (r_duty_cycle < 0)
    {
        ledcDetachPin(D2);
        ledcAttachPin(D3, R_MOTOR_CH);
        digitalWrite(D2, 0);
    }
    else
    {
        ledcDetachPin(D3);
        ledcAttachPin(D2, R_MOTOR_CH);
        digitalWrite(D3, 0);
    }

    l_duty_cycle = round(abs(l_duty_cycle) * MAX_PWM);
    r_duty_cycle = round(abs(r_duty_cycle) * MAX_PWM);

    // Serial.println("l_duty_cycle: " + String(l_duty_cycle));
    // Serial.println("r_duty_cycle: " + String(r_duty_cycle));

    ledcWrite(L_MOTOR_CH, l_duty_cycle);
    ledcWrite(R_MOTOR_CH, r_duty_cycle);
}

void moveMotor(int ch, long duty_cycle, bool reverse)
{
    bool s = duty_cycle > 0; // Checks the sign of duty_cycle

    // Using XOR, toggle direction with boolean reverse
    if (s ^ reverse)
    {
        ledcDetachPin(D2); // D0 is no longer connected to the PWM
        ledcAttachPin(D3, ch);
        digitalWrite(D2, 0);
    }
    else
    {
        ledcDetachPin(D3);
        ledcAttachPin(D2, ch);
        digitalWrite(D3, 0);
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
