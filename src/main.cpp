#include <PPMReader.h>
#include <helper.h>
#include <Arduino.h>
// #include <FastLED.h>

// #define LED_PIN     33 // led strip
// #define NUM_LEDS    17

// CRGB leds[NUM_LEDS];
// #include <Movement_Polar.h>

// Library for web monitoring
// #include "BluetoothSerial.h"
// BluetoothSerial SerialBT;

// Movement object - contains move method
// Movement myMovement;

// Initialize a PPMReader on digital pin 3 with 6 expected channels.
int channel_arr[10] = {};
byte interruptPin = 34;
byte channelAmount = 10;
PPMReader ppm(interruptPin, channelAmount);

const int D0 = 21; // R_MOTOR
const int D1 = 19; // R_MOTOR
const int D2 = 18; // L_MOTOR
const int D3 = 17; // L_MOTOR
const int D4 = 23; // ROTOR
const int D5 = 22; // ROTOR

const int L_MOTOR_CH = 0; // Choose a PWM channel (0-15)
const int R_MOTOR_CH = 1;

const int PWM_FREQ = 70;       // PWM frequency in Hz
const int PWM_RESOLUTION = 20; // Allows control over the granularity of the speed
const int MAX_PWM = pow(2, PWM_RESOLUTION);

void moveMotors(float l_duty_cycle, float r_duty_cycle);
void moveRotors(float rotor_duty_cycle);

void setup()
{
    ledcSetup(L_MOTOR_CH, PWM_FREQ, PWM_RESOLUTION); // Configure PWM parameters for channel 0
    ledcSetup(R_MOTOR_CH, PWM_FREQ, PWM_RESOLUTION);
    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    pinMode(D2, OUTPUT);
    pinMode(D3, OUTPUT);
    pinMode(D4, OUTPUT);
    pinMode(D5, OUTPUT);

    // FastLED.addLeds<WS2812, LED_PIN, RGB>(leds, NUM_LEDS);

    Serial.begin(115200);

    moveMotors(0.0,0.0);
    delay(1000);
}

void loop()
{
    // resting channel_arr[2] = 1500;
    // resting channel_arr[1] = 1568;
    // Print latest valid values from all channels
    for (byte channel = 1; channel <= channelAmount; ++channel)
    {
        unsigned value = ppm.latestValidChannelValue(channel, 0);
        // Serial.print("C" + String(channel) + ": " + String(value) + "\t");
        channel_arr[channel - 1] = value;
    }
    // Serial.println();
    delay(100);
    
    float l_duty_cycle = (channel_arr[2] - 1500); // result -500 to 500, middle 0
    float r_duty_cycle = (channel_arr[1] - 1568); // result -568 to 432, middle 0
    
    // 0-375 = 0-0.5, 375-500 = 0.5-1.0
    // -568--409 = -1--0.5, -409-0 = -0.5-0, -0-341 = 0-0.5, 341-432 = 0.5-1.0
    if (abs(l_duty_cycle) < 375){
        l_duty_cycle = mapfloat(l_duty_cycle, -375, 375, -0.5, 0.5);
    }
    else if (l_duty_cycle > 375){
        l_duty_cycle = mapfloat(l_duty_cycle, 375, 500, 0.5, 1);
    }
    else if (l_duty_cycle <= -375){
        l_duty_cycle = mapfloat(l_duty_cycle, -500, -375, -1, -0.5);
    }

    if (r_duty_cycle < -409){
        r_duty_cycle = mapfloat(r_duty_cycle, -568, -409, -1, -0.5);
    }
    else if (r_duty_cycle < 0){
        r_duty_cycle = mapfloat(r_duty_cycle, -409, 0, -0.5, 0);
    }
    else if (r_duty_cycle < 341){
        r_duty_cycle = mapfloat(r_duty_cycle, 0, 341, 0, 0.5);
    }    
    else {
        r_duty_cycle = mapfloat(r_duty_cycle, 341, 568, 0.5, 1);
    }
        
    // Serial.println(l_duty_cycle);
    // Serial.println(r_duty_cycle);

    float divide_speed = channel_arr[4];
    // divide_speed = mapfloat(divide_speed, 1000, 2000, 0, 4);
    // divide_speed = constrain(divide_speed, 1, 4);

    // l_duty_cycle = l_duty_cycle / divide_speed;
    // r_duty_cycle = r_duty_cycle / divide_speed;
    if (divide_speed == 1000){
        l_duty_cycle = l_duty_cycle / 8;
        r_duty_cycle = r_duty_cycle / 8;
    }
    else if (divide_speed == 2000){
        l_duty_cycle = l_duty_cycle / 4;
        r_duty_cycle = r_duty_cycle / 4;
    }
    else {
        l_duty_cycle = l_duty_cycle / 2;
        r_duty_cycle = r_duty_cycle / 2;
    }

    // main moving code
    if ((l_duty_cycle != 0) || (r_duty_cycle != 0))
    {
        moveMotors(l_duty_cycle, r_duty_cycle);
    }

    // rotor move inwards/outwards:
    float rotor_direction = channel_arr[5];
    rotor_direction = mapfloat(rotor_direction, 1000, 2000, -1, 1);

    if (rotor_direction < 0) {
        digitalWrite(D4, 0);
        digitalWrite(D5, 0);

    }
    else if (rotor_direction > 0) {
        digitalWrite(D4, 0);
        digitalWrite(D5, 1);        
    }
    else {
        digitalWrite(D4, 1);
        digitalWrite(D5, 0);       
    }


    // for (int i = 0; i <= 17; i++) {
    //     int colour1 = rand() % 256;
    //     int colour2 = rand() % 256;
    //     int colour3 = rand() % 256;
    //     leds[i] = CRGB (colour1, colour2, colour3);
    // }
    // FastLED.show();
}


// Powers the L_MOTOR and R_MOTOR.
// duty_cycle inputs go from -1 to 1
void moveMotors(float l_duty_cycle, float r_duty_cycle)
{
    if (l_duty_cycle > (0 + 0.05))
    {
        //left motor move backwards
        ledcDetachPin(D2);
        ledcAttachPin(D3, L_MOTOR_CH);
        digitalWrite(D2, 0);
    }
    else if (l_duty_cycle < (0 - 0.05))
    {
        //left motor move forwards
        ledcDetachPin(D3);
        ledcAttachPin(D2, L_MOTOR_CH);
        digitalWrite(D3, 0);
    }

    if (r_duty_cycle > (0 + 0.05))
    {
        //right motor move backwards
        ledcDetachPin(D1);
        ledcAttachPin(D0, R_MOTOR_CH);
        digitalWrite(D1, 0);
    }
    else if (r_duty_cycle < (0 - 0.05))
    {
        //right motor move forwards
        ledcDetachPin(D0);
        ledcAttachPin(D1, R_MOTOR_CH);
        digitalWrite(D0, 0);
    }

    l_duty_cycle = round(abs(l_duty_cycle) * MAX_PWM);
    r_duty_cycle = round(abs(r_duty_cycle) * MAX_PWM);

    // Serial.println("l_duty_cycle: " + String(l_duty_cycle));
    // Serial.println("r_duty_cycle: " + String(r_duty_cycle));

    ledcWrite(L_MOTOR_CH, l_duty_cycle);
    ledcWrite(R_MOTOR_CH, r_duty_cycle);
    
}
