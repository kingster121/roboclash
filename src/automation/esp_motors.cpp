#include <Arduino.h>
// #include <WiFi.h>
// #include <esp_now.h>
// #include <Wire.h>
// #include <SPI.h>

// int currentTime = 0;

// // RX adddress
// uint8_t broadcast_address1[] = {0x24, 0x62, 0xAB, 0xE0, 0xEF, 0xF0};

// // Structure example to receive data
// // Must match the sender structure
// typedef struct struct_message
// {
//     float l_motor_duty_cycle;
//     float r_motor_duty_cycle;
//     String command;
//     int angle;
// } struct_message;

// // Create a struct_message called myData
// struct_message data;

// esp_now_peer_info_t peerInfo;

// // callback function that will be executed when data is received
// void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

const int D0 = 21; // R_MOTOR
const int D1 = 19; // R_MOTOR
const int D2 = 18; // L_MOTOR
const int D3 = 5;  // L_MOTOR

// Spinners --- 4 and 5 are for front spinner --- 6 and 7 is for the top thigny
const int D4 = 23;
const int D5 = 22;
const int D6 = 1;
const int D7 = 3;

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
    pinMode(D4, OUTPUT);
    pinMode(D5, OUTPUT);
    pinMode(D6, OUTPUT);
    pinMode(D7, OUTPUT);

    // Front spinner will always spin inwards
    digitalWrite(D5, LOW);
    digitalWrite(D6, HIGH);

    Serial.begin(115200);
}

void loop()
{
    moveMotors
    // Serial.println("This is looping");
    // // Print latest valid values from all channels
    // for (byte channel = 1; channel <= channelAmount; ++channel)
    // {
    //     unsigned value = ppm.latestValidChannelValue(channel, 0);
    //     // Serial.print("C" + String(channel) + ": " + String(value) + "\t");
    //     channel_arr[channel - 1] = value;
    // }
    // // Serial.println();
    // delay(100);
    // delay(1000);
}

// Powers the L_MOTOR and R_MOTOR.
// duty_cycle inputs go from -1 to 1
void moveMotors(float l_duty_cycle, float r_duty_cycle)
{
    if (l_duty_cycle < 0)
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

    if (r_duty_cycle < 0)
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

    if (l_duty_cycle == 0)
        l_duty_cycle = 0;
    else
        l_duty_cycle = round(abs(l_duty_cycle) * MAX_PWM);

    if (r_duty_cycle == 0)
        r_duty_cycle = 0;
    else
        r_duty_cycle = round(abs(r_duty_cycle) * MAX_PWM);

    // Serial.println("l_duty_cycle: " + String(l_duty_cycle));
    // Serial.println("r_duty_cycle: " + String(r_duty_cycle));

    ledcWrite(L_MOTOR_CH, l_duty_cycle);
    ledcWrite(R_MOTOR_CH, r_duty_cycle);
}

// void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
// {

//     memcpy(&data, incomingData, sizeof(data));
//     // Serial.print("Bytes received: ");
//     // Serial.println(len);
//     // Serial.print("l_motor_duty_cycle: ");
//     // Serial.println(data.l_motor_duty_cycle);
//     // Serial.print("r_motor_duty_cycle: ");
//     // Serial.println(data.r_motor_duty_cycle);
//     // Serial.println();
//     // Serial.println("time: ");
//     // int pastTime = currentTime;
//     // pastTime = millis();
//     // Serial.println(String(pastTime - currentTime));

//     moveMotors(data.l_motor_duty_cycle, data.r_motor_duty_cycle);
//     delay(50);
//     moveMotors(0, 0);
//     delay(100);
// }