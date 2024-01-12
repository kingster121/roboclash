#include <Arduino.h>
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

//--------------- ESP32 Logic ----------------------//
void setup()
{
    Serial.begin(115200);

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
}

void loop()
{
    // Serial.println(sonar_front.ping_cm());
    float lspeed = random(-100, 100) / 100.0;
    float rspeed = random(-100, 100) / 100.0;
    long time = random(1000, 3000);

    moveMotors(lspeed, rspeed);
    delay(time);
    // if (state == 1)
    // {
    //     data.l_motor_duty_cycle = 0.15;
    //     data.r_motor_duty_cycle = -0.1;
    //     data.command = "Volcano turn";
    //     esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));
    //     delay(1000);

    //     state++;
    //     stop();
    //     delay(1000);
    // }
    // int *ultrasonic_dist_arr = get_ultrasonic_dist();
    // show_distance(ultrasonic_dist_arr);
    // data.l_motor_duty_cycle = 0.5;
    // data.r_motor_duty_cycle = 0.5;
    // data.command = "forward long";
    // data.angle = 0;
    // esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));

    // if (sonar_front.ping_cm() < 30 && state == 1){
    //     forward();}
    // stop();
    // data.command = "Forward ended";
    // Serial.println("Brain looping");
    // int *ultrasonic_dist_arr = get_ultrasonic_dist();
    // show_distance(ultrasonic_dist_arr);
    // data.command = "Does loop run?";
    // esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));

    // while (millis() - start_time < 180000)
    // {
    //     forward();
    //     turn_left();
    //     forward(2);
    //     turn_left();

    //     forward();
    //     turn_right();
    //     forward(2);
    //     turn_right();
    // }

    // Execute go home function
}

// Pseudo-code
/*
- Start by turning right 90 deg
- loop until either (time left 90s) OR (?):
    go straight until (front == true)
    turn left 90 deg or until (right== true)
    then move forward for 2s
    turn left 90 deg or until (back == true)
    go straight until (front == true)
    turn right 90 deg or until (left == true)
    then move forward for 2s
    turn right 90 deg or until (back == true)
*/

//--------------- Helper functions -----------------//
// int *get_ultrasonic_dist()
// {
//     static int distanceArray[4];
//     distanceArray[0] = sonar_right.ping_cm();
//     distanceArray[1] = sonar_front.ping_cm();
//     distanceArray[2] = sonar_left.ping_cm();
//     distanceArray[3] = sonar_back.ping_cm();

//     return distanceArray; // Returns pointer to the array
// }

// void show_distance(int *ultrasonic_dist_arr)
// {
//     for (int i = 0; i < 4; i++)
//     {
//         int dist = ultrasonic_dist_arr[i];
//         if (i == 0)
//             Serial.print("Right: ");
//         else if (i == 1)
//             Serial.print("\tFront: ");
//         else if (i == 2)
//             Serial.print("\tLeft: ");
//         else if (i == 3)
//             Serial.print("\tBack: ");

//         Serial.print(dist);
//         Serial.print("cm");
//     }
//     Serial.println();
// }

// float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
// {
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }
// // --------- End of Helper Functions ---------------//

// //-------------- Essential functions ---------------//
// void forward()
// {
//     Serial.println("Forward all the way");
//     data.l_motor_duty_cycle = 0.15;
//     data.r_motor_duty_cycle = 0.15;
//     data.command = "forward long";
//     esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));
// }

// void forward(unsigned long time)
// {
//     Serial.println("Foward for " + String(time));
//     data.l_motor_duty_cycle = 0.5;
//     data.r_motor_duty_cycle = 0.5;
//     data.command = "forward short";
//     esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));

//     compass.read();
//     int start_angle = compass.getAzimuth();
//     unsigned long start = millis();
//     while (millis() - start <= time)
//     {
//         data.angle = get_angle(start_angle);

//         float proportional = mapfloat(data.angle, -45, 45, -0.05, 0.05);

//         data.l_motor_duty_cycle += proportional;
//         data.r_motor_duty_cycle -= proportional;
//         esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));

//         delay(100);
//     }
// }

// void turn_left()
// {
//     Serial.println("Turn left");

//     data.l_motor_duty_cycle = 0;
//     data.r_motor_duty_cycle = 0.5;
//     data.command = "Turn left";
//     esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));

//     compass.read();
//     int start_angle = compass.getAzimuth();
//     int angle_offset = 0;
//     while (angle_offset < 90)
//     {
//         angle_offset = get_angle(start_angle);
//         delay(100);
//     }

//     stop();
// }
// void turn_right()
// {
//     Serial.print("Turning right");

//     data.l_motor_duty_cycle = 0.5;
//     data.r_motor_duty_cycle = 0;
//     data.command = "Turn right";
//     esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));

//     compass.read();
//     int start_angle = compass.getAzimuth();
//     int angle_offset = 0;
//     while (angle_offset > -90)
//     {
//         // Serial.print(".");
//         angle_offset = get_angle(start_angle);
//         data.angle = angle_offset;
//         esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));
//         // Serial.print(angle_offset);
//         delay(100);
//     }

//     stop();
// }
// void stop()
// {
//     data.l_motor_duty_cycle = 0;
//     data.r_motor_duty_cycle = 0;
//     data.command = "Stop";
//     esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));
// }

// void go_home()
// {
//     /*
//     1. Go north until
//     2. Front sense wall
//     3. Check if left senses wall also
//         3.1. If left sense no wall, turn left and go straight until front hits wall. Go back to (1).
//         3.2. Else already already at airlock, deposit loot and terminate programme
//     */

//     // Turn until facing north
// }
// // --------End of Essential functions --------------//