// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <esp_now.h>
#include <WiFi.h>

// control pins for left and right motors
const float leftSpeed = 0; // means pin 9 on the Arduino controls the speed of left motor
const float rightSpeed = 0;

const int MPU = 0x68;                                                      // MPU6050 I2C address
float AccX, AccY, AccZ;                                                    // linear acceleration
float GyroX, GyroY, GyroZ;                                                 // angular velocity
float accAngleX, accAngleY, accAngleZ, gyroAngleX, gyroAngleY, gyroAngleZ; // used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const float maxSpeed = 0.3; // max PWM value written to motor speed pin. It is typically 255.
const float minSpeed = 0.1; // min PWM value at which motor moves
int angle;                  // due to how I orientated my MPU6050 on my car, angle = roll
int targetAngle = 0;
int equilibriumSpeed = 248; // rough estimate of PWM at the speed pin of the stronger motor, while driving straight
// and weaker motor at maxSpeed
float leftSpeedVal;
float rightSpeedVal;
bool isDriving = false;    // it the car driving forward OR rotate/stationary
bool prevIsDriving = true; // equals isDriving in the previous iteration of void loop()
bool paused = false;       // is the program paused

// (RX) ESP MAC address
uint8_t broadcast_address1[] = {0xB0, 0xA7, 0x32, 0x2B, 0x6E, 0x24};
uint8_t broadcast_address2[] = {0x24, 0x62, 0xAB, 0xE0, 0xEF, 0xF0};

// ESP NOW message struct
typedef struct struct_message
{
    float l_motor_duty_cycle;
    float r_motor_duty_cycle;
    String command;
    int angle;
} struct_message;

struct_message data;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // char macStr[18];
    // Serial.print("Packet to: ");
    // // Copies the sender mac address to a string
    // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
    //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    // Serial.print(macStr);
    // Serial.print(" send status:\t");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    // Serial.println("l_motor: " + String(data.l_motor_duty_cycle));
    // Serial.println("r_motor: " + String(data.r_motor_duty_cycle));
    // Serial.println("command: " + String(data.command));
    // Serial.println("angle: " + String(data.angle));
}

Adafruit_MPU6050 mpu;

// Functions
void readIMU();
void calculateError();
float changeSpeed(float motorSpeed, float increment);
void controlSpeed();
void driving();

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Adafruit MPU6050 test!");

    // Try to initialize!
    // if (!mpu.begin())
    // {
    //     Serial.println("Failed to find MPU6050 chip");
    //     while (1)
    //     {
    //         delay(10);
    //     }
    // }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange())
    {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange())
    {
    case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
    case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
    case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
    case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth())
    {
    case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
    case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
    case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
    case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
    case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
    case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
    case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }

    Serial.println("");
    delay(100);

    calculateError();
    delay(20);
    currentTime = micros();
}

void loop()
{
    readIMU();

    // ------- Caculate roll and pitch from acceleration -------- //
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX is calculated in the calculateError() function
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

    previousTime = currentTime;
    currentTime = micros();
    elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds

    // Correct the outputs with the calculated error values
    GyroX -= GyroErrorX; // GyroErrorX is calculated in the calculateError() function
    GyroY -= GyroErrorY;
    GyroZ -= GyroErrorZ;

    // Serial.println("accAngleX: " + String(accAngleX) + "accAngleY: " + String(accAngleY));
    // Serial.println("GyroX: " + String(GyroX) + "GyroY: " + String(GyroY) + "GyroZ: " + String(GyroZ));

    delay(100);
    // // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY += GyroY * elapsedTime;
    yaw += GyroZ * elapsedTime;
    // // // combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    angle = roll; // if you mounted MPU6050 in a different orientation to me, angle may not = roll. It can roll, pitch, yaw or minus version of the three
    //               // for me, turning right reduces angle. Turning left increases angle.

    // // Forward
    targetAngle = 0;

    driving();

    // // Print the values on the serial monitor
    // if (Serial.available())
    // {
    //     char c = Serial.read();
    //     if (c == 'w')
    //     { // drive forward
    //         Serial.println("forward");
    //         isDriving = true;
    //     }
    //     else if (c == 'a')
    //     { // turn left
    //         Serial.println("left");
    //         targetAngle += 90;
    //         if (targetAngle > 180)
    //         {
    //             targetAngle -= 360;
    //         }
    //         isDriving = false;
    //     }
    //     else if (c == 'd')
    //     { // turn right
    //         Serial.println("right");
    //         targetAngle -= 90;
    //         if (targetAngle <= -180)
    //         {
    //             targetAngle += 360;
    //         }
    //         isDriving = false;
    //     }
    //     else if (c == 'q')
    //     { // stop or brake
    //         Serial.println("stop");
    //         isDriving = false;
    //     }
    //     else if (c == 'i')
    //     { // print information. When car is stationary, GyroX should approx. = 0.
    //         Serial.print("angle: ");
    //         Serial.println(angle);
    //         Serial.print("targetAngle: ");
    //         Serial.println(targetAngle);
    //         Serial.print("GyroX: ");
    //         Serial.println(GyroX);
    //         Serial.print("elapsedTime (in ms): "); // estimates time to run void loop() once
    //         Serial.println(elapsedTime * pow(10, 3));
    //         Serial.print("equilibriumSpeed: ");
    //         Serial.println(equilibriumSpeed);
    //     }
    //     else if (c == 'p')
    //     { // pause the program
    //         paused = !paused;
    //         stopCar();
    //         isDriving = false;
    //         Serial.println("key p was pressed, which pauses/unpauses the program");
    //     }
    // }

    // static int count;
    // static int countStraight;
    // if (count < 6)
    // {
    //     count++;
    // }
    // else
    // { // runs once after void loop() runs 7 times. void loop runs about every 2.8ms, so this else condition runs every 19.6ms or 50 times/second
    //     count = 0;
    //     if (!paused)
    //     {
    //         if (isDriving != prevIsDriving)
    //         {
    //             leftSpeedVal = equilibriumSpeed;
    //             countStraight = 0;
    //             Serial.print("mode changed, isDriving: ");
    //             Serial.println(isDriving);
    //         }
    //         if (isDriving)
    //         {
    //             if (abs(targetAngle - angle) < 3)
    //             {
    //                 if (countStraight < 20)
    //                 {
    //                     countStraight++;
    //                 }
    //                 else
    //                 {
    //                     countStraight = 0;
    //                     equilibriumSpeed = leftSpeedVal; // to find equilibrium speed, 20 consecutive readings need to indicate car is going straight
    //                     Serial.print("EQUILIBRIUM reached, equilibriumSpeed: ");
    //                     Serial.println(equilibriumSpeed);
    //                 }
    //             }
    //             else
    //             {
    //                 countStraight = 0;
    //             }
    //             driving();
    //         }
    //         else
    //         {
    //             rotate();
    //         }
    //         prevIsDriving = isDriving;
    //     }
    // }
}

void driving()
{                                         // called by void loop(), which isDriving = true
    int deltaAngle = targetAngle - angle; // rounding is neccessary, since you never get exact values in reality
    // forward();
    data.l_motor_duty_cycle = leftSpeedVal;
    data.r_motor_duty_cycle = rightSpeedVal;
    data.command = "Forward";
    data.angle = deltaAngle;
    esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));

    Serial.println("L_MOTOR: " + String(data.l_motor_duty_cycle) + "\tR_MOTOR: " + String(data.r_motor_duty_cycle));
    if (deltaAngle != 0)
    {
        controlSpeed();
        // data.l_motor_duty_cycle = leftSpeedVal;
        // data.r_motor_duty_cycle = rightSpeedVal;
        // data.command = "Forward";
        // data.angle = deltaAngle;
        // esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));

        // Serial.println("L_MOTOR: " + String(data.l_motor_duty_cycle) + "\tR_MOTOR: " + String(data.r_motor_duty_cycle));
    }
}

void controlSpeed()
{ // this function is called by driving ()
    int deltaAngle = round(targetAngle - angle);
    int targetGyroX;

    // setting up propoertional control, see Step 3 on the website
    if (deltaAngle > 30)
        targetGyroX = 60;
    else if (deltaAngle < -30)
        targetGyroX = -60;
    else
        targetGyroX = 2 * deltaAngle;

    if (round(targetGyroX - GyroX) == 0)
        ;
    else if (targetGyroX > GyroX)
    {
        leftSpeedVal = changeSpeed(leftSpeedVal, -0.001); // would increase GyroX
        rightSpeedVal = changeSpeed(rightSpeed, 0.001);
    }
    else
    {
        leftSpeedVal = changeSpeed(leftSpeedVal, 0.001);
        rightSpeedVal = changeSpeed(leftSpeedVal, -0.001);
    }
}

// void rotate()
// { // called by void loop(), which isDriving = false
//     int deltaAngle = round(targetAngle - angle);
//     int targetGyroX;
//     if (abs(deltaAngle) <= 1)
//     {
//         stopCar();
//     }
//     else
//     {
//         if (angle > targetAngle)
//         { // turn left
//             left();
//         }
//         else if (angle < targetAngle)
//         { // turn right
//             right();
//         }

//         // setting up propoertional control, see Step 3 on the website
//         if (abs(deltaAngle) > 30)
//         {
//             targetGyroX = 60;
//         }
//         else
//         {
//             targetGyroX = 2 * abs(deltaAngle);
//         }

//         if (round(targetGyroX - abs(GyroX)) == 0)
//         {
//             ;
//         }
//         else if (targetGyroX > abs(GyroX))
//         {
//             leftSpeedVal = changeSpeed(leftSpeedVal, +1); // would increase abs(GyroX)
//         }
//         else
//         {
//             leftSpeedVal = changeSpeed(leftSpeedVal, -1);
//         }
//         rightSpeedVal = leftSpeedVal;
//         // analogWrite(rightSpeed, rightSpeedVal);
//         // analogWrite(leftSpeed, leftSpeedVal);
//     }
// }

float changeSpeed(float motorSpeed, float increment)
{
    motorSpeed += increment;
    if (motorSpeed > maxSpeed)
    { // to prevent motorSpeed from exceeding 255, which is a problem when using analogWrite
        motorSpeed = maxSpeed;
    }
    else if (motorSpeed < minSpeed)
    {
        motorSpeed = minSpeed;
    }
    return motorSpeed;
}

void calculateError()
{
    // Ensure robot is stationary and calculates the default offset
    for (int c = 0; c < 200; c++)
    {
        AccErrorX += AccX;
        AccErrorY += AccY;
        AccErrorZ += AccZ;

        GyroErrorX += GyroX;
        GyroErrorY += GyroY;
        GyroErrorZ += GyroZ;
    }

    // Divide the sum by 200 to get the error value
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    AccErrorZ = AccErrorZ / 200;

    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
    Serial.println("The setting in MPU6050 has been calibrated");
}

void readIMU()
{
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    AccX = a.acceleration.x;
    AccY = a.acceleration.y;
    AccZ = a.acceleration.z;

    GyroX = g.gyro.x;
    GyroY = g.gyro.y;
    GyroZ = g.gyro.z;
}

// void forward()
// {                               // drives the car forward, assuming leftSpeedVal and rightSpeedVal are set high enough
//     digitalWrite(right1, HIGH); // the right motor rotates FORWARDS when right1 is HIGH and right2 is LOW
//     digitalWrite(right2, LOW);
//     digitalWrite(left1, HIGH);
//     digitalWrite(left2, LOW);
// }

// void left()
// { // rotates the car left, assuming speed leftSpeedVal and rightSpeedVal are set high enough
//     digitalWrite(right1, LOW);
//     digitalWrite(right2, HIGH);
//     digitalWrite(left1, HIGH);
//     digitalWrite(left2, LOW);
// }

// void right()
// {
//     digitalWrite(right1, HIGH);
//     digitalWrite(right2, LOW);
//     digitalWrite(left1, LOW);
//     digitalWrite(left2, HIGH);
// }

// void stopCar()
// {
//     digitalWrite(right1, LOW);
//     digitalWrite(right2, LOW);
//     digitalWrite(left1, LOW);
//     digitalWrite(left2, LOW);
//     analogWrite(rightSpeed, 0);
//     analogWrite(leftSpeed, 0);
// }