#include <Arduino.h>
#include <NewPing.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

// Compass
QMC5883LCompass compass;
int north;
int get_angle(int base_north)
{
    compass.read();
    int current_angle = compass.getAzimuth();
    int azimuth = current_angle - base_north;
    if (azimuth < -180)
        azimuth = 360 + azimuth;
    if (azimuth > 180)
        azimuth = azimuth - 360;

    // Serial.print("North:" + String(north) + "\t");
    // Serial.print("Current Angle: " + String(current_angle) + "\t");
    // Serial.println("Azimuth: " + String(azimuth) + "\t");
    return azimuth;
}

// (RX) ESP MAC address
uint8_t broadcast_address1[] = {0xB0, 0xA7, 0x32, 0x2B, 0x6E, 0x24};
uint8_t broadcast_address2[] = {0x24, 0x62, 0xAB, 0xE0, 0xEF, 0xF0};

// ESP NOW message struct
typedef struct struct_message
{
    int ultrasonic1;
    int ultrasonic2;
    int ultrasonic3;
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
}

// Max distance for ultrasonic
const int MAX_DISTANCE = 50;
const int WALL_DISTANCE = 40;

// Ultrasonic pin config
const int TRIGGER_PIN_1 = 23;
const int ECHO_PIN_1 = 19;

const int TRIGGER_PIN_2 = 5;
const int ECHO_PIN_2 = 18;

const int TRIGGER_PIN_3 = 33;
const int ECHO_PIN_3 = 35;

// const int TRIGGER_PIN_4 = 23;
// const int ECHO_PIN_4 = 19;

// Initialise Newping Object (Ultrasonics)
NewPing sonar_right(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar_front(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar_left(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);
// NewPing sonar_back(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE);

// IMU - I2C address 0x68
// Nothing for now

//--------------- Helper functions -----------------//
// int *get_ultrasonic_dist(); // Returns array[4] = {right, front, left, back}
// void show_distance(int *ultrasonic_dist_arr);
// float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

// //-------------- Essential functions ---------------//
// void forward(); // Returns array[2] = {l_motor_duty_cycle, r_motor_duty_cycle}
// void forward(unsigned long time);
// void turn_left();
// void turn_right();
// void stop();
// void go_home();

// Programme start timer
unsigned long start_time = millis();

//--------------- ESP32 Logic ----------------------//
void setup()
{
    Serial.begin(115200);

    // ESP_NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initialising ESP_NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    // Register peer
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    // Register first peer
    memcpy(peerInfo.peer_addr, broadcast_address1, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }
    // Register second peer
    memcpy(peerInfo.peer_addr, broadcast_address2, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }

    // Compass initialisation and calibration
    compass.init();
    compass.setCalibration(-1227, 920, -1190, 1457, -5683, -3171);

    // Get the true-north
    Serial.println("Get what degree is the true-north");
    compass.read();
    north = compass.getAzimuth();

    // // Turn right until (right == true)
    // Serial.println("Turn right until compass is East");
    // turn_right();
}

void loop()
{
    // int *ultrasonic_dist_arr = get_ultrasonic_dist();
    data.ultrasonic1 = sonar_right.ping_cm();
    data.ultrasonic2 = sonar_front.ping_cm();
    data.ultrasonic3 = sonar_left.ping_cm();
    data.angle = get_angle(north);
    esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));

    // Serial.println("Brain looping");
    // int *ultrasonic_dist_arr = get_ultrasonic_dist();
    // show_distance(ultrasonic_dist_arr);

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

    delay(1000);
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
