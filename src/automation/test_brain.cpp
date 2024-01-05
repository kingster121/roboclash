#include <Arduino.h>
#include <NewPing.h>
#include <esp_now.h>
#include <WiFi.h>

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    char macStr[18];
    Serial.print("Packet to: ");
    // Copies the sender mac address to a string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
    Serial.print(" send status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// (RX) ESP MAC address
uint8_t broadcast_address1[] = {0x24, 0x0A, 0xC4, 0x60, 0xCA, 0x8C};

// ESP NOW message struct
typedef struct struct_message
{
    int pin1;
    int pin2;
    int pin3;
    int pin4;
    int on_off;
} struct_message;

// Create a struct_message called myData
struct_message data;

esp_now_peer_info_t peerInfo;

// Max distance for ultrasonic
const int MAX_DISTANCE = 50;
const int WALL_DISTANCE = 40;

// Ultrasonics
const int TRIGGER_PIN_1 = 5;
const int ECHO_PIN_1 = 18;

const int TRIGGER_PIN_2 = 23;
const int ECHO_PIN_2 = 19;

const int TRIGGER_PIN_3 = 32;
const int ECHO_PIN_3 = 34;

const int TRIGGER_PIN_4 = 33;
const int ECHO_PIN_4 = 35;

// Initialise Newping Object (Ultrasonics)
NewPing sonar_right(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar_front(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar_left(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);
NewPing sonar_back(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE);

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

    // Get the true-north
    Serial.println("Get what degree is the true-north");

    // Turn right until (right == true)
    Serial.println("Turn right until compass is East");

    data.pin1 = 17;
    data.pin2 = 16;
    data.pin3 = 18;
    data.pin4 = 5;
    data.on_off = 1;
}

void loop()
{
    esp_now_send(0, (uint8_t *)&data, sizeof(struct_message));
    delay(1000);
}