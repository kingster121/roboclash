#include <NewPing.h>
#include <ctime>

// Max distance for ultrasonic
const int MAX_DISTANCE = 50;
const int WALL_DISTANCE = 40;

// Ultrasonic pin config
const int TRIGGER_PIN_1 = 2;
const int ECHO_PIN_1 = 15;

const int TRIGGER_PIN_2 = 4;
const int ECHO_PIN_2 = 16;

const int TRIGGER_PIN_3 = 17;
const int ECHO_PIN_3 = 5;

const int TRIGGER_PIN_4 = 18;
const int ECHO_PIN_4 = 19;

// IMU - I2C address 0x68
// Nothing for now

// Compass
const int COMPASS_SCL = 22;
const int COMPASS_SDA = 21;

// Initialise Newping Object (Ultrasonics)
NewPing sonar_right(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);
NewPing sonar_front(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar_left(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);
NewPing sonar_back(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE);

//--------------- Helper functions -----------------//
int *get_ultrasonic_dist(); // Returns array[4] = {right, front, left, back}
void show_distance(int *ultrasonic_dist_arr);

//-------------- Essential functions ---------------//
void forward(); // Returns array[2] = {l_motor_duty_cycle, r_motor_duty_cycle}
void forward(int time);
void turn_left();
void turn_right();

//--------------- ESP32 Logic ----------------------//
void setup()
{
    Serial.begin(115200);

    // Get the true-north

    // turn right until (right == true)
}

time_t start_time = time(nullptr);
time_t current_time = start_time;
void loop()
{
    double time_passed = difftime(current_time, start_time);

    int *ultrasonic_dist_arr = get_ultrasonic_dist();
    show_distance(ultrasonic_dist_arr);

    // while (time_passed < 180)
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

    current_time = time(nullptr);

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
int *get_ultrasonic_dist()
{
    static int distanceArray[4];
    distanceArray[0] = sonar_right.ping_cm();
    distanceArray[1] = sonar_front.ping_cm();
    distanceArray[2] = sonar_left.ping_cm();
    distanceArray[3] = sonar_back.ping_cm();

    // distanceArray[1] = 0;
    // distanceArray[2] = 0;
    // distanceArray[3] = 0;

    return distanceArray; // Returns pointer to the array
}

void show_distance(int *ultrasonic_dist_arr)
{
    for (int i = 0; i < 4; i++)
    {
        int dist = ultrasonic_dist_arr[i];
        if (i == 0)
            Serial.print("Right: ");
        else if (i == 1)
            Serial.print("\tFront: ");
        else if (i == 2)
            Serial.print("\tLeft: ");
        else if (i == 3)
            Serial.print("\tBack: ");

        Serial.print(dist);
        Serial.print("cm");
    }
    Serial.println();
}
// --------- End of Helper Functions ---------------//

//-------------- Essential functions ---------------//

void forward()
{
    while (true)
    {
        int dist = sonar_front.ping_cm();
        if (dist > 0 && dist <= WALL_DISTANCE)
            break;
    }
}

void forward(int time)
{
}

void turn_left() {}
void turn_right() {}
// --------End of Essential functions --------------//