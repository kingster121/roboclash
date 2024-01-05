// ----------------------------------------------------------------------------------------------//
// Robert's Smorgasbord 2022                                                                     //
// https://robertssmorgasbord.net                                                                //
// https://www.youtube.com/channel/UCGtReyiNPrY4RhyjClLifBA                                      //
// QST QMC5883L 3-Axis Digital Compass and Arduino MCU – The Basics https://youtu.be/xh_KCkds038 //
// ----------------------------------------------------------------------------------------------//

#include <QMC5883LCompass.h>

// Mode Control (MODE)
const byte qmc5883l_mode_stby = 0x00;
const byte qmc5883l_mode_cont = 0x01;
// Output Data Rate (ODR)
const byte qmc5883l_odr_10hz = 0x00;
const byte qmc5883l_odr_50hz = 0x04;
const byte qmc5883l_odr_100hz = 0x08;
const byte qmc5883l_odr_200hz = 0x0C;
// Full Scale Range (RNG)
const byte qmc5883l_rng_2g = 0x00;
const byte qmc5883l_rng_8g = 0x10;
// Over Sample Ratio (OSR)
const byte qmc5883l_osr_512 = 0x00;
const byte qmc5883l_osr_256 = 0x40;
const byte qmc5883l_osr_128 = 0x80;
const byte qmc5883l_osr_64 = 0xC0;

QMC5883LCompass compass;
int north;
int get_angle()
{
    int current_angle = compass.getAzimuth();
    int azimuth = current_angle - north;
    if (azimuth < -180)
    {
        Serial.println("First");
        azimuth = 360 + azimuth;
    }

    if (azimuth > 180)
    {
        Serial.println("Second");
        azimuth = azimuth - 360;
    }

    Serial.print("North:" + String(north) + "\t");
    Serial.print("Current Angle: " + String(current_angle) + "\t");
    Serial.println("Azimuth: " + String(azimuth) + "\t");
    return azimuth;
}

void setup()
{
    Serial.begin(115200);
    compass.init();
    // compass.setADDR(byte b);
    // compass.setMode(byte mode,          byte odr,           byte rng,        byte osr        );
    // compass.setMode(qmc5883l_mode_cont, qmc5883l_odr_200hz, qmc5883l_rng_8g, qmc5883l_osr_512);
    // compass.setSmoothing(byte steps, bool adv);
    // compass.setCalibration(int x_min, int x_max, int y_min, int y_max, int z_min, int z_max);
    compass.setCalibration(-1227, 920, -1190, 1457, -5683, -3171);
    compass.read();
    north = compass.getAzimuth();
    // compass.setReset();
}

void loop()
{
    int x_value;
    int y_value;
    int z_value;
    int azimuth;  // 0° - 359°
    byte bearing; // 0 - 15 (N, NNE, NE, ENE, E, ...)
    char direction[strlen("NNE") + 1];
    char buffer[strlen("X=-99999 | Y=-99999 | Z=-99999 | A=259° | B=15 | D=NNE") + 1];

    compass.read(); // Read compass values via I2C

    x_value = compass.getX();
    y_value = compass.getY();
    z_value = compass.getZ();
    azimuth = get_angle(); // Calculated from X and Y value
    bearing = compass.getBearing(azimuth);

    compass.getDirection(direction, azimuth);

    // direction[3] = '\0';

    // sprintf(buffer,
    //         "X=%6d | Y=%6d | Z=%6d | A=%3d° | B=%02hu | %s",
    //         x_value,
    //         y_value,
    //         z_value,
    //         azimuth,
    //         bearing,
    //         direction);
    // Serial.println(buffer);

    delay(200);
}