#include "I2Cdev.h"
// #include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE

#include "Wire.h"

#endif

#define LED_PIN 27
bool blinkState = false;

byte fetch_pressure(int16_t dT_raw *p_P_dat, int16_t dT_raw *p_T_dat); // convert value to byte data type

#define TRUE 1
#define FALSE 0
#define PSI_to_Pa (6894.757f)

void setup()
{
    Wire.begin(32, 33);
    // Wire.setClock(400000);

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    // initialize device
    // Serial.println("Initializing I2C devices...");

    // verify connection
    // Serial.println("Testing device connections...");
    // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

void loop()
{

    byte _status;
    unsigned int P_dat;
    unsigned int T_dat;

    int16_t dp_raw;
    int16_t dT_raw;

    double P;
    double PR;
    double TR;
    double V;
    double VV;
    while (1)
    {
        _status = fetch_pressure(&dp_raw, &dT_raw);

        switch (_status)
        {
        case 0:
            Serial.println("Ok ");
            break;
        case 1:
            Serial.println("Busy");
            break;
        case 2:
            Serial.println("Slate");
            break;
        default:
            Serial.println("Error");
            break;
        }

        // PR = (double)((P_dat - 819.15) / (14744.7));
        // PR = (PR - 0.49060678);
        // PR = abs(PR);
        // P = (double)P_dat * .0009155;
        // V = ((PR * 13789.5144) / 1.225);
        // VV = (sqrt((V)));

        // TR = (double)((T_dat * 0.09770395701));
        // TR = TR - 50;

        Serial.print("raw Pressure:");
        Serial.println(dp_raw);

        float press_raw = PSI_to_Pa - (dp_raw - 0.1 * 16383) * PSI_to_Pa/(0.4f * 16383);

        Serial.print("pressure psi:");
        Serial.println(press_raw);
        // Serial.print("Draft:");
        // Serial.println(P * 2.3067);

        Serial.print(" ");
        Serial.print("raw Temp:");
        Serial.println(dT_raw);

        float temp = dT_raw * 200.0f / 2047 - 50;
        Serial.print("tempC:");
        Serial.println(temp);
        // Serial.print("tempF:");
        // Serial.println((TR * 1.8) + 32);
        Serial.println(" ");

        float currentSpeed = sqrt(press_raw * 2 / 1.225) * 3.6;

        delay(2000);
    }

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}

byte fetch_pressure(int16_t dT_raw *p_P_dat, int16_t dT_raw *p_T_dat)
{
    byte address, Press_H, Press_L, _status;
    int16_t dT_raw P_dat;
    int16_t dT_raw T_dat;

    address = 0x28;
    Wire.beginTransmission(address);
    Wire.endTransmission();
    delay(100);

    Wire.requestFrom((int)address, (int)4); // Request 4 bytes need 4 bytes are read
    Press_H = Wire.read();
    Press_L = Wire.read();
    byte Temp_H = Wire.read();
    byte Temp_L = Wire.read();
    Wire.endTransmission();

    _status = (Press_H >> 6) & 0x03;
    Press_H = Press_H & 0x3f;
    P_dat = (((unsigned int)Press_H) << 8) | Press_L;
    *p_P_dat = P_dat;

    Temp_L = (Temp_L >> 5);
    T_dat = (((unsigned int)Temp_H) << 3) | Temp_L;
    *p_T_dat = T_dat;
    return (_status);
}