
#include <Arduino.h>
#include <Stream.h>

#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X80_NONAME_F_HW_I2C u8g2(U8G2_R1, /* reset=*/U8X8_PIN_NONE, /* clock=*/22, /* data=*/21);

#define MSP_PAYLOAD_SIZE 128
#define MSP2_SENSOR_BAROMETER 0x1F05
#define MSP2_SENSOR_AIRSPEED 0x1F06

// struct mspSensorAirspeedDataMessage_t
// {
//     uint8_t instance;
//     uint32_t timeMs;
//     float diffPressurePa;
//     int16_t temp;
// };

struct mspSensorBaroDataMessage_t
{
    uint8_t instance;
    uint32_t timeMs;
    float pressurePa;
    int16_t temp; // centi-degrees C
};

struct msp_packet_t
{
    // recvSize can be NULL
    uint8_t recvMessageID;
    uint8_t recvSizeValue;
    uint8_t payload[MSP_PAYLOAD_SIZE];
    uint8_t recvSize;
    mspSensorBaroDataMessage_t mspMessage;
};

msp_packet_t packet;

Stream *_stream;
// uint8_t out_buf[MSP_PORT_OUTBUF_SIZE];

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii)
    {
        if (crc & 0x80)
        {
            crc = (crc << 1) ^ 0xD5;
        }
        else
        {
            crc = crc << 1;
        }
    }
    return crc;
}

// https://github.com/iNavFlight/inav/wiki/MSP-V2
void send(uint16_t messageID, void *payload, uint16_t size)
{
    uint8_t flag = 0;
    uint8_t crc = 0;
    uint8_t tmp_buf[2];

    // $X<
    _stream->write('$');
    _stream->write('X');
    _stream->write('<');

    crc = crc8_dvb_s2(crc, flag);
    _stream->write(flag);

    memcpy(tmp_buf, &messageID, 2);
    crc = crc8_dvb_s2(crc, tmp_buf[0]);
    crc = crc8_dvb_s2(crc, tmp_buf[1]);
    _stream->write(tmp_buf, 2);

    memcpy(tmp_buf, &size, 2);
    crc = crc8_dvb_s2(crc, tmp_buf[0]);
    crc = crc8_dvb_s2(crc, tmp_buf[1]);
    _stream->write(tmp_buf, 2);

    uint8_t *payloadPtr = (uint8_t *)payload;
    for (uint8_t i = 0; i < size; ++i)
    {
        uint8_t b = *(payloadPtr++);
        crc = crc8_dvb_s2(crc, b);
        _stream->write(b);
    }
    _stream->write(crc);
}
// mspSensorAirspeedDataMessage_t speedSensor = { 0, millis(), difference, (int16_t) middleTemperature*100};
// send(MSP2_SENSOR_AIRSPEED, &speedSensor, sizeof(speedSensor));

// timeout in milliseconds
uint32_t _timeout = 500;
bool recv(uint8_t *messageID, void *payload, uint8_t maxSize, uint8_t *recvSize)
{
    char header[3];
    uint8_t crc = 0;

    uint8_t flag = 0;
    uint32_t t0 = millis();

    while (1)
    {

        // read header
        while (_stream->available() < 6)
        {
            if (millis() - t0 >= _timeout)
            {
                return false;
            }
        }
        memset(header, 0, sizeof(header));
        _stream->readBytes((char *)header, 3);

        // check header
        if (header[0] == '$' && header[1] == 'X' && header[2] == '<')
        {
            // read flag
            flag = _stream->read();
            crc = crc8_dvb_s2(crc, flag);

            // read message ID (type)
            *messageID = _stream->read();
            crc = crc8_dvb_s2(crc, messageID[0]);
            crc = crc8_dvb_s2(crc, messageID[1]);

            // header ok, read payload size
            *recvSize = _stream->read();
            crc = crc8_dvb_s2(crc, recvSize[0]);
            crc = crc8_dvb_s2(crc, recvSize[1]);

            // read payload
            uint8_t *payloadPtr = (uint8_t *)payload;
            uint8_t idx = 0;
            while (idx < *recvSize)
            {
                if (millis() - t0 >= _timeout)
                {
                    return false;
                }

                if (_stream->available() > 0)
                {
                    uint8_t b = _stream->read();
                    crc = crc8_dvb_s2(crc, b);
                    if (idx < maxSize)
                    {
                        *(payloadPtr++) = b;
                    }
                    ++idx;
                }
            }
            // zero remaining bytes if *size < maxSize
            for (; idx < maxSize; ++idx)
            {
                *(payloadPtr++) = 0;
            }

            // read and check checksum
            while (_stream->available() == 0)
            {
                if (millis() - t0 >= _timeout)
                {
                    return false;
                }
            }
            uint8_t checksum = _stream->read();
            if (crc == checksum)
            {
                return true;
            }
        }
    }
}

// wait for messageID
// recvSize can be NULL
bool waitFor(uint8_t messageID, void *payload, uint8_t maxSize, uint8_t *recvSize)
{
    uint8_t recvMessageID;
    uint8_t recvSizeValue;
    uint32_t t0 = millis();
    while (millis() - t0 < _timeout)
        if (recv(&recvMessageID, payload, maxSize, (recvSize ? recvSize : &recvSizeValue)) && messageID == recvMessageID)
            return true;

    // timeout
    return false;
}

void setup(void)
{
    Wire.begin();
    Serial.begin(115200);
    _stream = &Serial;

    u8g2.begin();
    u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
}

void loop(void)
{
    u8g2.setFont(u8g2_font_unifont_t_chinese2); // use chinese2 for all the glyphs of "你好世界"
    u8g2.setFontDirection(0);
    u8g2.clearBuffer();



    // mspSensorAirspeedDataMessage_t speedSensor = { 0, millis(), difference, (int16_t) middleTemperature*100};
    // mspSensorAirspeedDataMessage_t baroSensor = { 0, millis(), middlePressure, (int16_t) middleTemperature*100};

    if (recv(&packet.recvMessageID, packet.payload, sizeof(packet.payload), &packet.recvSize))
    {
        memset(&packet.mspMessage, 0, sizeof(packet.mspMessage));
    
        if (sizeof(packet.mspMessage) >= packet.recvSize) {
            memcpy(&packet.mspMessage, packet.payload, sizeof(packet.mspMessage));
        }


        switch (packet.recvMessageID)
        {
        case MSP2_SENSOR_BAROMETER:
            u8g2.setCursor(0, 15);
            u8g2.print("气压");
            u8g2.print(packet.mspMessage.pressurePa, 2);
            u8g2.print(" (");
            u8g2.print(packet.mspMessage.temp);
            u8g2.print("℃)");
            break;
        case MSP2_SENSOR_AIRSPEED:
            u8g2.setCursor(0, 40);
            u8g2.print("压差");
            u8g2.print(packet.mspMessage.pressurePa, 2);
            u8g2.print(" (");
            u8g2.print(packet.mspMessage.temp);
            u8g2.print("℃)");
            break;
        default:
            break;
        }
    }

    u8g2.sendBuffer();

    delay(1000);
}
