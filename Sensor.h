#pragma once

#include <Arduino.h>
#include <QTRSensors.h>

class Sensor {
private:
    QTRSensors qtr;
    const uint8_t SensorCount = 8;
    uint16_t sensorValues[8];
    uint16_t Minvalue[8];
    uint16_t Maxvalue[8];

public:
    Sensor();
    void setCalibrationMinMax(uint16_t minVal[], uint16_t maxVal[]);
    void calibrate();
    void ValueRaw();
    void ValueCallibrate();
    int  readLine();
    bool Switch(int pin, int threshold = 100);
    void Switch1();
    void Switch2();
};
