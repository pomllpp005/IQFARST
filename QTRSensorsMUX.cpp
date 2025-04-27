#include "QTRSensorsMUX.h"

QTRSensorsMUX::QTRSensorsMUX(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t sigPin, uint8_t sensorCount)
  : _s0(s0), _s1(s1), _s2(s2), _s3(s3), _sigPin(sigPin), _sensorCount(sensorCount), _lastPosition(3500) {
    for (uint8_t i = 0; i < 16; i++) {
        _min[i] = 1023;
        _max[i] = 0;
    }
}

void QTRSensorsMUX::begin() {
    pinMode(_s0, OUTPUT);
    pinMode(_s1, OUTPUT);
    pinMode(_s2, OUTPUT);
    pinMode(_s3, OUTPUT);
}

void QTRSensorsMUX::read(uint16_t* sensorValues) {
    for (uint8_t i = 0; i < _sensorCount; i++) {
        selectMuxChannel(i);
        delayMicroseconds(5);
        sensorValues[i] = analogRead(_sigPin);
    }
}

void QTRSensorsMUX::calibrate(uint16_t* sensorValues) {
    for (uint8_t i = 0; i < _sensorCount; i++) {
        if (sensorValues[i] < _min[i]) _min[i] = sensorValues[i];
        if (sensorValues[i] > _max[i]) _max[i] = sensorValues[i];
    }
}

void QTRSensorsMUX::resetCalibration() {
    for (uint8_t i = 0; i < _sensorCount; i++) {
        _min[i] = 1023;
        _max[i] = 0;
    }
}

void QTRSensorsMUX::readCalibrated(uint16_t* sensorValues) {
    uint16_t rawValues[_sensorCount];
    read(rawValues);

    for (uint8_t i = 0; i < _sensorCount; i++) {
        uint16_t denominator = _max[i] - _min[i];
        int16_t value = 0;

        if (denominator != 0) {
            value = (((int32_t)rawValues[i]) - _min[i]) * 1000 / denominator;
        }

        if (value < 0) value = 0;
        else if (value > 1000) value = 1000;

        sensorValues[i] = value;
    }
}

uint16_t QTRSensorsMUX::readLineBlack(uint16_t* sensorValues) {
    readCalibrated(sensorValues);

    bool onLine = false;
    uint32_t avg = 0;
    uint16_t sum = 0;

    for (uint8_t i = 0; i < _sensorCount; i++) {
        uint16_t value = sensorValues[i];

        if (value > 50) {
            onLine = true;
            avg += (uint32_t)value * (i * 1000);
            sum += value;
        }
    }

    if (!onLine) {
        return _lastPosition;
    }

    _lastPosition = avg / sum;
    return _lastPosition;
}

void QTRSensorsMUX::selectMuxChannel(uint8_t channel) {
    digitalWrite(_s0, channel & 0x01);
    digitalWrite(_s1, (channel >> 1) & 0x01);
    digitalWrite(_s2, (channel >> 2) & 0x01);
    digitalWrite(_s3, (channel >> 3) & 0x01);
}
