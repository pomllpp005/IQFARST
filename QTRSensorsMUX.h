#pragma once

#include <Arduino.h>

class QTRSensorsMUX {
public:
    QTRSensorsMUX(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t sigPin, uint8_t sensorCount)
      : _s0(s0), _s1(s1), _s2(s2), _s3(s3), _sigPin(sigPin), _sensorCount(sensorCount) {
        for (uint8_t i = 0; i < 16; i++) {
            _min[i] = 1023;
            _max[i] = 0;
        }
        _lastPosition = ((_sensorCount - 1) * 1000) / 2; // กำหนดตรงนี้แทน!
    }

    void begin() {
        pinMode(_s0, OUTPUT);
        pinMode(_s1, OUTPUT);
        pinMode(_s2, OUTPUT);
        pinMode(_s3, OUTPUT);
    }

    void read(uint16_t *sensorValues) {
        for (uint8_t i = 0; i < _sensorCount; i++) {
            selectMuxChannel(i);
            delayMicroseconds(2);
            sensorValues[i] = analogRead(_sigPin);
        }
    }

    void setCalibrationData(const uint16_t *minArray, const uint16_t *maxArray, uint8_t count) {
        uint8_t limit = (count < _sensorCount) ? count : _sensorCount;
        for (uint8_t i = 0; i < limit; i++) {
            _min[i] = minArray[i];
            _max[i] = maxArray[i];
        }
    }

    void resetCalibration() {
        for (uint8_t i = 0; i < _sensorCount; i++) {
            _min[i] = 1023;
            _max[i] = 0;
        }
    }

    void calibrate() {
        uint16_t sensorValues[_sensorCount];
        uint16_t maxSensorValues[_sensorCount];
        uint16_t minSensorValues[_sensorCount];

        for (uint8_t i = 0; i < _sensorCount; i++) {
            maxSensorValues[i] = 0;
            minSensorValues[i] = 1023;
        }

        for (uint8_t j = 0; j < 10; j++) { // เก็บค่าหลายครั้ง
            read(sensorValues);

            for (uint8_t i = 0; i < _sensorCount; i++) {
                if (sensorValues[i] > maxSensorValues[i]) {
                    maxSensorValues[i] = sensorValues[i];
                }
                if (sensorValues[i] < minSensorValues[i]) {
                    minSensorValues[i] = sensorValues[i];
                }
            }
        }

        for (uint8_t i = 0; i < _sensorCount; i++) {
            _min[i] = minSensorValues[i];
            _max[i] = maxSensorValues[i];
        }
    }

    void readCalibrated(uint16_t *sensorValues) {
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
    
            // สลับลำดับค่าที่คำนวณแล้ว
            sensorValues[i] = value;
        }
    
        // Flip the order of the calibrated values
        for (uint8_t i = 0; i < _sensorCount / 2; i++) {
            uint16_t temp = sensorValues[i];
            sensorValues[i] = sensorValues[_sensorCount - 1 - i];
            sensorValues[_sensorCount - 1 - i] = temp;
        }
    }

    uint16_t readLineBlack(uint16_t *sensorValues) {
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

private:
    uint8_t _s0, _s1, _s2, _s3, _sigPin;
    uint8_t _sensorCount;
    uint16_t _min[16];
    uint16_t _max[16];
    uint16_t _lastPosition;

    void selectMuxChannel(uint8_t channel) {
        digitalWrite(_s0, channel & 0x01);
        digitalWrite(_s1, (channel >> 1) & 0x01);
        digitalWrite(_s2, (channel >> 2) & 0x01);
        digitalWrite(_s3, (channel >> 3) & 0x01);
    }
};
