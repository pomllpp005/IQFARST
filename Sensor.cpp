#include "Sensor.h"

Sensor::Sensor() {
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){A0, A3, A4, A5, 12, 11, 10, 9}, SensorCount);
}

void Sensor::setCalibrationMinMax(uint16_t minVal[], uint16_t maxVal[]) {
    for (uint8_t i = 0; i < SensorCount; i++) {
        Minvalue[i] = minVal[i];
        Maxvalue[i] = maxVal[i];
    }
}

void Sensor::calibrate() {
    qtr.calibrate();
    for (uint16_t i = 0; i < SensorCount; i++) {
        qtr.calibrationOn.minimum[i] = Minvalue[i];
        qtr.calibrationOn.maximum[i] = Maxvalue[i];
    }
}

void Sensor::ValueRaw() {
    qtr.read(sensorValues);
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();
    delay(100);
}

void Sensor::ValueCallibrate() {
    qtr.readLineBlack(sensorValues);
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();
    delay(100);
}

int Sensor::readLine() {
    return qtr.readLineBlack(sensorValues);
}

bool Sensor::Switch(int pin, int threshold) {
    return analogRead(pin) < threshold;
}

void Sensor::Switch1() {
    while (1) {
        if (Switch(A7)) break;
    }
}

void Sensor::Switch2() {
    while (1) {
        if (Switch(A6)) break;
    }
}
