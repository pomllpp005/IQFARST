#pragma once
#include <Arduino.h>
#include "Sensor.h"
#include "MotorController.h"
#include "PIDController.h"
#include "QTRSensorsMUX.h"

class Robot {
private:
    enum SensorMode {
        SENSOR_MODE_NONE,
        SENSOR_MODE_NORMAL,
        SENSOR_MODE_MUX
    };

    SensorMode mode = SENSOR_MODE_NONE;
    Sensor* sensor = nullptr;
    QTRSensorsMUX* mux = nullptr;
    MotorController* motors;
    PIDController* pid;
    int baseSpeed;
    int position = 0;
    bool Flip;

    bool allSensorsBelow(uint16_t* vals, uint16_t threshold) {
        for (int i = 0; i < 16; i++) {
            if (vals[i] >= threshold) return false;
        }
        return true;
    }

    bool allSensorsAbove(uint16_t* vals, uint16_t threshold, int start = 5, int end = 15) {
        for (int i = start; i <= end; i++) {
            if (vals[i] <= threshold) return false;
        }
        return true;
    }

public:
    // Constructor for normal sensor
    Robot(Sensor* s, MotorController* m, PIDController* p, int speed = 40, bool flip = false)
        : sensor(s), motors(m), pid(p), baseSpeed(speed), mode(SENSOR_MODE_NORMAL), Flip(flip) {}

    // Constructor for MUX sensor
    Robot(QTRSensorsMUX* mx, MotorController* m, PIDController* p, int speed = 40, bool flip = false)
        : mux(mx), motors(m), pid(p), baseSpeed(speed), mode(SENSOR_MODE_MUX), Flip(flip) {}

    void setSpeed(int speed) {
        baseSpeed = speed;
    }

    void followLine() {
        uint16_t sensorValues[16];

        if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
            position = sensor->readLine();
        } 
        else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
            position = mux->readLineBlack(sensorValues, Flip);
        } 
        else {
            Serial.println("Error: No sensor mode selected!");
            return;
        }

        int motorL, motorR;
        pid->calculate(position, motorL, motorR, baseSpeed, 90);
        motors->control(motorL, motorR);
    }

    void followLineTime(int baseSpeed, int maxSpeed, float kp, float kd, unsigned long time) {
        uint16_t sensorValues[16];
        int motorL, motorR;
        pid->setParameters(kp, kd);
        unsigned long start = millis();

        while (millis() - start < time) {
            if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
                position = sensor->readLine();
            } 
            else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
                position = mux->readLineBlack(sensorValues, Flip);
            }

            pid->calculate(position, motorL, motorR, baseSpeed, maxSpeed);
            motors->control(motorL, motorR);
        }
    }

    void followLineBridge(int baseSpeed, int maxSpeed, float kp, float kd) {
        uint16_t sensorValues[16];
        int motorL, motorR;
        pid->setParameters(kp, kd);

        mux->readLineBlack(sensorValues, Flip);

        // Approach the bridge
        while (!allSensorsBelow(sensorValues, 500)) {
            position = mux->readLineBlack(sensorValues, Flip);
            pid->calculate(position, motorL, motorR, baseSpeed, maxSpeed);
            motors->control(motorL, motorR);
        }

        // Fully on bridge
        while (allSensorsAbove(sensorValues, 500)) {
            position = mux->readLineBlack(sensorValues, Flip);
            motors->breakHard();  // Consider soft braking if smoother transition is needed
        }

        // Continue forward briefly after crossing
        if (!allSensorsBelow(sensorValues, 500)) {
            unsigned long start = millis();
            while (millis() - start < 300) {
                position = mux->readLineBlack(sensorValues, Flip);
                pid->calculate(position, motorL, motorR, baseSpeed, maxSpeed);
                motors->control(motorL, motorR);
            }
        }
    }

    void followObstacleDir(int baseSpeed, int maxSpeed, float kp, float kd, unsigned long time, int sp1, int sp2, bool turnLeft = true) {
        uint16_t sensorValues[16];
        int motorL, motorR;
        pid->setParameters(kp, kd);

        while (analogRead(A2) > 400) {
            if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
                position = sensor->readLine();
            } 
            else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
                position = mux->readLineBlack(sensorValues, Flip);
            }

            pid->calculate(position, motorL, motorR, baseSpeed, maxSpeed);
            motors->control(motorL, motorR);
        }

        // Turn to avoid obstacle
        motors->control(turnLeft ? 5 : 45, turnLeft ? 40 : 5);
        delay(120);

        // Move with custom speeds
        motors->control(sp1, sp2);
        delay(time);
        motors->control(0, 0);
    }

    void moveForward(unsigned long duration) {
        unsigned long start = millis();
        while (millis() - start < duration) {
            motors->control(baseSpeed, baseSpeed);
        }
    }

    void moveBackward(unsigned long duration) {
        unsigned long start = millis();
        while (millis() - start < duration) {
            motors->control(-baseSpeed, -baseSpeed);
        }
    }

    void stop() {
        motors->control(0, 0);
    }

    void softStop() {
        for (int i = baseSpeed; i >= 0; i -= 2) {
            motors->control(i, i);
            delay(20);
        }
        stop();
    }
};
