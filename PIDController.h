#pragma once
#include <Arduino.h>

class PIDController {
private:
    float KP = 0.2;
    float KD = 0.26;
    int lastError = 0;
    int center = 7500; // ค่ากลางของเซนเซอร์

public:
    void setParameters(float kp, float kd) {
        KP = kp;
        KD = kd;
    }

    void calculate(int position, int &motorL, int &motorR, int baseSpeed, int maxSpeed) {
        int error = position - center;
        int diff = (error * KP) + ((error - lastError) * KD);
        lastError = error;

        motorL = baseSpeed + diff;
        motorR = baseSpeed - diff;

        if (motorL > maxSpeed) motorL = maxSpeed;
        else if (motorL < -maxSpeed) motorL = -maxSpeed;

        if (motorR > maxSpeed) motorR = maxSpeed;
        else if (motorR < -maxSpeed) motorR = -maxSpeed;
    }
};
