#pragma once
#include <Arduino.h>
#include <Servo.h>

class ESCController {
private:
    Servo esc;
    int escPin;
    int currentSpeed = 0;
    unsigned long lastUpdateTime = 0;
    const int updateInterval = 20;

public:
    void begin(int pin) {
        escPin = pin;
        esc.attach(escPin);
        esc.writeMicroseconds(1000);
        delay(2000);
    }

    void setSpeed(int speed) {
        currentSpeed = constrain(speed, 0, 100);
        updateESC();
    }

    void softStop(int decelerationRate = 1, int interval = 50) {
        static unsigned long lastDeceleration = 0;
        unsigned long currentTime = millis();

        if (currentTime - lastDeceleration >= interval) {
            if (currentSpeed > 0) {
                currentSpeed = max(0, currentSpeed - decelerationRate);
                updateESC();
            }
            lastDeceleration = currentTime;
        }
        emergencyStop();
    }

    void emergencyStop() {
        currentSpeed = 0;
        esc.writeMicroseconds(1000);
    }

private:
    void updateESC() {
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= updateInterval) {
            int pulseWidth = map(currentSpeed, 0, 100, 1000, 2000);
            esc.writeMicroseconds(pulseWidth);
            lastUpdateTime = currentTime;
        }
    }
};
