#pragma once
#include <Arduino.h>

class MotorController {
private:
    // Motor R (Right)
    int R_AIN1, R_AIN2, R_PWMA;
    // Motor L (Left)
    int L_BIN1, L_BIN2, L_PWMB;

public:
    // Constructor to initialize pins for both motors
    MotorController(int r_ain1, int r_ain2, int r_pwma, int l_bin1, int l_bin2, int l_pwmb)
        : R_AIN1(r_ain1), R_AIN2(r_ain2), R_PWMA(r_pwma),
          L_BIN1(l_bin1), L_BIN2(l_bin2), L_PWMB(l_pwmb) {}

    void begin() {
        // Set timer configurations (if necessary)
        TCCR0B = (TCCR0B & 0b11111000) | 0x03;
        TCCR2B = (TCCR2B & 0b11111000) | 0x03;
        OCR0A = 16;
        OCR2A = 16;

        // Configure pins for Motor R
        pinMode(R_AIN1, OUTPUT);
        pinMode(R_AIN2, OUTPUT);
        pinMode(R_PWMA, OUTPUT);

        // Configure pins for Motor L
        pinMode(L_BIN1, OUTPUT);
        pinMode(L_BIN2, OUTPUT);
        pinMode(L_PWMB, OUTPUT);
    }

    void control(int speedR, int speedL) {
        // Control Motor R
        if (speedR >= 0) {
            digitalWrite(R_AIN1, LOW);
            digitalWrite(R_AIN2, HIGH);
        } else {
            digitalWrite(R_AIN1, HIGH);
            digitalWrite(R_AIN2, LOW);
            speedR = -speedR; // Make speed positive
        }
        analogWrite(R_PWMA, speedR * 2.55); // Scale 0-100 to 0-255

        // Control Motor L
        if (speedL >= 0) {
            digitalWrite(L_BIN1, LOW);
            digitalWrite(L_BIN2, HIGH);
        } else {
            digitalWrite(L_BIN1, HIGH);
            digitalWrite(L_BIN2, LOW);
            speedL = -speedL; // Make speed positive
        }
        analogWrite(L_PWMB, speedL * 2.55); // Scale 0-100 to 0-255
    }
};