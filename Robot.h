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

    SensorMode mode = SENSOR_MODE_NONE; // โหมดที่ใช้อยู่

    Sensor* sensor = nullptr;
    QTRSensorsMUX* mux = nullptr;
    MotorController* motors;
    PIDController* pid;
    int baseSpeed;
    int position = 0;
public:
    // Constructor ใช้ Sensor ปกติ
    Robot(Sensor* s, MotorController* m, PIDController* p, int speed = 40)
      : sensor(s), motors(m), pid(p), baseSpeed(speed), mode(SENSOR_MODE_NORMAL) {}

    // Constructor ใช้ MUX
    Robot(QTRSensorsMUX* mx, MotorController* m, PIDController* p, int speed = 40)
      : mux(mx), motors(m), pid(p), baseSpeed(speed), mode(SENSOR_MODE_MUX) {}

    void setSpeed(int speed) {
        baseSpeed = speed;
    }

    void followLine() {
        int position = 0;
        uint16_t sensorValues[16]; // รองรับเซนเซอร์ได้ถึง 16 ตัว

        if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
            position = sensor->readLine();
        } 
        else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
            position = mux->readLineBlack(sensorValues);
        } 
        else {
            // ยังไม่ได้เลือก sensor หรือเลือกผิด
            Serial.println("Error: No sensor mode selected!");
            return;
        }

        int motorL, motorR;
        pid->calculate(position, motorL, motorR, baseSpeed, 90);
        motors->control(motorL, motorR);
    }
    void followLineTime(int baseSpeed,int Maxsp,float kp ,float kd,unsigned long time){
        uint16_t sensorValues[16]; // รองรับเซนเซอร์ได้ถึง 16 ตัว
        int motorL, motorR;
        pid->setParameters(kp,kd);
        unsigned long start = millis();
        while (millis() - start < time) {
            if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
                position = sensor->readLine();
            } 
            else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
                position = mux->readLineBlack(sensorValues);
            } 
        pid->calculate(position, motorL, motorR, baseSpeed,Maxsp);
         motors->control(motorL, motorR);
        // Serial.print("position = ");
        // Serial.print("\t");
        //  Serial.print(position);
        // Serial.print("\t");
    
        // Serial.print("motorL =");
        // Serial.print(motorL);
        // Serial.print("\t");
    
        // Serial.print("motorR =");
        // Serial.print(motorR);
        // Serial.println("\t");
    
    
        }
      }

      void followLineBridge(int baseSpeed,int Maxsp,float kp ,float kd)
      {
        uint16_t sensorValues[16]; // รองรับเซนเซอร์ได้ถึง 16 ตัว
        int motorL, motorR;
        pid->setParameters(kp,kd);
        mux->readLineBlack(sensorValues);
        while (sensorValues[0]<500||sensorValues[1]<500||sensorValues[2]<500||sensorValues[3]<500||sensorValues[4]<500||sensorValues[5]<500||sensorValues[6]<500||sensorValues[7]<500||sensorValues[8]<500||sensorValues[9]<500||sensorValues[10]<500||sensorValues[11]<500||sensorValues[12]<500||sensorValues[13]<500||sensorValues[14]<500||sensorValues[15]<500) {
            if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
                position = sensor->readLine();
            } 
            else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
                position = mux->readLineBlack(sensorValues);
            } 

        pid->calculate(position, motorL, motorR, baseSpeed,Maxsp);
        motors->control(motorL, motorR);
    
        }
        while (sensorValues[5]>500&&sensorValues[6]>500&&sensorValues[7]>500&&sensorValues[8]>500&&sensorValues[9]>500&&sensorValues[10]>500&&sensorValues[11]>500&&sensorValues[12]>500&&sensorValues[13]>500&&sensorValues[14]>500&&sensorValues[15]>500) {
            if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
                position = sensor->readLine();
            } 
            else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
                position = mux->readLineBlack(sensorValues);
            }

          motors->breakHard();

        }
        if (sensorValues[0]<500||sensorValues[1]<500||sensorValues[2]<500||sensorValues[3]<500||sensorValues[4]<500||sensorValues[5]<500||sensorValues[6]<500||sensorValues[7]<500||sensorValues[8]<500||sensorValues[9]<500||sensorValues[10]<500||sensorValues[11]<500||sensorValues[12]<500||sensorValues[13]<500||sensorValues[14]<500||sensorValues[15]<500) {
            unsigned long start = millis();
            while (millis() - start < 300) {
                if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
                    position = sensor->readLine();
                } 
                else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
                    position = mux->readLineBlack(sensorValues);
                } 
            pid->calculate(position, motorL, motorR, baseSpeed,Maxsp);
             motors->control(motorL, motorR);
    
        }
      }
    }
    void followOfstacle(int baseSpeed,int Maxsp,float kp ,float kd,unsigned long time,int sp1 ,int sp2){
        uint16_t sensorValues[16]; // รองรับเซนเซอร์ได้ถึง 16 ตัว
        int motorL, motorR;
        pid->setParameters(kp,kd);
        while(analogRead(A2)>400){
            if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
                position = sensor->readLine();
            } 
            else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
                position = mux->readLineBlack(sensorValues);
            }
        pid->calculate(position, motorL, motorR, baseSpeed,Maxsp);
         motors->control(motorL, motorR);


        }

    motors->control(5, 40);
    delay(120);
    motors->control(sp1, sp2);
    delay(time);
    motors->control(0, 0);
  
}
  void followOfstacleL(int baseSpeed,int Maxsp,float kp ,float kd,unsigned long time,int sp1 ,int sp2){
        uint16_t sensorValues[16]; // รองรับเซนเซอร์ได้ถึง 16 ตัว
        int motorL, motorR;
        pid->setParameters(kp,kd);
        while(analogRead(A2)>400){
            if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
                position = sensor->readLine();
            } 
            else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
                position = mux->readLineBlack(sensorValues);
            }
        pid->calculate(position, motorL, motorR, baseSpeed,Maxsp);
         motors->control(motorL, motorR);


        }

    motors->control(5, 40);
    delay(120);
    motors->control(sp1, sp2);
    delay(time);
    motors->control(0, 0);
  
}  
void followOfstacleR(int baseSpeed,int Maxsp,float kp ,float kd,unsigned long time,int sp1 ,int sp2){
    uint16_t sensorValues[16]; // รองรับเซนเซอร์ได้ถึง 16 ตัว
    int motorL, motorR;
    pid->setParameters(kp,kd);
    while(analogRead(A2)>400){
        if (mode == SENSOR_MODE_NORMAL && sensor != nullptr) {
            position = sensor->readLine();
        } 
        else if (mode == SENSOR_MODE_MUX && mux != nullptr) {
            position = mux->readLineBlack(sensorValues);
        }
    pid->calculate(position, motorL, motorR, baseSpeed,Maxsp);
     motors->control(motorL, motorR);


    }

motors->control(45, 5);
delay(120);
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
