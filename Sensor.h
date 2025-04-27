#pragma once

class Sensor {
    private:
      QTRSensors qtr;
      const uint8_t SensorCount = 8;
      uint16_t sensorValues[8];
      uint16_t Minvalue[8];
      uint16_t Maxvalue[8];
    
       
     
    public:
      Sensor(){
        qtr.setTypeRC();
        qtr.setSensorPins((const uint8_t[]){A0, A3, A4, A5, 12, 11, 10, 9}, SensorCount);
      }
       void setCalibrationMinMax(uint16_t minVal[], uint16_t maxVal[]) {
        for (uint8_t i = 0; i < SensorCount; i++) {
          Minvalue[i] = minVal[i];
          Maxvalue[i] = maxVal[i];
        }
       }
      void calibrate() {
        qtr.calibrate();
        for(uint16_t i = 0; i < SensorCount; i++) {
          qtr.calibrationOn.minimum[i] = Minvalue[i];
          qtr.calibrationOn.maximum[i] = Maxvalue[i];
        }
      }
    
      void ValueRaw() {
        
          qtr.read(sensorValues);
          for (uint8_t i = 0; i < SensorCount; i++)
          {
            Serial.print(sensorValues[i]);
            Serial.print('\t');
          }
          Serial.println();
          delay(100);
      }
       void ValueCallibrate() {
        
          qtr.readLineBlack(sensorValues);
          for (uint8_t i = 0; i < SensorCount; i++)
          {
            Serial.print(sensorValues[i]);
            Serial.print('\t');
          }
          Serial.println();
          delay(100);
      }
    
      int readLine() {
        return qtr.readLineBlack(sensorValues);
      }
    
      bool Switch(int pin, int threshold = 100) {
        return analogRead(pin) < threshold;
      }
      void Switch1(){
        while(1){
          if(Switch(A7)==true)break;
        }
      }
      void Switch2(){
        while(1){
          if(Switch(A6)==true)break;
        }
      }
    };