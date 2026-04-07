#ifndef I2C_H
#define I2C_H

#include "config.h"
#include <Wire.h>
#include <VL53L0X.h>

#define MPU6500_ADDR 0x68 

class I2C;
extern I2C i2c;

class I2C{
  private:
    VL53L0X sensorL, sensorR, sensorFL, sensorFR;
    bool ActiveSensor = false;
    volatile int senL, senR, senFL, senFR;

    float gyroZ_offset = 0;
    volatile float angleZ = 0;        
    volatile float current_omega = 0; 
    unsigned long last_mpu_time = 0;  
    
    volatile float angleZ_change = 0; 

  public:
    void enable(){ ActiveSensor = true; }
    void disable(){ ActiveSensor = false; }

    void begin(){
      disable();
      pinMode(XSHUTL, OUTPUT); digitalWrite(XSHUTL, LOW);
      pinMode(XSHUTR, OUTPUT); digitalWrite(XSHUTR, LOW);
      pinMode(XSHUTFL, OUTPUT); digitalWrite(XSHUTFL, LOW);
      pinMode(XSHUTFR, OUTPUT); digitalWrite(XSHUTFR, LOW);

      Wire.begin(); 
      Wire.setClock(400000); 
      delay(50);
     
      digitalWrite(XSHUTL, HIGH); delay(50); sensorL.init(); sensorL.setAddress(0x30);
      digitalWrite(XSHUTR, HIGH); delay(50); sensorR.init(); sensorR.setAddress(0x31);
      digitalWrite(XSHUTFL, HIGH); delay(50); sensorFL.init(); sensorFL.setAddress(0x32);
      digitalWrite(XSHUTFR, HIGH); delay(50); sensorFR.setTimeout(500); sensorFR.init(); sensorFR.setAddress(0x33);
     
      sensorL.startContinuous(); sensorR.startContinuous();
      sensorFL.startContinuous(); sensorFR.startContinuous();

      Wire.beginTransmission(MPU6500_ADDR);
      Wire.write(0x6B); Wire.write(0x00); 
      Wire.endTransmission(); delay(50);

      Wire.beginTransmission(MPU6500_ADDR);
      Wire.write(0x1B); Wire.write(0x18); 
      Wire.endTransmission(); delay(50);

      Serial.println("Dang Calibrate MPU6500... Vui long giu xe dung im!");
      delay(1000); 
      CalibrateGyroZ();
      Serial.println("Calibrate Xong!");
      last_mpu_time = micros(); 
   }  

//   void CalibrateGyroZ() {
//      long sum = 0;
//      const int samples = 500;
//      for (int i = 0; i < samples; i++) {
//          Wire.beginTransmission(MPU6500_ADDR);
//          Wire.write(0x47); 
//          Wire.endTransmission(false);
//          Wire.requestFrom(MPU6500_ADDR, 2, true);
//          int16_t rawZ = (Wire.read() << 8) | Wire.read();
//          sum += rawZ;
//          delay(2);
//      }
//      gyroZ_offset = (float)sum / samples; 
//   }


   void CalibrateGyroZ() {
      // --- 1. CALIBRATE MPU (500 mẫu, tốn ~1 giây) ---
      long sumZ = 0;
      const int mpu_samples = 500;
      for (int i = 0; i < mpu_samples; i++) {
          Wire.beginTransmission(MPU6500_ADDR);
          Wire.write(0x47); 
          Wire.endTransmission(false);
          Wire.requestFrom(MPU6500_ADDR, 2, true);
          int16_t rawZ = (Wire.read() << 8) | Wire.read();
          sumZ += rawZ;
          delay(2);
      }
      gyroZ_offset = (float)sumZ / mpu_samples; 

      // --- 2. CALIBRATE WALL TARGET (10 mẫu, tốn ~0.3 giây) ---
      long sumSenL = 0;
      long sumSenR = 0;
      const int sensor_samples = 10;
      
      ActiveSensor = true; // Mở khoá hàm ReadSensor
      
      for (int i = 0; i < sensor_samples; i++) {
          ReadSensor(); // Hàm này tự delay ~30ms chờ phần cứng VL53L0X
          sumSenL += senL;
          sumSenR += senR;
          // Không cần thêm delay(2) ở đây nữa
      }
      ActiveSensor = false; // Khoá lại

      // 3. Tính trung bình
      float calib_L = (float)sumSenL / sensor_samples;
      float calib_R = (float)sumSenR / sensor_samples;
      
      // 4. Chốt số với bộ lọc an toàn (đề phòng xe đang để ngoài bàn trống)
      if(calib_L > 80.0f && calib_L < 150.0f) WALL_TARGET_L = calib_L;
      else WALL_TARGET_L = 119.0f;
      
      if(calib_R > 80.0f && calib_R < 150.0f) WALL_TARGET_R = calib_R;
      else WALL_TARGET_R = 119.0f;

      Serial.print("Da chot Wall Target -> L: "); 
      Serial.print(WALL_TARGET_L, 1);
      Serial.print(" mm | R: "); 
      Serial.println(WALL_TARGET_R, 1);
   }

   void ReadSensor(){
      if(!ActiveSensor) return;
      senL  = sensorL.readRangeContinuousMillimeters() - 6;
      senR  = sensorR.readRangeContinuousMillimeters() - 6;
      senFL = sensorFL.readRangeContinuousMillimeters() + 30;
      senFR = sensorFR.readRangeContinuousMillimeters();
   }

   int GetSenL()  { return senL; }
   int GetSenR()  { return senR; }
   int GetSenFL() { return senFL; }
   int GetSenFR() { return senFR; }

   void PrintSensor(){
      ReadSensor(); 
      static unsigned long last_print_time = 0;
      if (millis() - last_print_time > 50) { 
        last_print_time = millis(); 
        Serial.print("senFL: ");  Serial.print(senFL);
        Serial.print("  senL: "); Serial.print(senL);
        Serial.print("  senR: "); Serial.print(senR);
        Serial.print("  senFR: "); Serial.println(senFR);
      }
   }

   void ReadMPU() {
      Wire.beginTransmission(MPU6500_ADDR);
      Wire.write(0x47); 
      Wire.endTransmission(false); 
      Wire.requestFrom(MPU6500_ADDR, 2, true); 
      
      if(Wire.available() == 2) {
          int16_t rawZ = (Wire.read() << 8) | Wire.read();
          unsigned long current_time = micros();
          float dt = (current_time - last_mpu_time) / 1000000.0f;
          last_mpu_time = current_time;

          float dps = (rawZ - gyroZ_offset) / 16.4f;
          
          // Bộ lọc phần mềm chống nhiễu trôi tĩnh
          if (abs(dps) < 0.5f) dps = 0.0f; 

          current_omega = dps;
          
          const float GYRO_SCALE = 1.0032f; 
          float d_angle = (dps * dt) * GYRO_SCALE; 
          
          angleZ += d_angle; 
          angleZ_change += d_angle; 
      }
   }

   float GetAngleZ() { return angleZ; }
   float GetOmegaZ() { return current_omega; }
   
   float GetAndResetAngleZChange() {
      noInterrupts(); 
      float val = angleZ_change;
      angleZ_change = 0;
      interrupts();
      return val;
   }
  
   void ResetAngleZ() { 
      noInterrupts();
      angleZ = 0; 
      interrupts();
   }

   void PrintMPU() {
      ReadMPU(); 
      static unsigned long last_mpu_print_time = 0;
      if (millis() - last_mpu_print_time > 50) { 
        last_mpu_print_time = millis(); 
        Serial.print("Goc Z MPU: "); Serial.print(angleZ, 2);
        Serial.print(" do   |   Van toc xoay: "); Serial.print(current_omega, 2);
        Serial.println(" do/s");
      }
   }
};

#endif
