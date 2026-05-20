#ifndef SYS_ID_H
#define SYS_ID_H

#include "motors.h"
#include "encoders.h"
#include "mpu_spi.h"
#include "uart.h"

class SysID {
public:

    void calibrateBias() {
        Serial.println("--- CALIBRATING BIAS FF ---");
        float v = 0.0f;
        while (v < 2.5f) {
            v += 0.01f;
            motors.SetLeftMotorVolts(v);
            motors.SetRightMotorVolts(v);
            delay(30); 
            
            if (fabs(encoders.robot_speed()) > 5.0f) {
                Serial.print(">> Result: BiasFF = ");
                Serial.println((float)v, 3); 
                break;
            }
        }
        motors.stop();
        delay(1000);
    }

   
    void testForwardStep(float testVolts) {
        Serial.println("--- FWD STEP RESPONSE START ---");
        Serial.println("Time(ms),Speed(mm/s)");
        uint32_t start_t = millis();
        
        while (millis() - start_t < 800) { 
            motors.SetLeftMotorVolts(testVolts);
            motors.SetRightMotorVolts(testVolts);
            

            Serial.print((int)(millis() - start_t)); 
            Serial.print(",");

            Serial.println((int)encoders.robot_speed()); 
            delay(5);
        }
        motors.stop();
        Serial.println("--- FWD STEP DONE ---");
    }


    void testRotationStep(float testVolts) {
        Serial.println("--- ROT STEP RESPONSE START ---");
        Serial.println("Time(ms),Omega(deg/s)");
        uint32_t start_t = millis();
        
        while (millis() - start_t < 800) {

            mpu.ReadMPU(); 

            motors.SetLeftMotorVolts(-testVolts);
            motors.SetRightMotorVolts(testVolts);
            Serial.print((int)(millis() - start_t));
            Serial.print(",");

            Serial.println((int)mpu.GetOmegaZ()); 
            delay(5);
        }
        motors.stop();
        Serial.println("--- ROT STEP DONE ---");
    }
};

extern SysID sysID;
#endif