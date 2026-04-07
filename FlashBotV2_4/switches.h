#ifndef SWITCHES_H
#define SWITCHES_H

#include "config.h"
#include "I2C.h"     // Gọi I2C để xả cảm biến
#include "sensor.h"  // Gọi Sensor để enable

void SetupMode(){
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(Button, INPUT_PULLUP);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
}

int readSwitchMode(){
  int mode = 0;
  if(!digitalRead(SW1)) mode += 4;
  if(!digitalRead(SW2)) mode += 2;
  if(!digitalRead(SW3)) mode += 1;
  return mode;
}

void showModeLed(int mode){
  if(mode == 0){
    digitalWrite(LED4, LOW);
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
  }else {
    digitalWrite(LED4, HIGH);
    digitalWrite(LED1, (mode & 1) ? HIGH : LOW);
    digitalWrite(LED2, (mode & 2) ? HIGH : LOW);
    digitalWrite(LED3, (mode & 4) ? HIGH : LOW);
  }
}

void turnOffAllLeds(){
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, HIGH); // LED4 ngược logic
}

void blinkAllLeds(){
  sensor.enable();

  unsigned long start = millis();
  bool led_state = false;

  while(millis() - start < 2000){
    
    i2c.ReadSensor(); 
    i2c.ReadMPU();

    if((millis() - start) % 500 < 250){
      if(!led_state){
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, HIGH);
        digitalWrite(LED4, LOW); 
        led_state = true;
      }
    } else {
      if(led_state){
        turnOffAllLeds(); 
        led_state = false;
      }
    }
  }
  
  turnOffAllLeds();
  sensor.update();
}

void blinkLed() {
  unsigned long start = millis();
  bool led_state = false;

  while(millis() - start < 1000){
    if((millis() - start) % 500 < 250){
      if(!led_state){
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, HIGH);
        digitalWrite(LED4, LOW);
        led_state = true;
      }
    } else {
      if(led_state){
        turnOffAllLeds();
        led_state = false;
      }
    }
  }
  turnOffAllLeds();
}

bool checkButton(){
  if(digitalRead(Button) == LOW){
    delay(50);
    if(digitalRead(Button) == LOW) return true;
  }
  return false;
}

#endif
