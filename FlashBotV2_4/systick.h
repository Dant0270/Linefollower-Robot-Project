#ifndef SYSTICK_H
#define SYSTICK_H

#include <HardwareTimer.h>
#include "config.h"
#include "encoders.h"
#include "motors.h"
#include "motion.h"
#include "sensor.h"

void SysTick_ISR(); 

// 2. Định nghĩa toàn bộ Class Systick
class Systick {
 private:
  HardwareTimer *sysTimer;

 public:
  void begin() {
    sysTimer = new HardwareTimer(TIM4);  
    sysTimer->setOverflow(LOOP_FREQUENCY, HERTZ_FORMAT);
    sysTimer->attachInterrupt(SysTick_ISR);
    sysTimer->resume();
    delay(40);
  }

  void update() {
    encoders.update();
    motion.update();
    sensor.update();
    motors.updateControll(motion.velocity(), motion.omega(), sensor.GetSteerinFeedback()); 
  }
  
  void stop() {
    if(sysTimer) {
      sysTimer->pause();
    }
  }
};


extern Systick systick;
inline void SysTick_ISR() {
  systick.update();
}

#endif
