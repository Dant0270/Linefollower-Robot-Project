#ifndef SYSTICK_H
#define SYSTICK_H

#include "config.h"
#include "encoders.h"
#include "motors.h"
#include "motion.h"
#include "batery.h"
#include "adc.h"

class Systick {
 public:
  void begin() {
    SysTick_Config(SystemCoreClock / 1000);
  }

  void update() {
		
		static int batteryTick = 0;
    if (++batteryTick >= 100) {
        batteryTick = 0;
        battery.update();
    }
    encoders.update();
    motion.update();
    motors.updateControll(motion.velocity(), motion.omega(), 0.0f); 
  }
  
  void stop() {
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; 
  }
};

extern Systick systick;

extern "C" {
    void SysTick_Handler(void) {
        systick.update();
    }
}

#endif