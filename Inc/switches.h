#ifndef SWITCHES_H
#define SWITCHES_H

#include "config.h"
#include "encoders.h"
#include "motors.h"

/**
 * @brief Buzzer với Timer PWM (TIM3) - Non-blocking
 * @note  Sử dụng hardware PWM thay vì software toggle
 */
class Buzzer {
  private:
    bool enabled = true;
    bool isPlaying = false;
    uint32_t beepEndTime = 0;
    
  public:
    void begin() {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        
        GPIOB->MODER |= (2 << (4 * 2));
        GPIOB->AFR[0] |= (2 << ((4 - 0) * 4));
        
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
        TIM3->PSC = 84 - 1;  // 1MHz tick
        TIM3->ARR = 1000;
        TIM3->CCR1 = 500;
        TIM3->CCMR1 = (6 << 4) | TIM_CCMR1_OC1PE;
        TIM3->CCER = TIM_CCER_CC1E;
        TIM3->CR1 = TIM_CR1_CEN;
        stop();
    }
    
    void enable() { enabled = true; }
    void disable() { enabled = false; stop(); }
    
    void beep(int freq, int duration_ms) {
        if (freq <= 0 || !enabled) {
            return;
        }
        uint32_t arr = 1000000U / freq;
        TIM3->ARR = arr;
        TIM3->CCR1 = arr / 2;
        beepEndTime = millis() + duration_ms;
        isPlaying = true;
    }
    
    void stop() {
        TIM3->CCR1 = 0;
        isPlaying = false;
    }
    
    void update() {
        if (isPlaying && millis() >= beepEndTime) {
            stop();
        }
    }
    
    bool isActive() { return isPlaying; }
};

extern Buzzer buzzer;

inline void enableBuzzer() { buzzer.enable(); }
inline void disableBuzzer() { buzzer.disable(); }

const int PULSES_PER_MODE = 50;
const int MAX_MODE = 15;
extern long lastRawCount;
extern long virtualCount;
extern int lastHapticMode;

inline void turnOffAllLeds() {
  GPIOC -> BSRR = GPIO_BSRR_BR13; 
  GPIOA -> BSRR = GPIO_BSRR_BR7;
  GPIOB -> BSRR = GPIO_BSRR_BR0 | GPIO_BSRR_BR1;
}

inline void triggerHapticFeedback(bool direction) {
  buzzer.beep(3500, 5); 
  
  if (direction) motors.SetRightMotorVolts(-10.0f); 
  else motors.SetRightMotorVolts(10.0f);
  
  delay(5);
  motors.stop();
}

inline void triggerWallBlock(bool resistForward) {
  buzzer.beep(150, 15); 
  
  if (resistForward) motors.SetRightMotorVolts(-4.0f);
  else motors.SetRightMotorVolts(4.0f);
  
  delay(15);
  motors.stop();
}

void SetupMode();
void showModeLed(int mode);
int updateEncoderMode();
bool checkButton();

/**
 * @brief Non-blocking LED blink với state machine
 */
class LEDBlinker {
  private:
    bool active = false;
    uint32_t endTime = 0;
    uint32_t intervalMs = 200;
    uint32_t lastToggle = 0;
    bool ledState = false;
    
  public:
    void start(int duration_ms, int interval = 200) {
        active = true;
        endTime = millis() + duration_ms;
        intervalMs = interval;
        lastToggle = millis();
        ledState = false;
        turnOffAllLeds();
    }
    
    void stop() {
        active = false;
        turnOffAllLeds();
    }
    
    void update() {
        if (!active) return;
        
        if (millis() >= endTime) {
            stop();
            return;
        }
        
        if (millis() - lastToggle >= intervalMs) {
            ledState = !ledState;
            if (ledState) {
                GPIOC->BSRR = GPIO_BSRR_BS13;
                GPIOA->BSRR = GPIO_BSRR_BS7;
                GPIOB->BSRR = GPIO_BSRR_BS0 | GPIO_BSRR_BS1;
            } else {
                turnOffAllLeds();
            }
            lastToggle = millis();
        }
    }
    
    bool isActive() { return active; }
};

void blinkLedsFor(int duration);

#endif
