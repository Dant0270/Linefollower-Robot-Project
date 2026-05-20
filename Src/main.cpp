#include "config.h"
#include "uart.h"
#include "mpu_spi.h"
#include "switches.h"
#include "motors.h"
#include "encoders.h"
#include "systick.h"
#include "motion.h"
#include "batery.h"
#include "adc.h"
#include "SysID.h"

int currentMode = 0;
int botState = 0;

HardwareSerial Serial;
MPU6500_SPI mpu;
Encoders encoders;
Motors motors;
Motion motion;
Profile forward;
Profile rotation;
Systick systick;
ADC_Sensor adc;   
Battery battery;
SysID sysID;
Buzzer buzzer;
LEDBlinker ledBlinker;

void SystemClock_Config(void) {
  RCC -> CR |= RCC_CR_HSION;
  while (!(RCC -> CR & RCC_CR_HSIRDY));

  RCC -> CFGR |= RCC_CFGR_PPRE1_DIV2;
  RCC -> PLLCFGR = (16ul | (336ul << 6) | (1ul << 16) | RCC_PLLCFGR_PLLSRC_HSI);
  RCC -> CR |= RCC_CR_PLLON;
  while (!(RCC -> CR & RCC_CR_PLLRDY));

  FLASH -> ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS;
  RCC -> CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC -> CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

  SystemCoreClockUpdate();
}

void testWallFollow3Cells() {
 
  motion.ResetDriveSystem(); 
  motors.ResetControll(); 
  
  motion.StartMove(1000.0f , 300, 0, 2000);
  
  while(!motion.MoveFinished()) {
    
    delay(2);
  }
  motors.ResetControll();               
  motion.stop();
}

int main(void) {
  SystemClock_Config();
  DWT_Init();
	disableBuzzer();
  Serial.begin(115200);

  SetupMode();
  mpu.begin();
  encoders.begin();
  motors.begin();
	adc.begin();            
  adc.update();           
  battery.update();   
  motion.DisableDrive();
  motors.stop();
  systick.begin();
  battery.print_v();
  buzzer.begin();
  buzzer.beep(1047, 100);
  delay(50);
  buzzer.beep(1568, 150);

  while (1) {
    if (botState >= 0) {
      mpu.ReadMPU();
    }

    if (Serial.available()) {
      char cmd = Serial.read();
      if (cmd == 'c') {
        Serial.println("Calibrate MPU...");
        mpu.calibrateWithLEDs();
      }
    }

    if (botState == 0) {
      currentMode = updateEncoderMode();

      if (checkButton()) {
        buzzer.beep(880, 100);
        buzzer.beep(1760, 200);
        ledBlinker.start(1000);

        mpu.calibrateWithLEDs();

        buzzer.beep(2093, 100);
        delay(50);
        buzzer.beep(2093, 100);
        ledBlinker.start(1000);

        motion.ResetDriveSystem();
        botState = 1;
        Serial.print("Da chon Mode: ");
        Serial.println(currentMode);
      }
    } else if (botState == 1) {
      if (checkButton()) {
        buzzer.beep(2000, 500);
        botState = 2;
      }
    } else if (botState == 2) {
      ledBlinker.update();
      buzzer.update();
      
      switch (currentMode) {
      case 0:
        motors.DisableControll();
        sysID.calibrateBias();
        botState = 1;           
        break;

      case 1:
        motors.DisableControll(); 
        sysID.testForwardStep(3.0f);
        botState = 1;           
        break;

      case 2:
        motors.DisableControll();
        sysID.testRotationStep(3.0f);
        botState = 1;             
        break;

      case 3:
        motion.StartMove(1000.0f, 2500.0f, 0, 4000.0f);
        while (!motion.MoveFinished()) {
          mpu.ReadMPU();
          delay(2);
        }
        botState = 1;
        break;

      case 4:
        motion.spinTurn(3600.0f, 500.0f, 1500.0f);
			while (!motion.MoveFinished()) {
          mpu.ReadMPU();
				mpu.PrintMPU();
          delay(2);
        }
        botState = 1;
        break;
      }

      if (checkButton()) {
        motion.DisableDrive();
        motors.stop();
        botState = 0;
        buzzer.beep(1047, 100);
        buzzer.beep(523, 200);
        delay(500);
      }
    }
    
    ledBlinker.update();
    buzzer.update();
  }
}