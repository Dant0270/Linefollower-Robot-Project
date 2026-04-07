#ifndef MOUSE_H
#define MOUSE_H

#include "config.h"
#include "maze.h"
#include "motion.h"
#include "sensor.h"
#include "I2C.h"
#include "stm32f4xx_hal.h" 


#define FLASH_STORAGE_ADDRESS 0x08060000 
#define FLASH_STORAGE_SECTOR  FLASH_SECTOR_7 

typedef struct {
    uint8_t checkByte; 
    uint8_t horizontalWalls[MAZE_WIDTH][MAZE_HEIGHT + 1];
    uint8_t verticalWalls[MAZE_WIDTH + 1][MAZE_HEIGHT];
    uint8_t travelArray[MAZE_WIDTH][MAZE_HEIGHT];
} MazeMapData;

class Mouse {
private:
    coord currentXY;        
    Heading currentHeading; 
    int targetMode;         
    bool m_handStart = false;
    
    void executeContinuousTurn(int turn_id, Heading nextDir, float finalSpeed = SEARCH_SPEED) {
        sensor.SetSteeringMode(STEERING_OFF);
        TurnParameters tp = turn_params[turn_id];
        motion.SetTargetVelocity(tp.speed); 
        float trigger = tp.trigger;
        if (sensor.SeeLeftWall) { trigger += 0; }
        if (sensor.SeeRightWall) { trigger += 0; }
        
        float turn_point = FULL_CELL + HALF_CELL - tp.entry_offset;
        
        while (motion.position() < turn_point) {
            i2c.ReadSensor(); 
            i2c.ReadMPU(); 
            sensor.update();
            if (sensor.GetFrontSum() < trigger) {
                motion.SetTargetVelocity(motion.velocity()); 
                break;
            }
            delay(2);
        }

        i2c.ResetAngleZ(); 
        motors.ResetControll();
        
        float target_angle = tp.angle; 

        motion.StartTurn(target_angle, tp.omega, 0, tp.alpha);
        while(!motion.TurnFinished()) {i2c.ReadSensor(); i2c.ReadMPU(); delay(2); }

        int timeout = 0;
        float wait_angle = abs(tp.angle) - 5.0f; 
        while(abs(i2c.GetAngleZ()) < wait_angle && timeout < 50) {
             i2c.ReadSensor(); i2c.ReadMPU();delay(2);
            timeout++;
        }
        
        i2c.ResetAngleZ(); 
        motors.ResetControll();

        int end_point = HALF_CELL + tp.exit_offset;
        
        motion.StartMove(SENSING_POSITION - end_point, motion.velocity(), finalSpeed, SEARCH_ACCELERATION);
        while(!motion.MoveFinished()) { delay(2); i2c.ReadSensor(); i2c.ReadMPU(); sensor.update(); } 

        motion.SetPosition(SENSING_POSITION);
        currentHeading = nextDir; 
    }

    void updateRealWalls() {
        sensor.update(); 
        if (sensor.SeeFrontWall) maze.placeWall(currentHeading, currentXY);
        if (sensor.SeeLeftWall)  maze.placeWall((Heading)((currentHeading + 1) % 4), currentXY);
        if (sensor.SeeRightWall) maze.placeWall((Heading)((currentHeading + 3) % 4), currentXY); 
    }

    bool mouseInGoal() {
        return (targetMode == 1 && 
               (currentXY.x >= LOWER_X_GOAL && currentXY.x <= UPPER_X_GOAL && 
                currentXY.y >= LOWER_Y_GOAL && currentXY.y <= UPPER_Y_GOAL));
    }
    bool allGoalCellsVisited() {
        return maze.travelArray[LOWER_X_GOAL][LOWER_Y_GOAL] && 
               maze.travelArray[UPPER_X_GOAL][LOWER_Y_GOAL] && 
               maze.travelArray[LOWER_X_GOAL][UPPER_Y_GOAL] && 
               maze.travelArray[UPPER_X_GOAL][UPPER_Y_GOAL];
    }
    int getOptimalPathLength() {
        maze.floodFill(1, false); 
        return maze.floodArray[STARTING_X][STARTING_Y];
    }

public:
    Mouse() {
        currentXY = {STARTING_X, STARTING_Y}; 
        currentHeading = NORTH;               
        targetMode = 1;                       
    }

    void printRawFlash() {
        Serial.println("\n=== KIEM TRA O DA DI TRONG FLASH (1=DA DI) ===");
        for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {
            if(y < 10) Serial.print(" ");
            Serial.print(y); Serial.print("| ");
            for (int x = 0; x < MAZE_WIDTH; x++) {
                Serial.print(maze.travelArray[x][y]);
                Serial.print(" ");
            }
            Serial.println();
        }
        Serial.println("   -------------------------------");
        Serial.println("    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5");
    }

    void printMaze() {
        Serial.println("\n=== BAN DO ME CUNG & FLOODFILL ===");
        for (int y = MAZE_HEIGHT - 1; y >= 0; y--) {
            for (int x = 0; x < MAZE_WIDTH; x++) {
                Serial.print("+");
                Serial.print(maze.horizontalWalls[x][y + 1] ? "---" : "   ");
            }
            Serial.println("+");
            for (int x = 0; x < MAZE_WIDTH; x++) {
                Serial.print(maze.verticalWalls[x][y] ? "|" : " ");
                if (currentXY.x == x && currentXY.y == y) {
                    if (currentHeading == NORTH) Serial.print(" ^ ");
                    else if (currentHeading == SOUTH) Serial.print(" v ");
                    else if (currentHeading == EAST) Serial.print(" > ");
                    else if (currentHeading == WEST) Serial.print(" < ");
                } else {
                    int val = maze.floodArray[x][y];
                    if (val == -1) Serial.print(" x "); 
                    else {
                        if (val < 10) Serial.print("  ");
                        else if (val < 100) Serial.print(" ");
                        Serial.print(val);
                    }
                }
            }
            Serial.println(maze.verticalWalls[MAZE_WIDTH][y] ? "|" : " ");
        }
        for (int x = 0; x < MAZE_WIDTH; x++) {
            Serial.print("+");
            Serial.print(maze.horizontalWalls[x][0] ? "---" : "   ");
        }
        Serial.println("+\n");
    }

    void saveMapToFlash() {
        MazeMapData dataToSave;
        dataToSave.checkByte = 111; 
        
        memcpy(dataToSave.horizontalWalls, maze.horizontalWalls, sizeof(maze.horizontalWalls));
        memcpy(dataToSave.verticalWalls, maze.verticalWalls, sizeof(maze.verticalWalls));
        memcpy(dataToSave.travelArray, maze.travelArray, sizeof(maze.travelArray));

        HAL_FLASH_Unlock();

        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t SectorError = 0;
        EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS; 
        EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3; 
        EraseInitStruct.Sector        = FLASH_STORAGE_SECTOR;
        EraseInitStruct.NbSectors     = 1; 

        if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
            Serial.println(">> LOI: KHONG THE XOA FLASH!");
            HAL_FLASH_Lock();
            return;
        }

        uint32_t dataSize = sizeof(MazeMapData);
        uint32_t numWords = (dataSize + 3) / 4; 
        uint32_t *dataPtr = (uint32_t *)&dataToSave;
        uint32_t currentAddr = FLASH_STORAGE_ADDRESS;

        for (uint32_t i = 0; i < numWords; i++) {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, currentAddr, dataPtr[i]) != HAL_OK) {
                Serial.println(">> LOI: GHI FLASH THAT BAI!");
                break;
            }
            currentAddr += 4; 
        }

        HAL_FLASH_Lock();
        Serial.println(">> DA LUU TOAN BO MAP VAO FLASH THANH CONG (F4)!");
    }

    void loadMapFromFlash() {
        MazeMapData loadedData;
        
        memcpy(&loadedData, (const void*)FLASH_STORAGE_ADDRESS, sizeof(MazeMapData));

        if (loadedData.checkByte != 111) {
            Serial.println(">> CANH BAO: FLASH TRONG HOAC CHUA CO MAP! Khong the tai map.");
            maze.resetAll();
            return;
        }

        memcpy(maze.horizontalWalls, loadedData.horizontalWalls, sizeof(loadedData.horizontalWalls));
        memcpy(maze.verticalWalls, loadedData.verticalWalls, sizeof(loadedData.verticalWalls));
        memcpy(maze.travelArray, loadedData.travelArray, sizeof(loadedData.travelArray));
        
        Serial.println(">> DA TAI MAP TU FLASH THANH CONG (F4)!");
    }
    
    void move_ahead(float targetSpeed = SEARCH_SPEED) {
        sensor.SetSteeringMode(STEER_NORMAL); 
        
        motion.SetTargetVelocity(targetSpeed); 
        
        motion.adjust_forward_position(-FULL_CELL); 
        while (motion.position() < SENSING_POSITION) { 
            i2c.ReadSensor(); 
            i2c.ReadMPU();
            sensor.update(); 
            delay(2); 
        }
    }

    void move_ahead2() {
        sensor.SetSteeringMode(STEER_NORMAL); 
        motion.adjust_forward_position(-FULL_CELL); 
        float remaining = SENSING_POSITION - motion.position();
        motion.StartMove(remaining, SEARCH_SPEED, 0, SEARCH_ACCELERATION); 
        while (!motion.MoveFinished()) { 
            i2c.ReadSensor(); 
            i2c.ReadMPU();
            sensor.update(); 
            delay(2); 
        }
        sensor.SetSteeringMode(STEERING_OFF); 
        motors.ResetControll();               
        motion.stop();
        motion.SetPosition(SENSING_POSITION);
    }


void case3_searchAndReturn() {
    motion.ResetDriveSystem();
    maze.resetAll();
    currentXY = {STARTING_X, STARTING_Y + 1};
    currentHeading = NORTH;

    maze.travelArray[STARTING_X][STARTING_Y] = 1;     
    maze.travelArray[currentXY.x][currentXY.y] = 1;
    
    maze.placeWall(WEST, {STARTING_X, STARTING_Y});
    maze.placeWall(EAST, {STARTING_X, STARTING_Y});
    maze.placeWall(SOUTH, {STARTING_X, STARTING_Y});
  
    sensor.enable();
    sensor.update();

    sensor.WaitStart();
    blinkAllLeds();
    motion.ResetDriveSystem();
    sensor.SetSteeringMode(STEERING_OFF); 

    motion.move(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    while (!motion.MoveFinished()) { i2c.ReadSensor();i2c.ReadMPU(); sensor.update(); delay(2); }
    motion.SetPosition(HALF_CELL);
    motion.wait_until_position(SENSING_POSITION);

    int searchPhase = 1;

    while (true) {
        sensor.SetSteeringMode(STEER_NORMAL); 
        maze.travelArray[currentXY.x][currentXY.y] = 1;
        i2c.ReadSensor(); 
        i2c.ReadMPU();
        sensor.update(); 
        updateRealWalls(); 
        
        if (searchPhase == 1 && mouseInGoal()) {
            Serial.println(">> DA VAO DICH! CHUYEN SANG PHA 2: KHAM PHA 4 O...");
            searchPhase = 2;
        }
        
        if (searchPhase == 2 && allGoalCellsVisited()) {
            Serial.println(">> DA QUET XONG 4 O DICH! CHUYEN SANG PHA 3: TIM DUONG VE...");
            searchPhase = 3;
        }

        if (searchPhase == 3 && currentXY.x == STARTING_X && currentXY.y == STARTING_Y) {
            motion.StartMove(110.0f, SEARCH_SPEED, 0, SEARCH_ACCELERATION); 
            while (!motion.MoveFinished()) { i2c.ReadSensor();i2c.ReadMPU(); sensor.update(); delay(2); }
            maze.travelArray[currentXY.x][currentXY.y] = 1; 
            break; 
        }

        if (searchPhase == 1) targetMode = 1; 
        else if (searchPhase == 2) targetMode = 2; 
        else if (searchPhase == 3) targetMode = 0; 

        bool avoidVisitedPaths = (searchPhase == 3); 
        maze.floodFill(targetMode, avoidVisitedPaths, false);  
        
        Heading nextHeading = maze.pathArray[currentXY.x][currentXY.y];
        int hdgChange = (nextHeading - currentHeading + 4) % 4;
                    
        currentXY = maze.incrementCoord(nextHeading, currentXY, 1); 

        switch (hdgChange) {
            case 0: move_ahead(); break;
            case 1: turn_left(nextHeading); break;
            case 2: turn_back(); break;
            case 3: turn_right(nextHeading); break;
        }
    }

  sensor.SetSteeringMode(STEERING_OFF); 
  motors.ResetControll();               
  motion.stop();
  saveMapToFlash(); 
  Serial.println("DO MAP HOAN THANH VA DA LUU FLASH!");
}

    void turn_right(Heading nextDir) { executeContinuousTurn(3, nextDir, SEARCH_SPEED); }
    void turn_left(Heading nextDir) { executeContinuousTurn(2, nextDir, SEARCH_SPEED); }
    void turn_right2(Heading nextDir) { executeContinuousTurn(1, nextDir, SEARCH_SPEED); }
    void turn_left2(Heading nextDir) { executeContinuousTurn(0, nextDir, SEARCH_SPEED); }

   void turn_back() {
        Serial.println("Ngo cut! Quay dau...");
        
        motion.StartMove(70.0f, SEARCH_SPEED, 0, SEARCH_ACCELERATION);
        while (!motion.MoveFinished()) { i2c.ReadSensor();i2c.ReadMPU(); delay(2); }
         
        motion.ResetDriveSystem();
        sensor.SetSteeringMode(STEERING_OFF);
        motors.ResetControll();
        
        motion.spinTurn(180.0f, OMEGA_SPIN_TURN, ALPHA_SPIN_TURN);
        while(!motion.TurnFinished()) {  i2c.ReadSensor(); i2c.ReadMPU(); delay(2);}
        
        currentHeading = (Heading)((currentHeading + 2) % 4);
        sensor.SetSteeringMode(STEERING_OFF);
   
        motion.StartMove(-120.0f, 150, 0, 500); 
        while (!motion.MoveFinished()) { i2c.ReadSensor();i2c.ReadMPU(); sensor.update(); delay(2); }
         

        motion.stop();
        motors.ResetControll(); 
       
        motion.ResetDriveSystem(); 
        sensor.SetSteeringMode(STEERING_OFF); 
        motion.StartMove(70.0f, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
        while(!motion.MoveFinished()) { i2c.ReadSensor();i2c.ReadMPU();delay(2); }

        motion.SetPosition(SENSING_POSITION);
    }


void case1_searchToGoal() {
    motion.ResetDriveSystem();
    maze.resetAll();
    currentXY = {STARTING_X, STARTING_Y+1};
    currentHeading = NORTH;

    targetMode = 1;
    maze.travelArray[STARTING_X][STARTING_Y] = 1;     
    maze.travelArray[currentXY.x][currentXY.y] = 1;
    
    maze.placeWall(WEST, {STARTING_X, STARTING_Y});
    maze.placeWall(EAST, {STARTING_X, STARTING_Y});
    maze.placeWall(SOUTH, {STARTING_X, STARTING_Y});
  
    sensor.enable();
    sensor.update();

    sensor.WaitStart();
    blinkAllLeds();
    motion.ResetDriveSystem();
    sensor.SetSteeringMode(STEERING_OFF); 

    motion.move(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    while (!motion.MoveFinished()) { i2c.ReadSensor();i2c.ReadMPU(); sensor.update(); delay(2); }
    motion.SetPosition(HALF_CELL);
    motion.wait_until_position(SENSING_POSITION);

    while (true) {
        sensor.SetSteeringMode(STEER_NORMAL); 
        maze.travelArray[currentXY.x][currentXY.y] = 1;
        i2c.ReadSensor(); 
        i2c.ReadMPU();
        sensor.update(); 
        updateRealWalls(); 
        
        maze.floodFill(targetMode,false,false);  
        Heading nextHeading = maze.pathArray[currentXY.x][currentXY.y];
        
        if (mouseInGoal()) {
            motion.StartMove(110.0f, SEARCH_SPEED, 0, SEARCH_ACCELERATION); 
            while (!motion.MoveFinished()) { i2c.ReadSensor();i2c.ReadMPU(); sensor.update(); delay(2); }
            
            maze.travelArray[currentXY.x][currentXY.y] = 1; 
            Serial.println("DA DEN DICH THANH CONG VA LUU FLASH!");
            break; 
        }

        int hdgChange = (nextHeading - currentHeading + 4) % 4;
                    
        currentXY = maze.incrementCoord(nextHeading, currentXY, 1); 

        switch (hdgChange) {
            case 0: move_ahead(); break;
            case 1: turn_left(nextHeading); break;
            case 2: turn_back(); break;
            case 3: turn_right(nextHeading); break;
        }
    }

  sensor.SetSteeringMode(STEERING_OFF); 
  motors.ResetControll();               
  motion.stop();
  saveMapToFlash();
  blinkAllLeds();
  Serial.println("DA DEN DICH THANH CONG!");
}

void case2_speedRun() {
    motion.ResetDriveSystem();
    maze.resetAll();
    currentXY = {STARTING_X, STARTING_Y + 1};
    currentHeading = NORTH;
    
    loadMapFromFlash(); 
    
    // --- BƯỚC 1: TÍNH CHIỀU DÀI ĐƯỜNG CŨ TRƯỚC KHI CHẠY ---
    targetMode = 1; 
    maze.floodFill(targetMode, false, false); 
    int oldPathLength = maze.floodArray[STARTING_X][STARTING_Y];
    
    // Quét lại floodFill cho Phase 1 (cho phép đi chéo/tăng tốc)
    maze.floodFill(targetMode, false, true);  

    sensor.WaitStart();
    blinkAllLeds();
    sensor.SetSteeringMode(STEERING_OFF);
    sensor.enable();
    motors.ResetControll();               
    motion.move(BACK_WALL_TO_CENTER, SEARCH_SPEED, SEARCH_SPEED, SEARCH_ACCELERATION);
    while (!motion.MoveFinished()) { i2c.ReadSensor(); i2c.ReadMPU(); sensor.update(); delay(2); }
    motion.SetPosition(HALF_CELL);
    motion.wait_until_position(SENSING_POSITION);

    int runPhase = 1; 

    while (true) {
        bool phaseChanged = false; 
        Heading nextHeading = maze.pathArray[currentXY.x][currentXY.y];
        
        if (runPhase == 1 && mouseInGoal()) {
         
            motion.StartMove(110, SEARCH_SPEED, 0, SEARCH_ACCELERATION); 
            while (!motion.MoveFinished()) { i2c.ReadSensor(); i2c.ReadMPU(); sensor.update(); delay(2); }
            
            runPhase = 2;
            targetMode = 0;
            maze.travelArray[currentXY.x][currentXY.y] = 1; 
            phaseChanged = true;
        } 
        else if (runPhase == 2 && currentXY.x == STARTING_X && currentXY.y == STARTING_Y) {
           
            motion.StartMove(110, SEARCH_SPEED, 0, SEARCH_ACCELERATION); 
            while (!motion.MoveFinished()) { i2c.ReadSensor(); i2c.ReadMPU(); sensor.update(); delay(2); }
            
            maze.travelArray[currentXY.x][currentXY.y] = 1; 
            break; 
        }

       
        if (!phaseChanged) {
            if (runPhase == 1) {
                int hdgChange = (nextHeading - currentHeading + 4) % 4;

                if (hdgChange == 0) { 
                
                    int straightCells = 0;
                    coord scanXY = currentXY;

                    while (true) {
                        straightCells++;
                        coord nextScanXY = maze.incrementCoord(nextHeading, scanXY, 1);

                        if (targetMode == 1 && 
                           (nextScanXY.x >= LOWER_X_GOAL && nextScanXY.x <= UPPER_X_GOAL && 
                            nextScanXY.y >= LOWER_Y_GOAL && nextScanXY.y <= UPPER_Y_GOAL)) {
                            break; 
                        }

                        Heading nextNextHeading = maze.pathArray[nextScanXY.x][nextScanXY.y];
                        if (nextNextHeading != nextHeading) {
                            break; 
                        }
                        
                        scanXY = nextScanXY; 
                    }

                    for (int i = 0; i < straightCells; i++) {
                        maze.travelArray[currentXY.x][currentXY.y] = 1; 
                        currentXY = maze.incrementCoord(currentHeading, currentXY, 1); 

                        if (i == straightCells - 1) {
                            move_ahead(450.0f);  
                        } else {
                            move_ahead(1000.0f); 
                        }
                    }
                } 
                else { 
                  
                    maze.travelArray[currentXY.x][currentXY.y] = 1; 
                    currentXY = maze.incrementCoord(nextHeading, currentXY, 1); 

                    switch (hdgChange) {
                        case 1: turn_left(nextHeading); break;
                        case 2: turn_back(); break;
                        case 3: turn_right(nextHeading); break;
                    }
                }
            } 
            else if (runPhase == 2) {
                sensor.SetSteeringMode(STEER_NORMAL); 
                maze.travelArray[currentXY.x][currentXY.y] = 1;
                
                i2c.ReadSensor(); 
                i2c.ReadMPU(); 
                sensor.update(); 
                updateRealWalls(); 
                
                maze.floodFill(targetMode, true, false);  
                
                nextHeading = maze.pathArray[currentXY.x][currentXY.y];
                int hdgChange = (nextHeading - currentHeading + 4) % 4;
                            
                currentXY = maze.incrementCoord(nextHeading, currentXY, 1); 

                switch (hdgChange) {
                    case 0: move_ahead(); break; 
                    case 1: turn_left(nextHeading); break;
                    case 2: turn_back(); break;
                    case 3: turn_right(nextHeading); break;
                }
            }
        }
    }

    sensor.SetSteeringMode(STEERING_OFF); 
    motors.ResetControll();               
    motion.stop();
    
    targetMode = 1; 
    maze.floodFill(targetMode, false, false); 
    int newPathLength = maze.floodArray[STARTING_X][STARTING_Y];
    
    if (newPathLength < oldPathLength) {
        saveMapToFlash();
    }
    
    blinkAllLeds();
    Serial.println("SPEED RUN HOAN THANH!");
}

};

extern Mouse mouse;

#endif
