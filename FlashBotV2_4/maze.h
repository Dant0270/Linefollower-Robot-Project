#ifndef MAZE_H
#define MAZE_H

#include "config.h"
#include "queue.h"

class Maze {
public:
    uint8_t verticalWalls[MAZE_WIDTH + 1][MAZE_HEIGHT];
    uint8_t horizontalWalls[MAZE_WIDTH][MAZE_HEIGHT + 1];
    uint8_t travelArray[MAZE_WIDTH][MAZE_HEIGHT];
    
    int floodArray[MAZE_WIDTH][MAZE_HEIGHT];
    Heading pathArray[MAZE_WIDTH][MAZE_HEIGHT];

    Maze() { resetAll(); }

    void resetAll() {
        for(int x=0; x<=MAZE_WIDTH; x++)
            for(int y=0; y<=MAZE_HEIGHT; y++) {
                if(x < MAZE_WIDTH) horizontalWalls[x][y] = 0;
                if(y < MAZE_HEIGHT) verticalWalls[x][y] = 0;
            }
            
        for(int x=0; x<MAZE_WIDTH; x++)
            for(int y=0; y<MAZE_HEIGHT; y++) {
                travelArray[x][y] = 0;
                pathArray[x][y] = NORTH;
            }
        generateInitialWalls();
        resetFloodArray(1); 
    }

    void generateInitialWalls() {
        for (int x = 0; x < MAZE_WIDTH; x++) {
            horizontalWalls[x][0] = 1;               
            horizontalWalls[x][MAZE_HEIGHT] = 1;     
        }
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            verticalWalls[0][y] = 1;                 
            verticalWalls[MAZE_WIDTH][y] = 1;        
        }
    }

    void resetFloodArray(int targetMode) {
        for (int x = 0; x < MAZE_WIDTH; x++)
            for (int y = 0; y < MAZE_HEIGHT; y++)
                floodArray[x][y] = -1;

        if (targetMode == 1) { 
            for (int x = LOWER_X_GOAL; x <= UPPER_X_GOAL; x++)
                for (int y = LOWER_Y_GOAL; y <= UPPER_Y_GOAL; y++)
                    floodArray[x][y] = 0;
                    
        } else if (targetMode == 2) { 
            for (int x = LOWER_X_GOAL; x <= UPPER_X_GOAL; x++)
                for (int y = LOWER_Y_GOAL; y <= UPPER_Y_GOAL; y++) {
                    if (travelArray[x][y] == 0) {
                        floodArray[x][y] = 0;
                    }
                }
        } else { 
            floodArray[STARTING_X][STARTING_Y] = 0;
        }
    }

    bool checkWall(Heading heading, coord c) {
        switch (heading) {
            case NORTH: return horizontalWalls[c.x][c.y + 1];
            case WEST:  return verticalWalls[c.x][c.y];
            case SOUTH: return horizontalWalls[c.x][c.y];
            case EAST:  return verticalWalls[c.x + 1][c.y];
        }
        return true;
    }

    coord incrementCoord(Heading heading, coord c, int numCells) {
        switch (heading) {
            case NORTH: c.y += numCells; break;
            case WEST:  c.x -= numCells; break;
            case SOUTH: c.y -= numCells; break;
            case EAST:  c.x += numCells; break;
        }
        return c;
    }

    int getNeighborFlood(Heading heading, coord c) {
        coord nextC = incrementCoord(heading, c, 1);
        if (nextC.x < 0 || nextC.x >= MAZE_WIDTH || nextC.y < 0 || nextC.y >= MAZE_HEIGHT)
            return OUT_OF_BOUNDS;
        return floodArray[nextC.x][nextC.y];
    }

    void placeWall(Heading heading, coord c) {
        switch (heading) {
            case NORTH: horizontalWalls[c.x][c.y + 1] = 1; break;
            case WEST:  verticalWalls[c.x][c.y] = 1; break;
            case SOUTH: horizontalWalls[c.x][c.y] = 1; break;
            case EAST:  verticalWalls[c.x + 1][c.y] = 1; break;
        }
    }

    void floodFill(int targetMode, bool avoidVisited = false, bool speedRunMode = false) {
        resetFloodArray(targetMode);
        Queue q;

        for (int x = 0; x < MAZE_WIDTH; x++) {
            for (int y = 0; y < MAZE_HEIGHT; y++) {
                if (floodArray[x][y] == 0) {
                    q.push((neighbor){{x, y}, NORTH, 0});
                    q.push((neighbor){{x, y}, WEST, 0});
                    q.push((neighbor){{x, y}, SOUTH, 0});
                    q.push((neighbor){{x, y}, EAST, 0});
                }
            }
        }

        while (!q.isEmpty()) {
            neighbor current = q.pop();
            int currentVal = floodArray[current.c.x][current.c.y];

            Heading directions[] = {NORTH, WEST, SOUTH, EAST};
            for (int i = 0; i < 4; i++) {
                Heading dir = directions[i];
                if (!checkWall(dir, current.c)) {
                    
                    coord nextC = incrementCoord(dir, current.c, 1);

                    if (speedRunMode && travelArray[nextC.x][nextC.y] != 1) {
                            continue; 
                            }

                    int penalty = 0;
                    if (avoidVisited && travelArray[nextC.x][nextC.y] == 1) {
                        penalty = 200; 
                    }
                    
                    int nextVal = currentVal + TILE_SCORE + penalty;
                    neighbor next;
                    
                    if (current.heading != dir) {
                        nextVal += TURN_SCORE;
                        next.streak = 0;
                    } else {
                        nextVal += (STREAK_MULTIPLIER * (current.streak - 1)) + STREAK_SCORE;
                        next.streak = current.streak + 1;
                    }

                    next.c = nextC;
                    next.heading = dir;

                    int neighborVal = getNeighborFlood(dir, current.c);
                    if (neighborVal == NOT_YET_SET || nextVal < neighborVal) {
                        q.push(next);
                        floodArray[next.c.x][next.c.y] = nextVal;
                        pathArray[next.c.x][next.c.y] = (Heading)((dir + 2) % 4); 
                    }
                }
            }
        }
    }
};

extern Maze maze; 

#endif
