#ifndef QUEUE_H
#define QUEUE_H

#include "config.h"

enum Heading { NORTH = 0, WEST = 1, SOUTH = 2, EAST = 3 };

struct coord {
    int x;
    int y;
};

struct neighbor {
    coord c;
    Heading heading;
    int streak;
};

class Queue {
private:
    neighbor data[256];
    int head;
    int tail;
    int count;

public:
    Queue() { reset(); }

    void reset() { head = 0; tail = 0; count = 0; }
    bool isEmpty() { return count == 0; }

    void push(neighbor elem) {
        if (count < 256) {
            data[tail] = elem;
            tail = (tail + 1) % 256;
            count++;
        }
    }

    neighbor pop() {
        neighbor elem = {{0, 0}, NORTH, 0}; 
        if (count > 0) {
            elem = data[head];
            head = (head + 1) % 256;
            count--;
        }
        return elem;
    }
};

#endif
