#ifndef NEWCLASS_H
#define NEWCLASS_H

#include <AP_HAL/AP_HAL.h>

class NewClass {
public:
    void init();          // set up UART, etc.
    void newMethod();     // runs at 1 Hz
};

#endif // NEWCLASS_H
