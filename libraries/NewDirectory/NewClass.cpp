#include "NewClass.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

void NewClass::newMethod() {
    // Print to SITL console / USB console
    hal.console->printf("NewClass running at 1Hz\r\n");
    gcs().send_text(MAV_SEVERITY_INFO, "NewClass running at 1Hz");
}
