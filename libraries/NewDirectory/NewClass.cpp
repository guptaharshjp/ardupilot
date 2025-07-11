
// WORKING  RELAY CODE>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// #include "NewDirectory/NewClass.h"
// #include <AP_HAL/AP_HAL.h>
// #include <GCS_MAVLink/GCS.h>
// #include <AP_AHRS/AP_AHRS.h>
// #include <cmath>

// extern const AP_HAL::HAL& hal;

// #define LED_PIN 107
// #define PITCH_ROLL_THRESHOLD 10.0f

// void NewClass::init() {
//     hal.gpio->pinMode(LED_PIN, HAL_GPIO_OUTPUT);
//     gcs().send_text(MAV_SEVERITY_INFO, "✅ NewClass initialized - GPIO 107 (MAIN OUT 7 / RELAY1) control ready");
// }

// void NewClass::update() {
//     static uint32_t last_message_time = 0;
//     static uint32_t last_debug_time = 0;
//     uint32_t now = AP_HAL::millis();

//     if (now - last_message_time < 2000) {
//         return;
//     }
//     last_message_time = now;

//     const float pitch_deg = degrees(AP::ahrs().get_pitch());
//     const float roll_deg  = degrees(AP::ahrs().get_roll());

//     char msg[60];
//     hal.util->snprintf(msg, sizeof(msg), "Pitch: %.2f° Roll: %.2f°", pitch_deg, roll_deg);
//     gcs().send_text(MAV_SEVERITY_INFO, "%s", msg);

//     if (fabsf(pitch_deg) > PITCH_ROLL_THRESHOLD || fabsf(roll_deg) > PITCH_ROLL_THRESHOLD) {
//         hal.gpio->write(LED_PIN, 1);  // ✅ LED ON
//         hal.util->snprintf(msg, sizeof(msg), "⚠️ Threshold Exceeded! Pitch: %.2f° Roll: %.2f° LED ON", pitch_deg, roll_deg);
//         gcs().send_text(MAV_SEVERITY_WARNING, "%s", msg);
//     } else {
//         hal.gpio->write(LED_PIN, 0);  // ✅ LED OFF
//         if (now - last_debug_time > 10000) {
//             gcs().send_text(MAV_SEVERITY_INFO, "LED OFF on GPIO 107");
//             last_debug_time = now;
//         }
//     }
// }



//  RECIEVER AN TRANSMITTE TRY 1- Working


#include "NewDirectory/NewClass.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>   

extern const AP_HAL::HAL& hal;

#define LED_PIN 107

void NewClass::update() {
    static uint8_t led_state = 0;
    static uint32_t last_toggle_time = 0;
    RC_Channel* ch5 = RC_Channels::rc_channel(4);
    if (!ch5) {
        gcs().send_text(MAV_SEVERITY_ERROR, "RC channel 5 not available");
        return;
    }
    const uint16_t pwm = ch5->get_radio_in();
    char msg[60];
    hal.util->snprintf(msg, sizeof(msg), "RC5 PWM: %u", pwm);
    gcs().send_text(MAV_SEVERITY_INFO, "%s", msg);

    if (pwm >= 950 && pwm <= 1050) {
        hal.gpio->write(LED_PIN, 0);
        gcs().send_text(MAV_SEVERITY_INFO, "LED OFF");
    } 
    else if (pwm >= 1450 && pwm <= 1550) {
        uint32_t now = AP_HAL::millis();
        if (now - last_toggle_time > 5) {
            led_state = !led_state;
            hal.gpio->write(LED_PIN, led_state);
            gcs().send_text(MAV_SEVERITY_INFO, led_state ? "LED TOGGLED ON" : "⚫ LED TOGGLED OFF");
            last_toggle_time = now;
        }
    } 
    else if (pwm >= 1950 && pwm <= 2050) {
        hal.gpio->write(LED_PIN, 1);
        gcs().send_text(MAV_SEVERITY_INFO, "LED ON");
    }
}
