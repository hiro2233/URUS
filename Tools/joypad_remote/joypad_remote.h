
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Scheduler/AP_Scheduler.h>

#include "analog_sensor.h"

#define BUTTON_ARRAY_LENGTH 4
#define SENSORS_COUNT 6
#define CONTROLLER_DATA_CNT 1
#define PIN_FIRST   22
#define PIN_LAST    44

class JoypadRemote {
public:

    JoypadRemote();

    void setup();
    void loop();

private:

    enum STICKS {
        LS_X = 0,
        LS_Y,
        RS_X,
        RS_Y,
        ST_X,
        ST_Y,
    };

    typedef struct __data_controller
    {
        uint8_t button_array[BUTTON_ARRAY_LENGTH];
        uint8_t dpad_left_on : 1;
        uint8_t dpad_up_on : 1;
        uint8_t dpad_right_on : 1;
        uint8_t dpad_down_on : 1;
        uint8_t dummy : 4;

        int16_t left_stick_x;
        int16_t left_stick_y;
        int16_t right_stick_x;
        int16_t right_stick_y;
        int16_t stick3_x;
        int16_t stick3_y;

    } data_controller_t;

    AP_Scheduler scheduler;
    AnalogSensor::state_t state[SENSORS_COUNT];
    AnalogSensor *_analogsensor[SENSORS_COUNT];

    data_controller_t controller_data_buffer1;
    data_controller_t controller_data_buffer2;
    data_controller_t controller_data1;

#if CONTROLLER_DATA_CNT > 1
    data_controller_t controller_data2;
#endif

    uint32_t ins_counter;
    static const AP_Scheduler::Task scheduler_tasks[] PROGMEM;

    uint32_t nowmicros;
    uint8_t pin;
/*
    int16_t filtered_value1, filtered_value2;
    int16_t filtered_value3, filtered_value4;
    int16_t filtered_value5, filtered_value6;
*/
    int16_t filtered_value[SENSORS_COUNT];
    volatile static bool _sending;

    bool sw_pins;
    bool sw_pins_pushing;

    void update_sensor(void);
    void send_data(void);
    void set_data(void);
    void live(void);
    data_controller_t get_empty_data_controller(void);
    void set_controller_data(data_controller_t controller_data_set, uint8_t id);
};
