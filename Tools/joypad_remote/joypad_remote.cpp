
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <Filter/Filter.h>
#include <Filter/ModeFilter.h>
#include <Filter/AverageFilter.h>
#include <Filter/LowPassFilter2p.h>

#include "joypad_remote.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

volatile bool JoypadRemote::_sending = false;
JoypadRemote joypadremote;

static LowPassFilter2pLong *low_pass_filter[SENSORS_COUNT];

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(JoypadRemote, &joypadremote, func, _interval_ticks, _max_time_micros)

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task JoypadRemote::scheduler_tasks[] = {
    SCHED_TASK(update_sensor,   50,   2000),
    SCHED_TASK(live,             1,   1000),
};

JoypadRemote::JoypadRemote():
    sw_pins(false),
    sw_pins_pushing(false)
{
}

void JoypadRemote::live()
{
    hal.gpio->toggle(13);
}

void JoypadRemote::setup(void)
{
    hal.gpio->pinMode(13, HAL_GPIO_OUTPUT);
    hal.gpio->write(13, 0);
/*
    for (uint8_t i = 0; i < 20; i++) {
        hal.gpio->toggle(13);
        hal.scheduler->delay(50);
    }
*/
    for (uint8_t i = 0; i < SENSORS_COUNT; i++) {
        state[i].pin = i;
        _analogsensor[i] = new AnalogSensor(&state[i]);
        low_pass_filter[i] = new LowPassFilter2pLong(600, 10);
    }

    for (int i = PIN_FIRST; i < PIN_LAST; i++){
        hal.gpio->pinMode(i, HAL_GPIO_INPUT);
        hal.gpio->write(i, 1);
    }

    controller_data_buffer1 = get_empty_data_controller();
    controller_data_buffer2 = get_empty_data_controller();
    controller_data1 = get_empty_data_controller();
#if CONTROLLER_DATA_CNT > 1
    controller_data2 = get_empty_data_controller();
#endif

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&JoypadRemote::send_data, void));
    // initialise the scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
    nowmicros = AP_HAL::micros();
}

void JoypadRemote::loop(void)
{
    if ((AP_HAL::micros() - nowmicros) > 19900) {
        scheduler.tick();
        scheduler.run(20000);
        nowmicros = AP_HAL::micros();
    }
}

void JoypadRemote::update_sensor(void)
{

    for (uint8_t i = 0; i < SENSORS_COUNT; i++) {
        _analogsensor[i]->update();
    }

    for (int i = PIN_FIRST; i < PIN_LAST; i++){
        hal.gpio->pinMode(i, HAL_GPIO_INPUT);
        hal.gpio->write(i, 1);
    }

    if (!_sending) {
        hal.scheduler->suspend_timer_procs();
        const uint16_t *new_value[SENSORS_COUNT];
        for (uint8_t i = 0; i < SENSORS_COUNT; i++) {
            new_value[i] =   new uint16_t(state[i].distance_cm);
            filtered_value[i] = (int16_t)low_pass_filter[i]->apply((int16_t)*new_value[i]);
        }
        set_data();
        hal.scheduler->resume_timer_procs();
    }
}

void JoypadRemote::set_data(void)
{

    for (int i = 0; i < BUTTON_ARRAY_LENGTH; i++) {
        controller_data1.button_array[i] = 0;
    }

    for (int i = PIN_FIRST; i < PIN_LAST; i++){
        controller_data1.button_array[(i - PIN_FIRST) / 8] |= (!hal.gpio->read(i)) << ((i - PIN_FIRST) % 8);
    }

    if (!hal.gpio->read(25) && !sw_pins_pushing) {
        sw_pins = !sw_pins;
        sw_pins_pushing = true;
        hal.gpio->toggle(13);
    } else if (hal.gpio->read(25) && sw_pins_pushing) {
        sw_pins_pushing = false;
    }

#ifndef DISABLED_SWITCHING
    if (sw_pins) {
        if (!hal.gpio->read(26)) {
            controller_data1.button_array[(26 - PIN_FIRST) / 8] &= ~(1 << ((26 - PIN_FIRST) % 8));
            controller_data1.button_array[(28 - PIN_FIRST) / 8] |= !hal.gpio->read(26) << ((28 - PIN_FIRST) % 8);
        }

        if (!hal.gpio->read(27)) {
            controller_data1.button_array[(27 - PIN_FIRST) / 8] &= ~(1 << ((27 - PIN_FIRST) % 8));
            controller_data1.button_array[(29 - PIN_FIRST) / 8] |= !hal.gpio->read(27) << ((29 - PIN_FIRST) % 8);
        }
    }
#endif

#ifndef DISABLED_ANALOG
    controller_data1.left_stick_x = filtered_value[LS_X];
    controller_data1.left_stick_y = filtered_value[LS_Y];
    controller_data1.right_stick_x = filtered_value[RS_X];
    controller_data1.right_stick_y = filtered_value[RS_Y];
    controller_data1.stick3_x = filtered_value[ST_X];
    controller_data1.stick3_y = filtered_value[ST_Y];
#endif

    set_controller_data(controller_data1, 0);

#if CONTROLLER_DATA_CNT > 1
    for (int i = 0; i < BUTTON_ARRAY_LENGTH; i++) {
        controller_data2.button_array[i] = 0;
    }

    controller_data2.right_stick_x = filtered_value[LS_X];
    controller_data2.right_stick_y = filtered_value[LS_Y];

    set_controller_data(controller_data2, 1);
#endif

}

void JoypadRemote::send_data(void)
{
    if (_sending) {
        return;
    }
    _sending = true;

    if (hal.uartA->available() > 0) {
        uint8_t inByte = hal.uartA->read();
        if (inByte < sizeof(data_controller_t)) {
            hal.uartA->write(((uint8_t*)&controller_data_buffer1)[inByte]);
        } else {
            inByte = inByte - sizeof(data_controller_t);
            hal.uartA->write(((uint8_t*)&controller_data_buffer2)[inByte]);
        }
    }
    _sending = false;
}

JoypadRemote::data_controller_t JoypadRemote::get_empty_data_controller(void)
{
    data_controller_t controller_data_empty;
    // Make the buttons zero
    for (int i = 0; i < BUTTON_ARRAY_LENGTH; i++) {
        controller_data_empty.button_array[i] = 0;
    }

    controller_data_empty.dpad_left_on = 0;
    controller_data_empty.dpad_up_on = 0;
    controller_data_empty.dpad_right_on = 0;
    controller_data_empty.dpad_down_on = 0;
    controller_data_empty.dummy = 0;

    // Center the sticks
    controller_data_empty.left_stick_x = 1000;
    controller_data_empty.left_stick_y = 1000;
    controller_data_empty.right_stick_x = 1000;
    controller_data_empty.right_stick_y = 1000;
    controller_data_empty.stick3_x = 1000;
    controller_data_empty.stick3_y = 1000;

    return controller_data_empty;
}

void JoypadRemote::set_controller_data(data_controller_t controller_data_set, uint8_t id)
{
    if (!_sending) {
        switch (id) {
        case 0:
            memcpy(&controller_data_buffer1, &controller_data_set, sizeof(data_controller_t));
            break;
        case 1:
            memcpy(&controller_data_buffer2, &controller_data_set, sizeof(data_controller_t));
            break;
        default:
            break;
        }
    }
}

/*
  compatibility with old pde style build
 */
void setup(void);
void loop(void);

void setup(void)
{
    joypadremote.setup();
}

void loop(void)
{
    joypadremote.loop();
}

AP_HAL_MAIN();
