
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <Filter/Filter.h>
#include <Filter/ModeFilter.h>
#include <Filter/AverageFilter.h>
#include <Filter/LowPassFilter2p.h>
#include <AP_Param/AP_Param.h>

#include "joypad_remote.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

volatile bool JoypadRemote::_sending = false;
volatile uint8_t JoypadRemote::inByte = 0;

JoypadRemote::data_controller_t JoypadRemote::controller_data_buffer1;
JoypadRemote::data_controller_t JoypadRemote::controller_data_buffer2;

JoypadRemote joypadremote;

static LowPassFilter2pInt *low_pass_filter[SENSORS_COUNT];
int16_t filter_tmp;

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(JoypadRemote, &joypadremote, func, _interval_ticks, _max_time_micros)

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */
const AP_Scheduler::Task JoypadRemote::scheduler_tasks[] = {
    SCHED_TASK(update_sensor,   30,   6000),
    SCHED_TASK(live,            10,   5000),
    SCHED_TASK(beep,            20,   6000),
};

JoypadRemote::JoypadRemote():
    sw_pins(false),
    sw_pins_pushing(false),
    _cal_ch_mask(0),
    _tone_speed(10),
    _tone_on_off(false)
{
}

void JoypadRemote::live()
{
#if CONFIG_JOYPAD != CFG_BCH_EXTRA
#ifndef ENABLED_EXT_MUX

    if (_cnt_sw_filter < 15) {
        _cnt_sw_controller = 0;
        for (uint8_t i = 0; i < SENSORS_COUNT; i++) {
            if (filtered_value_buf[i] < 20) {
                _cnt_filter[i]++;
                if (_cnt_filter[i] > 10) {
                    _cnt_sw_controller++;
                }
            }

            if ((filtered_value_buf[i] > 20) && (filtered_value_buf[i] < 500)) {
                _cal_ch_mask |= (1 << i);
            }
        }
        _cnt_sw_filter++;
    }

    if ((1 & (_cal_ch_mask >> 2)) &&
        (1 & (_cal_ch_mask >> 1)) && (_cnt_ch_cal_min < 20)) {
        _tone_on_off = true;
        hal.gpio->write(13, 1);

        if (_cnt_ch_cal_min < 5) {
            for (uint8_t i = 0; i < SENSORS_COUNT; i++) {
                if (1 & (_cal_ch_mask >> i)) {
                    rc[i]->set_radio_min(filtered_value_buf[i]);
                }
            }
        }

        if (_cnt_ch_cal_min > 0) {
            _tone_speed = 10;
        }

        if (_cnt_ch_cal_min > 15) {
            _cnt_ch_cal_min = 20;
            _tone_speed = 2;
        }
        _cnt_ch_cal_min++;
    } else {
        if (filtered_value_buf[2] > (rc[2]->get_radio_min() + 200) && (_cnt_ch_cal_max < 30) && (_cnt_ch_cal_min > 15)) {
            _tone_speed = 7;
            if (_cnt_ch_cal_max > 28) {
                _cnt_ch_cal_max = 30;
                for (uint8_t i = 0; i < SENSORS_COUNT; i++) {
                    if (1 & (_cal_ch_mask >> i)) {
                        rc[i]->set_radio_max(filtered_value_buf[i]);
                        rc[i]->set_radio_trim((filtered_value_buf[i]) / 2);
                        rc[i]->save_eeprom();
                    }
                }
                hal.gpio->write(13, 0);
                hal.gpio->write(11, 0);
                _tone_on_off = false;
            }
            _cnt_ch_cal_max++;
        }
    }

#else
    if (_cnt_update_sensor == 0) {
        hal.gpio->write( 8, 0);
        hal.gpio->write( 9, 0);
        hal.gpio->write(10, 0);
    }

    if (_cnt_update_sensor == 1) {
        hal.gpio->write( 8, 1);
        hal.gpio->write( 9, 0);
        hal.gpio->write(10, 0);
    }

    if (_cnt_update_sensor == 2) {
        hal.gpio->write( 8, 1);
        hal.gpio->write( 9, 1);
        hal.gpio->write(10, 0);
    }

    _cnt_update_sensor++;
    _cnt_update_sensor = _cnt_update_sensor % 3;
#endif
#endif // CONFIG_JOYPAD
}

void JoypadRemote::setup(void)
{

    hal.scheduler->delay(200);

#ifndef ENABLED_EXT_MUX
    hal.gpio->pinMode(13, HAL_GPIO_OUTPUT);
    hal.gpio->write(13, 0);
    hal.gpio->pinMode(12, HAL_GPIO_OUTPUT);
    hal.gpio->write(12, 0);
    hal.gpio->pinMode(11, HAL_GPIO_OUTPUT);
    hal.gpio->write(11, 1);
#else
    hal.gpio->pinMode(8, HAL_GPIO_OUTPUT);
    hal.gpio->write(8, 0);
    hal.gpio->pinMode(9, HAL_GPIO_OUTPUT);
    hal.gpio->write(9, 0);
    hal.gpio->pinMode(10, HAL_GPIO_OUTPUT);
    hal.gpio->write(10, 0);
#endif

    _tone_cnt = -1;
    _tone_on_off = true;
    beep();
    hal.scheduler->delay(200);
    _tone_cnt = -1;
    beep();
    hal.scheduler->delay(200);
    _tone_on_off = false;

    hal.scheduler->delay(2000);

    _cal_ch_mask = 0;
    load_parameters();

    rc[0] = &g.rc_1;
    rc[1] = &g.rc_2;
    rc[2] = &g.rc_3;
    rc[3] = &g.rc_4;
    rc[4] = &g.rc_5;
    rc[5] = &g.rc_6;
    rc[6] = &g.rc_7;

    for (uint8_t i = 0; i < SENSORS_COUNT; i++) {
        state[i].pin = i;
#ifndef ENABLED_EXT_MUX
        _analogsensor[i] = new AnalogSensor(&state[i]);
#endif // ENABLED_EXT_MUX
        low_pass_filter[i] = new LowPassFilter2pInt(800, 20);
        rc[i]->set_range(300);
        //rc[i]->set_radio_max(1000);
        //rc[i]->set_radio_trim((800 - 1) / 2);
        //rc[i]->set_radio_min(1);
        //rc[i]->set_default_dead_zone(0);
        //rc[i]->save_eeprom();
    }

#ifdef ENABLED_EXT_MUX
        _analogsensor[0] = new AnalogSensor(&state[0]);
        _analogsensor[1] = new AnalogSensor(&state[1]);
#endif // ENABLED_EXT_MUX

    //rc[2]->set_range(1000);
    //rc[2]->set_radio_max(680);
    //rc[2]->set_radio_trim(340);
    //rc[2]->set_radio_min(115);
    //rc[2]->set_default_dead_zone(0);
    //rc[2]->set_and_save_trim();
    //rc[2]->save_radio_trim();
    //rc[2]->save_eeprom();

    //rc[0].load_eeprom();
    //rc[0].set_radio_max(3000);
    //rc[0].save_eeprom();

    for (int i = PIN_FIRST; i < PIN_LAST; i++){
        hal.gpio->pinMode(i, HAL_GPIO_INPUT);
        hal.gpio->write(i, 1);
    }

    controller_data_buffer1 = get_empty_data_controller();
    controller_data_buffer2 = get_empty_data_controller();
    controller_data1 = get_empty_data_controller();
    controller_data2 = get_empty_data_controller();

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&JoypadRemote::send_data, void));
    // initialise the scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
    nowmicros = AP_HAL::micros();
}

void JoypadRemote::loop(void)
{
    if ((AP_HAL::micros() - nowmicros) > 19900) {
        nowmicros = AP_HAL::micros();
        scheduler.tick();
        scheduler.run(20000);
    }
}

void JoypadRemote::update_sensor(void)
{
#ifndef ENABLED_EXT_MUX
    for (uint8_t i = 0; i < SENSORS_COUNT; i++) {
        _analogsensor[i]->update();
    }
#else
    _analogsensor[0]->update();
    _analogsensor[1]->update();

#endif

    if (!_sending) {
        hal.scheduler->suspend_timer_procs();
        int16_t new_value[SENSORS_COUNT];

        for (uint8_t i = 0; i < SENSORS_COUNT; i++) {
            new_value[i] =  state[i].distance_cm;
            filtered_value_buf[i] = new_value[i];
            rc[i]->set_pwm(new_value[i]);

            filtered_value[i] = (int16_t)low_pass_filter[i]->apply(rc[i]->get_control_in());

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

#ifndef DISABLED_SWITCHING
    if (!hal.gpio->read(25) && !sw_pins_pushing) {
        sw_pins = !sw_pins;
        sw_pins_pushing = true;
        hal.gpio->toggle(13);
    } else if (hal.gpio->read(25) && sw_pins_pushing) {
        sw_pins_pushing = false;
    }

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
    if ((_cnt_sw_controller < 1 ) && (_cnt_sw_filter >= 15)) {
        controller_data1.left_stick_x = filtered_value[LS_X];
        controller_data1.left_stick_y = filtered_value[LS_Y];
        controller_data1.right_stick_x = filtered_value[RS_X];
        controller_data1.right_stick_y = filtered_value[RS_Y];
        controller_data1.stick3_x = filtered_value[ST_X];
        controller_data1.stick3_y = filtered_value[ST_Y];
    } else {
        controller_data2.left_stick_x = filtered_value[LS_X];
        controller_data2.left_stick_y = filtered_value[LS_Y];
        controller_data2.right_stick_x = filtered_value[RS_X];
        controller_data2.right_stick_y = filtered_value[RS_Y];
        controller_data2.stick3_x = filtered_value[ST_X];
        controller_data2.stick3_y = filtered_value[ST_Y];
    }
#endif

    set_controller_data(controller_data1, 0);

#if CONTROLLER_DATA_CNT > 1
    for (int i = 0; i < BUTTON_ARRAY_LENGTH; i++) {
        controller_data2.button_array[i] = 0;
    }

    if (!hal.gpio->read(27)) {
        controller_data2.button_array[0] = 63;
    } else {
        controller_data2.button_array[0] = 0;
    }

    controller_data2.right_stick_x = filtered_value[LS_X1];
    controller_data2.right_stick_y = filtered_value[LS_Y1];
    controller_data2.stick3_x = filtered_value[RS_X1];
#endif

    set_controller_data(controller_data2, 1);
}

void JoypadRemote::send_data(void)
{
    if (_sending) {
        return;
    }
    _sending = true;

    if (hal.uartA->available() > 0) {
        inByte = hal.uartA->read();
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

void JoypadRemote::beep(void) {

    if (_tone_on_off) {
        if (_tone_cnt > _tone_speed) {
            for (uint8_t i = 0; i < 25; i++) {
                hal.gpio->write(12, 1);
                hal.scheduler->delay_microseconds(105);
                hal.gpio->write(12, 0);
                hal.scheduler->delay_microseconds(115);
            }
            _tone_cnt = 0;
        }
        _tone_cnt++;
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
