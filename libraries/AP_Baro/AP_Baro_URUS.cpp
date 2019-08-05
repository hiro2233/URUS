/*
   URUS Synthetic Barometer driver.
   Copyright (c) 2017, 2019 Hiroshi Takey <htakey@gmail.com>

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "AP_Baro_URUS.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_URUS
#include <AP_HAL_URUS/AP_HAL_URUS.h>
#endif
#include <UR_Atmosphere/atmosphere_definitions.h>

extern const AP_HAL::HAL& hal;

using namespace ISA_MATH_CONST;

AP_Baro_URUS::AP_Baro_URUS(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
#if HAL_BARO_DEFAULT == HAL_BARO_URUS
    _instance = _frontend.register_sensor();
#endif // HAL_BARO_DEFAULT
    init();
}

void AP_Baro_URUS::init(void)
{
    hal.console->printf("init Baro URUS!\n");
}

// Read the sensor
void AP_Baro_URUS::update(void)
{
#if HAL_BARO_DEFAULT == HAL_BARO_URUS
        float rnd_press;
        float rnd_temp;

        rnd_press = rand() % 5 + 1;
        rnd_temp = rand() % 5 + 1;
        rnd_temp = rnd_temp / 10;
        _copy_to_frontend(0, 97660.0 + rnd_press, 28.0 + rnd_temp);
#endif // HAL_BARO_DEFAULT
}
