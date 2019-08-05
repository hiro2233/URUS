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

#pragma once

#include "AP_Baro_Backend.h"

class AP_Baro_URUS : public AP_Baro_Backend
{
public:
    AP_Baro_URUS(AP_Baro &baro);
    void update(void);
    static void init(void);

private:
    uint8_t _instance;
};

