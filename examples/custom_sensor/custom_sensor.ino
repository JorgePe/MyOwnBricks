/*
 * MyOwnBricks is a library for the emulation of PoweredUp sensors on microcontrollers
 * Copyright (C) 2021-2023 Ysard - <ysard@users.noreply.github.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <CustomSensor.h>

#define SWITCH_GPIO 23

int8_t sensorSwitch;
bool   connection_status;

CustomSensor myOwnSwitch(& sensorSwitch);

void setup() {
    pinMode(SWITCH_GPIO, INPUT_PULLUP);
#if (defined(INFO) || defined(DEBUG))
    Serial.begin(115200); // USB CDC
   
    while (!Serial) {
        // Wait for serial port to connect.
    }
#endif

    connection_status = false;
}


void loop() {
    // Get data from GPIO pin
    if( digitalRead(SWITCH_GPIO) != HIGH) {
      sensorSwitch = 100;
    }
    else {
      sensorSwitch = 0;
    }

    myOwnSwitch.process(); 

    if (myOwnSwitch.isConnected()) {
        // Already connected ?
        if (!connection_status) {
            INFO_PRINTLN(F("Connected !"));
            connection_status = true;
        }
    } else {
        INFO_PRINTLN(F("Not Connected !"));
        connection_status = false;
    }
}
