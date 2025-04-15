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

#include <Servo.h>
Servo myservo;
#define pinServo 29   // Rc Servo signal

#define pinGreen 26   // Green LED anode  - High to ON
#define pinYellow 27  // Yellow LED anode - High to ON
#define pinRed 28     // Red LED anode    - High to ON

#define SWITCH_GPIO 23

int8_t sensorSwitch;

bool   connection_status;

CustomSensor myOwnSwitch(& sensorSwitch);

void ledSignal(bool green, bool yellow, bool red){
    digitalWrite(pinGreen, (green == true) ? HIGH : LOW);
    digitalWrite(pinYellow, (yellow == true) ? HIGH : LOW);
    digitalWrite(pinRed, (red == true) ? HIGH : LOW);
}

void setup() {
    pinMode(SWITCH_GPIO, INPUT_PULLUP);

    // initialize the RGB LED pins as an output:
    pinMode(pinGreen, OUTPUT);
    pinMode(pinYellow, OUTPUT);
    pinMode(pinRed, OUTPUT);
    // turn all LEDs off
    ledSignal(LOW, LOW, LOW);

    // initialize RC servo - Tower Pro SG92R
    // with custom MIN and MAX timings
    myservo.attach(pinServo, 400, 2500);

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

    switch(myOwnSwitch.m_sensorOutput){
        case 0:
            ledSignal(false, false, false);   // all off
            break;
        case 1:
            ledSignal(true, false, false);    // green on
            break;
        case 2:
            ledSignal(false, true, false);    // yellow on
            break;
        case 3:
            ledSignal(false, false, true);    // red on
            break;
        case 4:
            ledSignal(true, true, true);  // all on
            break;
        case 5:
            myservo.write(0);      // servo to min
            break;
        case 6:
            myservo.write(90);     // servo to mid
            break;
        case 7:
            myservo.write(180);    // servo to max
            break;
    }
}