/*
 * MyOwnBricks is a library for the emulation of PoweredUp sensors on microcontrollers
 * Copyright (C) 2021-2023 Ysard - <ysard@users.noreply.github.com>
 *
 * Based on the original work of Ahmed Jouirou - <ahmed.jouirou@gmail.com>
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
#ifndef CUSTOMSENSOR_H
#define CUSTOMSENSOR_H

#include "BaseSensor.h"


/**
 * @brief Handle the LegoUART protocol and define modes of the
 * Custom sensor.
 *
 * @param m_sensorSwitch int8 (DATA8)

 */
class CustomSensor : public BaseSensor {

    enum {   
        LEGO_DEVICE_MODE_PUP_CUSTOM_SENSOR__MYOWNSWITCH = 0,  // read 1x int18_t                
    };

public:
    CustomSensor();
    CustomSensor(int8_t *pSensorSwitch);
    
    void setSensorSwitch(int8_t *pData);  
    void sensorSwitchMode_0();
    
    int8_t m_sensorOutput;
    
private:
    // Process queries from/to hub
    virtual void handleModes();
    virtual void commSendInitSequence();

    // UART protocol
    uint8_t m_currentExtMode = 0;
    
    int8_t *m_sensorSwitch;  
};

#endif
