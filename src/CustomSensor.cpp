/*
 * A library for the emulation of PoweredUp sensors on microcontrollers
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
#include "CustomSensor.h"

CustomSensor::CustomSensor(){
    m_sensorSwitch = nullptr;  
} 

CustomSensor::CustomSensor(int8_t *pSensorSwitch){
    m_sensorSwitch = pSensorSwitch;
}

void CustomSensor::setSensorSwitch(int8_t *pData){
    m_sensorSwitch = pData;
}



/**
 * @brief Send initialization sequences for the current sensor.
 * @see https://github.com/pybricks/technical-info/blob/master/uart-protocol.md
 */
void CustomSensor::commSendInitSequence(){

    // Initialize uart
#if defined(ESP32)       
    SerialTTL.begin(2400, SERIAL_8N1, RXD1, TXD1);
#else
    SerialTTL.begin(2400);
#endif    

    SerialTTL.write("\x40\x24\x9B", 3);                              // CMD_TYPE, Type ID: 0x24 = 36 OK   = N/A

    // CustomSensor sensor has just 1 mode (mode 0, MYOWNSWITCH) 
    SerialTTL.write("\x49\x00\x00\xB6", 4);                          // CMD_MODES: 1 mode, 1 view, 0 Ext.Modes/Ext.Views
    
    SerialTTL.write("\x52\x00\xC2\x01\x00\x6E", 6);                  // CMD_SPEED: 115200
    
    SerialTTL.write("\x5F\x00\x00\x00\x10\x00\x00\x00\x10\xA0", 10); // CMD_VERSION: fw-version: 1.0.0.0, hw-version: 1.0.0.0
    SerialTTL.flush();

    // describe Mode 0
    SerialTTL.write("\xA0\x00\x4D\x59\x4F\x57\x4E\x53\x57\x49\x54\x43\x48\x00\x00\x00\x00\x00\x0F", 19); // Name: "MYOWNSWITCH"
      
    // next 5 defs copied from TiltSensor mode 0 - need to adapt
    SerialTTL.write("\x98\x01\x00\x00\x34\xC2\x00\x00\x34\x42\xE6", 11);                    // Range: -45.0 to 45.0
    SerialTTL.write("\x98\x02\x00\x00\xC8\xC2\x00\x00\xC8\x42\xE5", 11);                    // PCT Range: -100.0% to 100.0%
    SerialTTL.write("\x98\x03\x00\x00\x34\xC2\x00\x00\x34\x42\xE4", 11);                    // Si Range: -45.0 to 45.0
    SerialTTL.write("\x90\x04\x44\x45\x47\x00\x2D", 7);                                     // Si Symbol: DEG
    SerialTTL.write("\x88\x05\x10\x00\x62", 5);                                             // input_flags: Absolute, output_flags: None
    
    // Mode 0 just sends 1 byte (DATA8 for 8-bit signed integer = int8)
    SerialTTL.write("\x90\x80\x01\x00\x03\x00\xED", 7);                                     // Format: DATA8, 3 figures, 0 decimals
    SerialTTL.flush();

    // end of CustomSensor description

    SerialTTL.write("\x04", 1); // ACK
    SerialTTL.flush();
    delay(5);
}


/**
 * @brief Handle the protocol queries & responses from/to the hub.
 *      Queries can be read/write according to the requested mode.
 * @warning In the situation where the processing of the responses to the
 *      queries from the hub takes longer than 200ms, a disconnection
 *      will be performed here.
 */
void CustomSensor::handleModes(){
    if (SerialTTL.available() == 0)
        return;

    unsigned char header = SerialTTL.read();

    if (header == 0x02) {  // NACK
        m_lastAckTick = millis();

        // Send default mode: 0 (only mode in fact)
        this->sensorSwitchMode_0();
    } else if (header == 0x43) {
        // "Get value" commands (3 bytes message: header, mode, checksum)
        size_t ret = SerialTTL.readBytes(m_rxBuf, 2);
        if (ret < 2) {
            // check if all expected bytes are received without timeout
            DEBUG_PRINT("incomplete 0x43 message");
            return;
        }

        switch (m_rxBuf[0]) {        

            case CustomSensor::LEGO_DEVICE_MODE_PUP_CUSTOM_SENSOR__MYOWNSWITCH:
                this->sensorSwitchMode_0();                
                break;
                
            default:
                break;
        }
    }
}


/**
 * @brief Mode 0 response (read): Send 1 byte
 */


void CustomSensor::sensorSwitchMode_0(){
    m_txBuf[0] = 0xC0;                          // MESSAGE_CMD | LENGHT_1 | MODE_0 
    m_txBuf[1] = _(uint8_t)(*m_sensorSwitch);
    sendUARTBuffer(1);
}
