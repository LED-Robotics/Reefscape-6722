/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ColorSensor.h"

#include <hal/HAL.h>

using namespace frc;

#define CMD 0x80
#define MULTI_BYTE_BIT 0x20
#define PON 0b00000001
#define AEN 0b00000010
#define PEN 0b00000100
double integrationTime = 10;

ColorSensor::ColorSensor(Port port, int deviceAddress) : I2C(port, deviceAddress) {
    Write(CMD | 0x00, PON | AEN | PEN);
    Write(CMD | 0x01, (int) (256 - integrationTime / 2.38));
    Write(CMD | 0x0E, 0b1111);
}

int ColorSensor::getRed() {
    uint8_t readData[2];
    int red;
    Read(CMD | MULTI_BYTE_BIT | 0x16, 2, readData);
    red = (readData[1] << 8) + readData[0];
    return red;
}

int ColorSensor::getGreen() {
    uint8_t readData[2];
    int green;
    Read(CMD | MULTI_BYTE_BIT | 0x18, 2, readData);
    green = (readData[1] << 8) + readData[0];
    return green;
}

int ColorSensor::getBlue() {
    uint8_t readData[2];
    int blue;
    Read(CMD | MULTI_BYTE_BIT | 0x1A, 2, readData);
    blue = (readData[1] << 8) + readData[0];
    return blue;
}

int ColorSensor::getWhite() {
    int white;
    white = (ColorSensor::getRed() + ColorSensor::getGreen() + ColorSensor::getBlue()) / 3;
    return white;
}

int ColorSensor::getClear() {
    uint8_t readData[2];
    int clear;
    Read(CMD | MULTI_BYTE_BIT | 0x14, 2, readData);
    clear = (readData[1] << 8) + readData[0];
    return clear;
}

int ColorSensor::getProximity() {
    uint8_t readData[2];
    int proximity;
    Read(CMD | MULTI_BYTE_BIT | 0x1C, 2, readData);
    proximity = (readData[1] << 8) + readData[0];
    return proximity;
}

void ColorSensor::setGain(int gain) {
    switch(gain) {
        case 1:
            Write(CMD | 0x0F, 0b00100000);
            break;
        case 4:
            Write(CMD | 0x0F, 0b00100001);
            break;
        case 16:
            Write(CMD | 0x0F, 0b00100010);
            break;
        case 60:
            Write(CMD | 0x0F, 0b00100011);
            break;
    }
}