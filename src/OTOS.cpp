#include "main.h"
#include "pros/rtos.hpp"
#include "OTOS.hpp"

void OTOS::sendCommand(char command, void *data, int length)
{
    // command protocol: [0x01, length, commandCode, data, crc16, 0x00]
    char outBuf[3 + length + 2 + 1] = {0x01, (char)(length + 4), command};
    std::memcpy(outBuf + 3, data, length);

    uint16_t crc = command;

    for (int i = 0; i < length; i++)
    {
        crc += ((char *)data)[i];
    }
    outBuf[3 + length] = crc & 0xFF;
    outBuf[3 + length + 1] = (crc >> 8) & 0xFF;

    outBuf[3 + length + 2] = 0x00; // null terminator

    this->serial->write((uint8_t *)&outBuf, 3 + length + 2 + 1);
    // this->serial->flush();

    // for(int i = 0; i < 3 + length + 3; i++){
    //     printf("%x ", outBuf[i]);
    // }
    // printf("\n");
}

OTOS::OTOS(int port) : port(port)
{

}

OTOS::~OTOS()
{
    delete this->serial;
}
void OTOS::calibrate()
{
    this->serial = new pros::Serial(port, 921600);
    pros::delay(20);
    this->sendCommand('C', nullptr, 0);
}

void OTOS::resetTracking()
{
    this->sendCommand('R', nullptr, 0);
}

Pose OTOS::getPose()
{
    this->sendCommand('P', nullptr, 0);
    char buffer[50];
    int recv = this->serial->read((uint8_t *)buffer, 50);
    // for(int i = 0; i < 50; i++){
    //     printf("%x ", buffer[i]);
    // }
    // printf("\n\n\n");

    // check crc
    uint16_t crc = 0;
    for (int i = 0; i < 13; i++)
    {
        crc += buffer[i];
    }
    if (crc != (buffer[13] + (buffer[14] << 8)))
    {
        //std::cout << "recv error: " << recv << std::endl;
        // for (int i = 0; i < recv; ++i)
        // {
        //     std::cout << "0x" << std::hex << (int)buffer[i] << " ";
        // }
        // std::cout << std::endl;
        return lastRead;
    }

    Pose pose;
    std::memcpy(&pose, &buffer[1], 12);
    isCalibrated = true;
    lastRead = pose;
    return pose;
}

int OTOS::getData(char *buffer, int size)
{
    return this->serial->read((uint8_t *)buffer, size);
}

void OTOS::setPose(Pose pose)
{
    this->sendCommand('S', &pose, 12);
}

void OTOS::setOffset(Pose pose)
{
    this->sendCommand('O', &pose, 12);
}

void OTOS::setLinearScaler(float scaler)
{
    this->sendCommand('L', &scaler, 4);
}
void OTOS::setAngularScaler(float scaler)
{
    this->sendCommand('A', &scaler, 4);
}