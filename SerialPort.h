//
// Created by bess on 03.03.2020.
//

#ifndef SIMPLSERIAL_SERIALPORT_H
#define SIMPLSERIAL_SERIALPORT_H

#include <string>
#include <termios.h>

namespace Serial{
    struct SerialDevice{
        std::string name = "/dev/";
        std::string driver;
    };

    //Get serial device vector
    std::vector<SerialDevice> GetSerialPort();

    class SerialPort{
    public:
        SerialPort(const SerialDevice &device){
            _device = device;
        }

        ~SerialPort();

        void Open(const int &baudRate);
        int  Read(unsigned char *buffer, const size_t &offset, const size_t &count);
        void Write(const unsigned char *buffer, const size_t &size);
    private:
        int _serialDeviceDescriptor = -1;
        struct termios _deviceConfig;

        SerialDevice _device;
    };
}

std::ostream& operator<<(std::ostream& stream, const Serial::SerialDevice& dev);

#endif //SIMPLSERIAL_SERIALPORT_H
