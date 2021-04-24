#include <iostream>
#include <experimental/filesystem>
#include <vector>
#include <fstream>
#include <string>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/signal.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <sstream>
#include <sys/ioctl.h>

#include "SerialPort.h"

using namespace std;

#ifndef __has_include
static_assert(false, "__has_include not supported");
#else
#  if __has_include(<filesystem>)
#    include <filesystem>
     namespace fs = std::filesystem;
#  elif __has_include(<experimental/filesystem>)
#    include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#  elif __has_include(<boost/filesystem.hpp>)
#    include <boost/filesystem.hpp>
     namespace fs = boost::filesystem;
#  endif
#endif

Serial::SerialPort::~SerialPort(){
    close(_serialDeviceDescriptor);
    _serialDeviceDescriptor = -1;
}

void Serial::SerialPort::Open(const int &baudRate){
    speed_t baud;
    switch (baudRate)
    {
        case 1200:    baud = B1200	  ; break;
        case 1800:    baud = B1800	  ; break;
        case 2400:    baud = B2400	  ; break;
        case 4800:    baud = B4800	  ; break;
        case 9600:    baud = B9600    ; break;
        case 19200:   baud = B19200	  ; break;
        case 38400:   baud = B38400	  ; break;
        case 57600:   baud = B57600   ; break;
        case 115200:  baud = B115200  ; break;
        case 230400:  baud = B230400  ; break;
        case 460800:  baud = B460800  ; break;
        case 500000:  baud = B500000  ; break;
        case 576000:  baud = B576000  ; break;
        case 921600:  baud = B921600  ; break;
        case 1000000: baud = B1000000 ; break;
        case 1152000: baud = B1152000 ; break;
        case 1500000: baud = B1500000 ; break;
        case 2000000: baud = B2000000 ; break;
        case 2500000: baud = B2500000 ; break;
        case 3000000: baud = B3000000 ; break;
        case 3500000: baud = B3500000 ; break;
        case 4000000: baud = B4000000 ; break;
        default:
            stringstream stream;
            stream << "SerialPort.Open(const int &baudRate): Baud rate have wrong value: '"<< baudRate <<"'";
            throw runtime_error(stream.str());
    }
    _serialDeviceDescriptor  = open(_device.name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
    if (_serialDeviceDescriptor < 0)
    {
        stringstream stream;
        stream << "SerialPort::Open(const int &baudRate): Serial device open failed: device '"<<_device.name<<"'";
        throw runtime_error(stream.str());
    }
    if(tcgetattr(_serialDeviceDescriptor, &_deviceConfig) < 0)
    {
        stringstream stream;
        stream << "SerialPort::Open(const int &baudRate): Can't load config from TTY device '" << _device.name << "'";
        throw runtime_error(stream.str());
    }

    if(cfsetispeed(&_deviceConfig, baud) < 0 || cfsetospeed(&_deviceConfig, baud) < 0)
    {
        stringstream stream;
        stream << "SerialPort::Open(const int &baudRate): Baud rate not set for device '" << _device.name << "'";
        throw runtime_error(stream.str());
    }
    _deviceConfig.c_cflag |= (CLOCAL | CREAD | CS8);
    _deviceConfig.c_cflag &= ~PARENB;
    _deviceConfig.c_cflag &= ~CSTOPB;
    _deviceConfig.c_cflag &= ~CSIZE;
    _deviceConfig.c_lflag = 0;
    _deviceConfig.c_iflag &= ~(IXON | IXOFF | IXANY);
    _deviceConfig.c_iflag &= ~OPOST;
    cfmakeraw(&_deviceConfig);
    if(tcsetattr(_serialDeviceDescriptor, TCSAFLUSH, &_deviceConfig) < 0)
    {
        stringstream stream;
        stream << "SerialPort::Open(const int &baudRate): Can not config device '" << _device.name << "'";
        throw runtime_error(stream.str());
    }
}

int Serial::SerialPort::Read(unsigned char *buffer, const size_t &offset, const size_t &count)
{
    size_t byteCounter = count;
    unsigned char *receiver = &buffer[offset];
    unsigned char tryCounter = 0;
    while (0 != byteCounter){
        auto response = read(_serialDeviceDescriptor, receiver, byteCounter);
        if(response < 0){
            if (EINTR==errno)
            {
                continue;
            }
            if (EAGAIN ==errno)
            {
                if(tryCounter++<20){
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                return count-byteCounter;
            }
        }else if(response == 0){
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            fd_set rfds;
            struct timeval tv;
            FD_ZERO(&rfds);
            FD_SET(_serialDeviceDescriptor, &rfds);
            tv.tv_sec = 0;
            tv.tv_usec = 100000;
            int ready = select(_serialDeviceDescriptor + 1, &rfds, NULL, NULL, NULL);
            if(ready && FD_ISSET(_serialDeviceDescriptor, &rfds)) {
                size_t len = 0;
                ioctl(_serialDeviceDescriptor, FIONREAD, &len);
                if (len == 0) {
                    throw runtime_error("SerialPort::Read(*buffer, offset, count): error. Port closed");
                }
            }
        }else{
            receiver += response;
            byteCounter -= response;
        }
    }

    return count-byteCounter;
}

void Serial::SerialPort::Write(const unsigned char *buffer, const size_t &size)
{
    if (_serialDeviceDescriptor < 0) throw runtime_error("SerialPort.Write(const unsigned char *buffer, const size_t &size): Serial device closed");
    if (write(_serialDeviceDescriptor, buffer, size) < 0) throw runtime_error("SerialPort.Write(const unsigned char *buffer, const size_t &size): Error writing data");
    tcdrain(_serialDeviceDescriptor);
}

vector<Serial::SerialDevice> Serial::GetSerialPort()
{
    vector<Serial::SerialDevice> v;
    for(auto& dir: fs::directory_iterator("/sys/class/tty"))
    {
        if(fs::exists(dir.path().string() + "/device/port_number")){
            Serial::SerialDevice device;

            ifstream stream(dir.path().string() + "/device/uevent");
            getline(stream, device.driver,'=');
            getline(stream, device.driver);

            auto pathIterator = dir.path().end();
            pathIterator--;
            device.name += *pathIterator;
            v.push_back(device);
        }
    }
    return v;
}

ostream& operator<<(ostream& stream, const Serial::SerialDevice& dev)
{
    stream << "{\""<< dev.name <<"\":\""<<dev.driver<<"\"}";
    return stream;
}