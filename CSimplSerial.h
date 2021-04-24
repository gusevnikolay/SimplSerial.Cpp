#ifndef SIMPLSERIAL_CSIMPLSERIAL_H
#define SIMPLSERIAL_CSIMPLSERIAL_H

#include <cstdint>
#include <cstdlib>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include "SerialPort.h"

namespace Serial{

    enum CResponseStateEnum {
        ok,
        errorTimeout,
        errorFormat,
        errorCrc,
        errorPacketType,
        errorNotRequested,
        errorPortError,
        errorNetworkError
    };

    class CSSGuid {
    public:
        std::vector<uint8_t> Value;
        CSSGuid(){}
        CSSGuid(const std::vector<uint8_t> &guid){
            Value.insert(Value.begin(), guid.begin(), guid.end());
        }
        CSSGuid(const std::string &guid){
            std::istringstream ss(guid);
            std::string part;
            while(std::getline(ss, part, '-')) {
                Value.push_back(static_cast<uint8_t >(std::stoi(part.c_str(), nullptr, 16)));
            }
        }
        std::string GetString(){
            std::stringstream ss;
            ss << std::setw(2) << std::setfill('0');
            bool first = true;
            for (const auto &c : Value) {
                if(!first) ss << "-";
                ss << std::hex << std::setw(2) << static_cast<int>(c);
                first = false;
            }
            return ss.str();
        }
    };

    class CSSRequest {
    public :
        uint16_t Address;
        uint8_t Command;
        std::vector<uint8_t> Data;

        CSSRequest() {}

        CSSRequest(uint16_t address, uint16_t command) {
            Command = command;
            Address = address;
        }

        CSSRequest(const uint16_t &address, const int &command, const std::vector<uint8_t> &data) {
            Command = command;
            Address = address;
            Data = data;
        }
    };

    class CSSResponse {
    public:
        uint8_t Result;
        CResponseStateEnum ResponseState;
        uint16_t FromAddress;
        std::vector<uint8_t> Data;

        CSSResponse() {}
    };

    class CDeviceInfo {
    public:
        std::string DeviceName;
        std::string DeviceDate;
        CSSGuid DeviceGuid;
    };

    class CSimplSerialBus {
    public:
        uint16_t DefaultTimeout = 1000;
        std::vector<CDeviceInfo> SearchDevices();

        CSimplSerialBus(const std::string& port, const int& baud) {
            _port = port;
            _baudRate = baud;
            _bus = new SerialPort({port,""});
        }
        CSimplSerialBus(const SerialDevice& serialDevice, const int& baud) {
            _port = serialDevice.name;
            _baudRate = baud;
            _bus = new SerialPort(serialDevice);
        }

        std::vector<CSSGuid> FindDevices(const uint16_t & searchTime);
        CDeviceInfo RequestDeviceInfo(const uint16_t& address);
        void RequestSetAddress(CSSGuid &guid, const uint16_t &address);
        CSSResponse Read(const uint16_t &timeout);
        CSSResponse Request(const CSSRequest &requestPacket, const uint16_t &timeout);
        CSSResponse Request(const CSSRequest &requestPacket);
        CSSResponse Request(const CSSRequest &requestPacket, const int &retries);
        CSSResponse Request(const uint16_t &address, const uint8_t &command, const std::vector<uint8_t> &data);
        void Connect();

    private :
        SerialPort * _bus;
        std::string _port;
        int _baudRate;

        CSSResponse Read();
        void Send(const CSSRequest& request);
        std::vector<CSSGuid> FindDevices(const uint32_t &seed, const uint16_t &timeout);
    };

}
#endif //SIMPLSERIAL_CSIMPLSERIAL_H
