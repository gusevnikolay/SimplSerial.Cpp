#include "CSimplSerial.h"
#include <exception>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

std::mutex serialMutex;
class CRC16{
public:
    uint16_t Value = 0xFFFF;
    uint16_t Update(uint8_t a)
    {
        int i;
        Value ^= a;
        for (i = 0; i < 8; ++i)
        {
            if (Value & 1) Value = (Value >> 1) ^ 0xA001;
            else Value = (Value >> 1);
        }
        return Value;
    }

    uint16_t ComputeCrc(uint8_t receivedBuffer[], const int &offset, const int &count){
        for(int i=0;i<count;i++){
            Update(receivedBuffer[offset+i]);
        }
        return Value;
    }
};

void AddBytes(std::vector<uint8_t>& bytes, const uint8_t& value, CRC16& crc)
{
    bytes.push_back(value);
    crc.Update(value);
    if(value == 0x98) bytes.push_back(0);
}

std::vector<uint8_t> MakeRequestBytes(const Serial::CSSRequest& request) {
    auto address = request.Address;
    auto command = request.Command;
    std::vector<uint8_t> bytes;
    CRC16 crc16;
    bytes.push_back(0);
    bytes.push_back(0x98);
    bytes.push_back(1);
    AddBytes(bytes, (address >> 8) & 0xFF, crc16);
    AddBytes(bytes, (address) & 0xFF, crc16);
    AddBytes(bytes, (command), crc16);
    for(const auto & b: request.Data)
    {
     AddBytes(bytes, b, crc16);
    }
    uint16_t crcValue = crc16.Value;
    AddBytes(bytes, (crcValue>> 8) & 0xFF, crc16);
    AddBytes(bytes, (crcValue) & 0xFF, crc16);
    bytes.push_back(0x98);
    bytes.push_back(2);
    bytes.push_back(0);
    return bytes;
}

Serial::CDeviceInfo Serial::CSimplSerialBus::RequestDeviceInfo(const uint16_t &address){
    auto result = Request({address, 254, {}}, 5);
    CDeviceInfo device;
    if(result.ResponseState == ok){
        if(result.Result == 0 && result.Data.size()>=54){
            std::vector<uint8_t> guid;
            for(int i=0;i<16;i++){
                device.DeviceGuid.Value.push_back(result.Data[i]);
            }
            {
                std::stringstream ss;
                for (int i = 16; i < 48; i++) {
                    if(result.Data[i]>=0x20 && result.Data[i]<0x7E){
                        if(!(result.Data[i]==0x20 && result.Data[i-1]==0x20)){
                            ss << static_cast<char>(result.Data[i]);
                        }
                    }
                }
                device.DeviceName = ss.str();
            }
            {
                std::stringstream ss;
                for (int i = 48; i < 54; i++) {
                    ss << static_cast<char>(result.Data[i]);
                }
                device.DeviceDate = ss.str();
            }
        }
    }else{
        std::stringstream ss;
        ss << "RequestDeviceInfo: bad response: " << result.ResponseState << " " << result.Result;
        throw std::runtime_error(ss.str());
    }
    return device;
}

void Serial::CSimplSerialBus::RequestSetAddress(CSSGuid &guid, const uint16_t &address)
{
    std::vector<uint8_t> pack;
    pack.insert(pack.begin(), guid.Value.begin(), guid.Value.end());
    pack.push_back(static_cast<uint8_t>((address>>8) & 0xFF));
    pack.push_back(static_cast<uint8_t>((address) & 0xFF));
    auto result = Request({0, 253, pack}, 5);
    if(result.ResponseState == ok && result.Result==0)return;
    std::stringstream ss;
    ss << "RequestSetAddress: bad response: " << result.ResponseState << " " << result.Result;
    throw std::runtime_error(ss.str());
}

Serial::CSSResponse Serial::CSimplSerialBus::Read(const uint16_t &timeout)
{
    Serial::CSSResponse result;
    result.ResponseState = errorTimeout;
    auto time = std::chrono::high_resolution_clock::now();
    size_t receivedLength = 0;
    uint8_t receivedBuffer[1025];

    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time).count()
           < timeout && result.ResponseState == errorTimeout)
    {
        bool readSuccess = false;
        uint8_t currbyte;
        uint8_t lastData;
        try {
            uint8_t buffer[1];
            if (_bus->Read(buffer,0,1) > 0)
            {
                currbyte = buffer[0];
                readSuccess = true;
            }
        } catch (std::exception& e) {
            result.ResponseState = errorPortError;
            return result;
        }
        if (readSuccess)
        {
            if (receivedLength > sizeof(receivedBuffer) - 1)
                receivedLength = 0;
            if (lastData == 0x98)
            {
                switch (currbyte)
                {
                    case 0:
                        receivedBuffer[receivedLength] = 0x98; receivedLength += 1;
                        break;
                    case 0x3:
                        receivedLength = 0;
                        break;
                    case 0x4:
                        if (receivedLength > 4)
                        {
                            ushort recvCrc = receivedBuffer[receivedLength - 2] * 256 + receivedBuffer[receivedLength - 1];
                            CRC16 Crc16;
                            auto realCrc = Crc16.ComputeCrc(receivedBuffer, 0, receivedLength - 2);
                            if (recvCrc == realCrc)
                            {
                                result.FromAddress = receivedBuffer[0] * 256 + receivedBuffer[1];
                                result.Result = receivedBuffer[2];
                                auto length = receivedLength - 5;
                                for(int i=0;i<length;i++){
                                    result.Data.push_back(receivedBuffer[3+i]);
                                }
                                result.ResponseState = ok;
                                return result;
                            } else {
                                result.ResponseState = errorCrc;
                                return result;
                            }
                        }
                        break;
                    case 0x98:
                        receivedBuffer[receivedLength] = currbyte; receivedLength += 1;
                        break;
                    default:
                        break;
                }
            }
            else if (currbyte != 0x98)
            {
                receivedBuffer[receivedLength] = currbyte; receivedLength += 1;
            }
            lastData = currbyte;
        }
        else{
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    return result;
}

Serial::CSSResponse Serial::CSimplSerialBus::Request(const CSSRequest &requestPacket, const uint16_t &timeout)
{
    std::lock_guard<std::mutex> guard(serialMutex);
    try {
        uint8_t buffer[1];
        while (_bus->Read(buffer,0,1) != 0 );
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
    }
    Send(requestPacket);
    return Read(timeout);
}

Serial::CSSResponse Serial::CSimplSerialBus::Request(const CSSRequest &requestPacket)
{
    return Request(requestPacket, DefaultTimeout);
}

Serial::CSSResponse Serial::CSimplSerialBus::Request(const CSSRequest &requestPacket, const int &retries)
{
    for(int i=0;i<retries;i++){
        auto response = Request(requestPacket);
        if(response.ResponseState == ok) return response;
    }
    return Request(requestPacket);
}

Serial::CSSResponse Serial::CSimplSerialBus::Request(const uint16_t &address, const uint8_t &command, const std::vector<uint8_t> &data)
{
    return Request(CSSRequest(address, command, data));
}

std::vector<Serial::CSSGuid> Serial::CSimplSerialBus::FindDevices(const uint32_t &seed, const uint16_t &timeout)
{
    std::lock_guard<std::mutex> guard(serialMutex);
    std::vector<uint8_t> seedBytes = {
            static_cast<uint8_t>((seed>>24) & 0xFF),
            static_cast<uint8_t>((seed>>16) & 0xFF),
            static_cast<uint8_t>((seed>>8) & 0xFF),
            static_cast<uint8_t>(seed& 0xFF)
    };
    Send({0,255, seedBytes});
    auto time = std::chrono::high_resolution_clock::now();
    std::vector<CSSGuid> findedDevices;
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - time).count() < timeout){
        try {
            auto response = Read(timeout);
            if(response.ResponseState == ok && response.Data.size()==16){
                findedDevices.push_back(CSSGuid({response.Data}));
            }
        } catch (std::exception& e) {
            std::cout << e.what() << std::endl;
        }
    }
    return findedDevices;
}

std::vector<Serial::CSSGuid> Serial::CSimplSerialBus::FindDevices(const uint16_t & searchTime)
{
    return FindDevices(static_cast<uint32_t >(std::rand()),searchTime);
}

void Serial::CSimplSerialBus::Send(const CSSRequest& requestPacket)
{
    auto request = MakeRequestBytes(requestPacket);
    _bus->Write(&request[0], request.size());
}

void Serial::CSimplSerialBus::Connect(){
    _bus->Open(_baudRate);
}


