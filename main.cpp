#include <iostream>
#include <vector>
#include <string>
#include "SerialPort.h"
#include "CSimplSerial.h"

using namespace std;

int main() {
    try{
        auto serialPorts = Serial::GetSerialPort();
        Serial::CSimplSerialBus _bus(serialPorts[0], 9600);
        _bus.Connect();
        while(true){
            auto response = _bus.FindDevices(7000);
            cout << "Detected device: " << response.size() << endl;
            for(int i=0;i<response.size();i++){
                _bus.RequestSetAddress(response[i], i+10);
                auto devInfo = _bus.RequestDeviceInfo(i+10);
                cout <<"["<< devInfo.DeviceGuid.GetString() <<"]: \""<< devInfo.DeviceName << "\" - " << devInfo.DeviceDate << endl;
            }
        }
    } catch (exception & e) {
        cout << e.what() << endl;
    }
    return 0;
}
