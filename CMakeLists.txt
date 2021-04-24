cmake_minimum_required(VERSION 3.15)
project(SimplSerial)

set(CMAKE_CXX_STANDARD 17)

add_executable(SimplSerial main.cpp SerialPort.h SerialPort.cpp CSimplSerial.cpp CSimplSerial.h)
target_link_libraries(SimplSerial stdc++fs)