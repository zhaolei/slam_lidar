#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include "tofbf.h"
#include <string>
using namespace std;
#define RADIAN_TO_ANGLED(angle) ((angle)*180000/3141.59)

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>

int main(int argc, char **argv) {
    LiPkg *lidar = new LiPkg;
    CmdInterfaceLinux cmd_port;
    std::vector<std::pair<std::string, std::string> > device_list;
    std::string port_name;
    cmd_port.GetCmdDevices(device_list);
    for (auto n : device_list) {
        std::cout << n.first << "    " << n.second << std::endl;
        if (strstr(n.second.c_str(), "CP2102")) {
            port_name = n.first;
        }
    }
    struct timeval tv;
    __suseconds_t prev_time =-10;
    uint16_t prev_timestamp = -10;
    if (port_name.empty()) {
        std::cout << "Can't find LiDAR LD06" << std::endl;
    }
    std::cout << "FOUND LiDAR_LD06" << std::endl;
    cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
        if (lidar->Parse((uint8_t *) byte, len)) {
            lidar->AssemblePacket();
        }
    });
    if (cmd_port.Open(port_name))
        std::cout << "LiDAR_LD06 started successfully " << std::endl;

    while (true) {
        gettimeofday(&tv, NULL);
        __suseconds_t current_time = tv.tv_usec / 1000;
        if (abs(prev_time - current_time) > 1) {
            if (prev_timestamp==lidar->GetTimestamp())break;
            prev_timestamp=lidar->GetTimestamp();
        }
        prev_time = current_time;
        if (lidar->IsFrameReady()) {
            lidar->ResetFrameReady();
            auto data = lidar->GetLaserData();
            //unsigned int lens = (data.angle_max - data.angle_min) / data.angle_increment;
            unsigned int lens = data.size(); 

            std::cout << "current_speed: " << lidar->GetSpeed() << " "
                      << "len: " << lens <<std::endl; 
            std::cout << "----------------------------" << std::endl;
            for (int i = 0; i < lens; i++)
            {
                std::cout << "range: " <<  data[i] << " "
                          //<< "intensites: " <<  data.intensities[i] << std::endl;
                           << std::endl;
            }
            std::cout << "----------------------------" << std::endl;
        }
    }
    return 0;
}

