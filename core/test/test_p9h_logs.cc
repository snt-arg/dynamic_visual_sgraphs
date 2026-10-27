#include "PedestrianBA.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

int main()
{
    const std::string path = "/tmp/p9h_log_test.log";
    ORB_SLAM3::PedestrianBA::RecordEvent("IMU_INIT", 12.5, 37, 4, 2);
    ORB_SLAM3::PedestrianBA::RecordEvent("RESET", 13.0, 38, 5, 3);
    ORB_SLAM3::PedestrianBA::RecordEvent("SCALE", 14.0, 39, 5, 3, -1, -1, 0, 0, 0, 0, 1.25);
    ORB_SLAM3::PedestrianBA::DumpLevelsLog(path);

    std::ifstream in("/tmp/p9h_log_test_events.log");
    std::string line;
    int rows = 0;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        if (line.find("IMU_INIT 12.500000000 37 4 2") != 0 &&
            line.find("RESET 13.000000000 38 5 3") != 0 &&
            line.find("SCALE 14.000000000 39 5 3") != 0)
            return 1;
        ++rows;
    }
    std::remove(path.c_str());
    std::remove("/tmp/p9h_log_test_events.log");
    std::remove("/tmp/p9h_log_test_found.log");
    if (rows != 3)
        return 1;
    std::cout << "ok: P9-H event rows include sensor timestamp and frame index\n";
    return 0;
}
