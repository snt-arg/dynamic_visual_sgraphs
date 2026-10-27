#include "PedestrianBA.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main()
{
    const std::string path = "/tmp/p9h_log_test.log";
    // recorded out of order on purpose: the file is written in canonical (time, frame, type, ...) order
    ORB_SLAM3::PedestrianBA::RecordEvent("SCALE", 14.0, 39, 5, 3, -1, -1, 0, 0, 0, 0, 1.25);
    ORB_SLAM3::PedestrianBA::RecordEvent("IMU_INIT", 12.5, 37, 4, 2);
    ORB_SLAM3::PedestrianBA::RecordEvent("CREATE", 13.0, 38, 5, 3, 0, -1, -1.5, 0, 12, 0);
    ORB_SLAM3::PedestrianBA::RecordEvent("RESET", 13.0, 38, 5, 3);
    ORB_SLAM3::PedestrianBA::DumpLevelsLog(path);

    std::ifstream in("/tmp/p9h_log_test_events.log");
    std::string line;
    int rows = 0;
    std::vector<std::string> order;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream ss(line);
        std::string tok;
        int n = 0;
        while (ss >> tok)
            ++n;
        if (n != 12) // type stamp frame map epoch id other_id height other_height contacts other_contacts scale
            return 2;
        if (line.find("IMU_INIT 12.500000000 37 4 2") != 0 &&
            line.find("RESET 13.000000000 38 5 3") != 0 &&
            line.find("CREATE 13.000000000 38 5 3 0 -1") != 0 &&
            line.find("SCALE 14.000000000 39 5 3") != 0)
            return 1;
        order.push_back(line.substr(0, line.find(' ')));
        ++rows;
    }
    std::remove(path.c_str());
    std::remove("/tmp/p9h_log_test_events.log");
    std::remove("/tmp/p9h_log_test_found.log");
    if (rows != 4 || order != std::vector<std::string>{"IMU_INIT", "CREATE", "RESET", "SCALE"}) // canonical order, not arrival order
        return 1;
    std::cout << "ok: P9-H event rows have 12 fields incl. sensor timestamp, frame index, map and epoch\n";
    return 0;
}
