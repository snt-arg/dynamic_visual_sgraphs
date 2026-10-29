// init-trap unit test (header-only, no SLAM): every event type is written as a parseable 15-column row, the defaults are inert,
// and the three Debug.* keys are read from a settings file.   usage: test_init_events [tmp_dir]
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "InitEvents.h"

using namespace ORB_SLAM3;

static int failures = 0;
#define CHECK(cond, msg)                                 \
    do                                                   \
    {                                                    \
        if (!(cond))                                     \
        {                                                \
            std::cerr << "FAIL: " << msg << std::endl;   \
            ++failures;                                  \
        }                                                \
        else                                             \
            std::cout << "ok:   " << msg << std::endl;   \
    } while (0)

static std::vector<std::string> split(const std::string &s)
{
    std::vector<std::string> v;
    std::stringstream ss(s);
    std::string x;
    while (std::getline(ss, x, '\t'))
        v.push_back(x);
    return v;
}

int main(int argc, char **argv)
{
    const std::string dir = argc > 1 ? argv[1] : "/tmp";
    const std::string log = dir + "/test_init_events.log";
    const double nan = InitEvents::Nan();

    // defaults: no key -> inert
    {
        cv::FileStorage fs("{}", cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_YAML);
        InitEvents::Configure(fs);
        CHECK(!InitEvents::Logging(), "no Debug.InitEventsFile: no log");
        CHECK(!InitEvents::DisableNEMReset(), "default DisableNEMReset = 0");
        CHECK(InitEvents::InitMinTimeMono() == 2.0f, "default InitMinTime = 2.0");
        InitEvents::Write(1, 1, 0, 1, 0, "NEM", -1, nan, nan, nan, nan, nan, 1, 0.01, "x"); // must not crash
    }
    // keys read from a settings file
    {
        const std::string y = "%YAML:1.0\nDebug.InitEventsFile: \"" + log + "\"\nDebug.DisableNEMReset: 1\nDebug.InitMinTime: 10.0\n";
        cv::FileStorage fs(y, cv::FileStorage::READ | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_YAML);
        InitEvents::Configure(fs);
        CHECK(InitEvents::Logging(), "Debug.InitEventsFile opens the log");
        CHECK(InitEvents::DisableNEMReset(), "Debug.DisableNEMReset = 1 read");
        CHECK(InitEvents::InitMinTimeMono() == 10.0f, "Debug.InitMinTime = 10 read");
    }
    // roll/pitch of a known rotation: 30 deg about x then 20 deg about y (R = Ry * Rx)
    {
        const double r = 30 * M_PI / 180, p = 20 * M_PI / 180;
        // R = Ry(p) Rx(r): R20 = -sin p, R21 = cos p sin r, R22 = cos p cos r
        double roll, pitch;
        InitEvents::RollPitchDeg(-std::sin(p), std::cos(p) * std::sin(r), std::cos(p) * std::cos(r), roll, pitch);
        CHECK(std::fabs(roll - 30) < 1e-9 && std::fabs(pitch - 20) < 1e-9, "RollPitchDeg recovers 30/20 deg");
    }
    // one row per event type
    InitEvents::Write(12.25, 117, 0, 14, 3.5, "INIT_SOLVE", 0, 5.4321, 1.5, -2.5, 0.07, 0.003, 1, nan, "applied=1");
    InitEvents::Write(12.0, 110, 0, 12, nan, "INIT_SKIP", 0, nan, nan, nan, nan, nan, 0, nan, "min_time");
    InitEvents::Write(40.5, 900, 0, 80, 0.0, "NEM", -1, nan, nan, nan, nan, nan, 0, 0.0123, "reset_disabled");
    InitEvents::Write(75.2, 2400, 0, 150, 25.1, "SCALE_REFINE", -1, 1.012, nan, nan, nan, nan, 1, nan, "accepted");
    InitEvents::Write(384.0, 9000, 0, 1680, nan, "MAP_RESET", -1, nan, nan, nan, nan, nan, 1, nan, "tracking:tracking_lost_imu_not_initialized_or_ba2_pending");
    InitEvents::Close();

    std::ifstream f(log);
    std::string line;
    std::vector<std::string> rows;
    while (std::getline(f, line))
        if (!line.empty() && line[0] != '#')
            rows.push_back(line);
    CHECK(rows.size() == 6, "header + 5 event rows written (got " + std::to_string(rows.size()) + " lines)");
    if (rows.size() == 6)
    {
        const size_t ncol = split(rows[0]).size();
        CHECK(ncol == 15, "header has 15 columns");
        const char *names[] = {"INIT_SOLVE", "INIT_SKIP", "NEM", "SCALE_REFINE", "MAP_RESET"};
        for (int i = 0; i < 5; ++i)
        {
            auto c = split(rows[i + 1]);
            CHECK(c.size() == ncol && c[5] == names[i], std::string("row parses, event ") + names[i]);
            CHECK(!std::isnan(std::stod(c[0])), std::string(names[i]) + ": t_kf numeric");
        }
        auto c = split(rows[1]);
        CHECK(std::fabs(std::stod(c[7]) - 5.4321) < 1e-9 && c[6] == "0" && c[12] == "1", "INIT_SOLVE keeps stage, solved scale, accepted flag");
        CHECK(std::fabs(std::stod(split(rows[3])[13]) - 0.0123) < 1e-9, "NEM keeps dist");
    }
    std::remove(log.c_str());
    std::cout << (failures ? "FAILED" : "ALL PASSED") << std::endl;
    return failures ? 1 : 0;
}
