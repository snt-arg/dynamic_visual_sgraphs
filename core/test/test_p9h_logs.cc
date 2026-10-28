// P9-H unit tests of the added logs: A2.3 events (ped_levels_events.log) and A3.1 first-write log (ped_log_first.txt).
#include "PedestrianBA.h"

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using Rows = std::vector<std::vector<std::string>>;

static int failures = 0;
#define CHECK(c, m) do { if (!(c)) { ++failures; std::cout << "FAIL: " << m << " (line " << __LINE__ << ")\n"; } else std::cout << "ok:   " << m << "\n"; } while (0)

// data rows of a log with exactly `nfields` whitespace-separated fields each; empty result (and ok = false) if any row has another field count
static Rows rowsOf(const std::string &path, size_t nfields, bool &ok)
{
    Rows out;
    ok = true;
    std::ifstream f(path);
    std::string l;
    while (std::getline(f, l))
    {
        if (l.empty() || l[0] == '#')
            continue;
        std::istringstream ss(l);
        std::vector<std::string> v;
        std::string tok;
        while (ss >> tok)
            v.push_back(tok);
        if (v.size() != nfields)
            ok = false;
        out.push_back(v);
    }
    return out;
}

int main()
{
    using ORB_SLAM3::PedestrianBA;
    // ------------------------------------------------------------------ A2.3 events
    // recorded out of order on purpose: the file is written in canonical (time, frame, type, ...) order
    PedestrianBA::RecordEvent("IMAGE_SCALE", 14.0, 39, 5, 3, -1, -1, 0, 0, 0, 0, 1.25);
    PedestrianBA::RecordEvent("IMU_INIT", 12.5, 37, 4, 2);
    PedestrianBA::RecordEvent("CREATE", 13.0, 38, 5, 3, 0, -1, -1.5, 0, 12, 0);
    PedestrianBA::RecordEvent("RESET", 13.0, 38, 5, 3);

    // ------------------------------------------------------------------ A3.1 first-write log
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Vec7 = Eigen::Matrix<double, 7, 1>;
    Vec6 x1, x2, x3;
    x1 << 1, 2, 3, 0.1, 0.2, 0.3;
    x2 << 1.5, 2.5, 3.5, 0.4, 0.5, 0.6;
    x3 << -1, -2, -3, 0, 0, 0;
    Vec7 p1, p2;
    p1 << 10, 11, 12, 0, 0, 0, 1;
    p2 << 10.5, 11.5, 12.5, 0, 0, 0.1, 0.995;
    PedestrianBA::RecordWrite(1, 0, 12, 4, 5.2, x1, p1, 300.5, 200.25, 40.0, 150.0); // (map 1, epoch 0, kf 12, track 4): written once
    PedestrianBA::RecordWrite(1, 0, 10, 3, 5.0, x3, p1, 1, 2, 3, 4);                 // (1, 0, 10, 3): epoch 0 ...
    PedestrianBA::RecordWrite(1, 0, 11, 3, 5.1, x1, p1, 301.5, 201.25, 41.0, 151.0); // (1, 0, 11, 3): first write x1 ...
    PedestrianBA::RecordWrite(1, 0, 11, 3, 5.1, x2, p2, 999, 999, 999, 999);         // ... then x2: the first-write row must keep x1 and the observation of the first write
    PedestrianBA::RecordWrite(1, 1, 10, 3, 9.0, x3, p1, 5, 6, 7, 8);                // same (kf, track) in the next epoch: a row of its own

    const std::string base = "/tmp/p9h_log_test", lp = base + "_last.txt", fp = base + "_first.txt";
    PedestrianBA::DumpLevelsLog(base + ".log");
    PedestrianBA::DumpLog(lp);
    PedestrianBA::DumpFirstLog(fp);

    bool ok = false;
    const Rows ev = rowsOf(base + "_events.log", 13, ok); // type stamp frame map epoch id other_id height other_height contacts other_contacts scale tag
    CHECK(ok && ev.size() == 4, "event rows: 13 fields each, 4 rows");
    std::vector<std::string> order;
    for (const auto &r : ev)
        order.push_back(r[0]);
    CHECK(order == std::vector<std::string>({"IMU_INIT", "CREATE", "RESET", "IMAGE_SCALE"}), "events are written in canonical order, not arrival order");
    CHECK(ev.size() == 4 && ev[0][1] == "12.500000000" && ev[0][2] == "37" && ev[0][3] == "4" && ev[0][4] == "2", "IMU_INIT row: sensor time, frame index, map, epoch");
    CHECK(ev.size() == 4 && ev[3][0] == "IMAGE_SCALE" && ev[3][11] == "1.25" && ev[3][12] == "-", "IMAGE_SCALE row carries the image scale, tag \"-\"");

    const Rows first = rowsOf(fp, 22, ok); // map epoch kf stamp track px py pz vx vy vz tx ty tz qx qy qz qw root_u root_v lpx mask_h
    CHECK(ok && first.size() == 4, "first-write log parses: 22 fields per row, one row per (map, epoch, kf, track)");
    const Rows last = rowsOf(lp, 16, ok); // kf stamp track px py pz vx vy vz tx ty tz qx qy qz qw
    CHECK(ok && last.size() == 3, "last-write log is unchanged: one row per (kf, track), 16 fields");
    auto hex = [](double v) {
        char b[64];
        std::snprintf(b, sizeof(b), "%a", v);
        return std::string(b);
    };
    if (first.size() == 4 && last.size() == 3)
    {
        // first rows in key order: (1,0,10,3) (1,0,11,3) (1,0,12,4) (1,1,10,3); last rows in (kf, track) order: (10,3) (11,3) (12,4)
        bool same = true;
        for (size_t i = 0; i < 13; ++i)
            same = same && first[2][5 + i] == last[2][3 + i];
        CHECK(same, "a state written once: the first-write row equals the last-write row (position, velocity, Twc)");
        CHECK(first[2][18] == hex(300.5) && first[2][19] == hex(200.25) && first[2][20] == hex(40.0) && first[2][21] == hex(150.0), "first-write row carries the measurement inputs (root pixel, lpx, mask height)");
        CHECK(first[1][5] == hex(1.0) && last[1][3] == hex(1.5), "two write-backs of (1,0,11,3): the first-write row holds the FIRST values, the last-write log the last");
        CHECK(first[1][18] == hex(301.5) && first[1][20] == hex(41.0), "two write-backs: the first-write row keeps the observation of the first write");
        CHECK(first[0][5] == hex(-1.0) && first[3][5] == hex(-1.0) && first[0][1] == "0" && first[3][1] == "1", "the same (kf, track) in another epoch has a row of its own (epoch column 0 and 1)");
        CHECK(first[0][0] == "1" && first[0][2] == "10" && first[0][4] == "3" && first[2][4] == "4" && first[1][3] == "5.100000000", "map id, kf id, track and sensor time are logged");
    }
    return failures ? 1 : 0;
}
