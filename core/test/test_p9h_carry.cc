// P9-H A4.4 / A5 (gate K1): Map-level tests of the epoch logging and the level carry-over. Separate process from test_p9h_logs (the event log is a process-wide static).
// Covered here: every cause of an EPOCH row (IMU_INIT0, STAGE1, STAGE2, SCALE_REFINE, LOOP, MERGE, OTHER), MAP_SCALE and CARRY rows, the analytic similarity transform of the carried levels,
// the person-state bias still cleared, the epoch still incremented. A6: no loop carry exists.
#include "Map.h"
#include "PedestrianBA.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace ORB_SLAM3;
static int failures = 0;
#define CHECK(c, m) do { if (!(c)) { ++failures; std::cout << "FAIL: " << m << " (line " << __LINE__ << ")\n"; } else std::cout << "ok:   " << m << "\n"; } while (0)

static ped::LevelParams TP()
{
    ped::LevelParams P;
    P.sigmaA = 0.05; P.sigmaPt = 0.03; P.sMax = 0.2;
    P.assocK = 3; P.createK = 2; P.mergeK = 2;
    P.createMinContacts = 10; P.createMinTracks = 2; P.createMinExtentM = 1.0; P.edgeGuardS = 1.5;
    P.transWindowS = 1.5; P.transMinTravelM = 0.5; P.transMinContacts = 4;
    P.ransacThrK = 2; P.ransacRadiusM = 1.0; P.ransacMinInliers = 5; P.ransacMaxOffsetK = 3;
    P.ransacIters = ped::RansacIterations(0.99, 0.5);
    return P;
}

static void Feed(ped::LevelSet &S, double z0, double zLow)
{
    std::mt19937 g(3);
    std::normal_distribution<double> N(0, 0.01);
    for (int track = 1; track <= 4; ++track)
    {
        std::vector<ped::Contact> cs;
        for (int i = 0; i < 30; ++i)
        {
            ped::Contact c;
            c.track = track; c.kf = static_cast<unsigned long>(i); c.t = i / 6.0; c.x = c.t; c.y = 0.1 * track; c.h = (track <= 2 ? z0 : zLow) + N(g);
            cs.push_back(c);
        }
        S.Update(cs);
    }
}

int main()
{
    const ped::LevelParams P = TP();
    Map map(0);
    map.SetPedBias(3, 0.5);
    {
        std::unique_lock<std::mutex> lk(map.PedLevelsMutex());
        Feed(map.PedLevels(P), 0.0, -1.0);
        map.PedLevels(P).TakeEvents();
    }
    double b = 0;
    CHECK(map.GetPedBias(3, b), "setup: a per-track bias is stored");

    // similarity event with a yaw plus a tilt
    const double yaw = 0.4, pitch = 0.01, roll = -0.008;
    Eigen::Matrix3f R = (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())).toRotationMatrix();
    const Eigen::Vector3f t(0.3f, -0.2f, 0.9f);
    const float s = 1.17f;
    std::vector<std::array<double, 3>> before;
    {
        std::unique_lock<std::mutex> lk(map.PedLevelsMutex());
        for (const ped::Level &L : map.PedLevels(P).Levels())
            before.push_back({L.mx, L.my, L.z});
    }
    CHECK(before.size() == 2, "setup: two levels");
    const unsigned long e0 = map.PedLevelsEpoch();
    const Sophus::SE3f T1(R, t);   // Sophus re-orthonormalises R in float: the analytic reference uses the rotation the map is actually given
    const Eigen::Matrix3d Rm = T1.rotationMatrix().cast<double>();
    map.ApplyScaledRotation(T1, s, true, "STAGE1", 12.5, 345);
    {
        std::unique_lock<std::mutex> lk(map.PedLevelsMutex());
        const auto &Ls = map.PedLevels(P).Levels();
        CHECK(Ls.size() == 2 && map.PedLevelsEpoch() == e0 + 1, "the levels survive a scaled rotation and the epoch still increments");
        double worst = 0;
        for (size_t i = 0; i < Ls.size() && i < before.size(); ++i)
        {
            const Eigen::Vector3d c(before[i][0], before[i][1], before[i][2]);
            const Eigen::Vector3d e = static_cast<double>(s) * Rm * c + T1.translation().cast<double>();
            worst = std::max({worst, std::abs(Ls[i].mx - e(0)), std::abs(Ls[i].my - e(1)), std::abs(Ls[i].z - e(2))});
        }
        CHECK(worst < 1e-9, "Map::ApplyScaledRotation carries (x, y, z) as s R c + t within 1e-9 (worst " << worst << ")");
    }
    CHECK(!map.GetPedBias(3, b), "the person-state bias is still cleared");

    // the other causes
    map.ApplyScaledRotation(Sophus::SE3f(R, t), 0.99f, true, "STAGE2", 30.0, 700);
    map.ApplyScaledRotation(Sophus::SE3f(R, t), 1.01f, true, "SCALE_REFINE", 63.5, 1500);
    map.ApplyScaledRotation(Sophus::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero()), 1.f, true, "OTHER", 64.0, 1510);
    map.ResetPedestrians("LOOP", 612.5, 11000);   // CorrectLoop (loop closing is off in the VIO configuration; this is the unchanged destroy)
    {
        std::unique_lock<std::mutex> lk(map.PedLevelsMutex());
        CHECK(map.PedLevels(P).Levels().empty(), "a loop correction destroys the level set (no loop carry, A6)");
    }
    Map map2(1);
    {
        std::unique_lock<std::mutex> lk(map2.PedLevelsMutex());
        Feed(map2.PedLevels(P), 0.0, -1.0);
        map2.PedLevels(P).TakeEvents();
    }
    map2.ApplyScaledRotation(Sophus::SE3f(R, t), 1.1f, true, "IMU_INIT0", 19.2, 61);
    map2.ResetPedestrians("MERGE", 700.0, 12000);
    {
        std::unique_lock<std::mutex> lk(map2.PedLevelsMutex());
        CHECK(map2.PedLevels(P).Levels().empty() && map2.PedLevelsEpoch() == 2, "a merge destroys the set and increments the epoch");
    }

    const std::string base = "/tmp/p9h_carry_test";
    PedestrianBA::DumpLevelsLog(base + ".log");
    std::ifstream f(base + "_events.log");
    std::string l;
    std::map<std::string, int> epochCauses, nType;
    int bad = 0, carry = 0, carryDrop = 0, mapScale = 0, epochRows = 0;
    while (std::getline(f, l))
    {
        if (l.empty() || l[0] == '#') continue;
        std::istringstream ss(l);
        std::vector<std::string> v;
        std::string tok;
        while (ss >> tok) v.push_back(tok);
        if (v.size() != 13) { ++bad; continue; }
        ++nType[v[0]];
        if (v[0] == "EPOCH") { ++epochRows; ++epochCauses[v[12]]; }
        if (v[0] == "CARRY") ++carry;
        if (v[0] == "CARRY_DROP") ++carryDrop;
        if (v[0] == "MAP_SCALE") ++mapScale;
    }
    CHECK(bad == 0, "every event row has 13 fields (0 parse problems)");
    // map: 4 scaled rotations + 1 loop; map2: 1 scaled rotation + 1 merge reset; 7 increments, 7 EPOCH rows
    CHECK(epochRows == 7, "one EPOCH row per epoch increment (7 of 7)");
    CHECK(epochCauses["IMU_INIT0"] == 1 && epochCauses["STAGE1"] == 1 && epochCauses["STAGE2"] == 1 && epochCauses["SCALE_REFINE"] == 1 && epochCauses["LOOP"] == 1 && epochCauses["MERGE"] == 1 && epochCauses["OTHER"] == 1,
          "every cause appears exactly once (IMU_INIT0, STAGE1, STAGE2, SCALE_REFINE, LOOP, MERGE, OTHER)");
    CHECK(mapScale == 5, "one MAP_SCALE row per ApplyScaledRotation (5)");
    // carried levels: 2 per rotation on map until the loop drop, 2 on map2 for IMU_INIT0; the merge reset carries none
    CHECK(carry == 2 * 4 + 2 && carryDrop == 0, "10 CARRY rows (2 levels x 4 rotations on map, 2 on map2), no CARRY_DROP");
    std::cout << (failures ? "FAILURES: " : "all passed, failures = ") << failures << "\n";
    return failures ? 1 : 0;
}
