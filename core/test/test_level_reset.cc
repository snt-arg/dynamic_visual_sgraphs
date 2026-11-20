// [p9-level-reset] gate 1 (order of 19 Nov 2026): reset_active_map destroys the level set. Separate process (the event log is a process-wide static).
// Path under test = the one Tracking::ResetActiveMap takes: Atlas::clearMap(stamp, frame) -> Map::clear(stamp, frame). Then an IMU-initialisation similarity
// with s = 58 (Map::ApplyScaledRotation, cause IMU_INIT0): no level may survive or be carried. Controls: a map without a level set is left untouched by
// clear (epoch unchanged, no EPOCH row: OFF / ON runs unchanged); a map whose set is not cleared still carries its levels (the carry itself is intact).
// On M0 (28cc0f4) the first block fails: the two levels survive the clear and are carried to 58 x their height.
#include "Atlas.h"
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

static ped::LevelParams TP()   // = test_p9h_carry.cc
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

static void Feed(ped::LevelSet &S, double z0, double zLow)   // = test_p9h_carry.cc: two levels, 4 tracks x 30 contacts
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

static size_t NLevels(Map &m, const ped::LevelParams &P)   // PedLevels creates an empty set when none exists (as the BA does): size 0 either way
{
    std::unique_lock<std::mutex> lk(m.PedLevelsMutex());
    return m.PedLevels(P).Levels().size();
}

int main()
{
    const ped::LevelParams P = TP();
    const Sophus::SE3f Timu(Eigen::AngleAxisf(0.3f, Eigen::Vector3f::UnitZ()).toRotationMatrix(), Eigen::Vector3f(0.1f, 0.2f, -0.3f));
    const float s = 58.f;

    // 1. levels, reset_active_map, IMU initialisation with s = 58: nothing survives, nothing is carried
    Atlas atlas(0);
    Map *m = atlas.GetCurrentMap();
    {
        std::unique_lock<std::mutex> lk(m->PedLevelsMutex());
        Feed(m->PedLevels(P), -1.4, -7.4);
        m->PedLevels(P).TakeEvents();
    }
    CHECK(NLevels(*m, P) == 2, "setup: two levels in the active map");
    const unsigned long e0 = m->PedLevelsEpoch();
    atlas.clearMap(100.5, 2000);                                     // Tracking::ResetActiveMap's call
    CHECK(m->PedLevelsEpoch() == e0 + 1, "reset_active_map advances the epoch by one (the set is destroyed: LevelStore::Reset)");
    m->ApplyScaledRotation(Timu, s, true, "IMU_INIT0", 104.0, 2030);
    CHECK(NLevels(*m, P) == 0, "after the IMU initialisation (s = 58) no level survives");
    CHECK(m->PedLevelsEpoch() == e0 + 2, "the IMU initialisation advances the epoch as before");

    // 2. control: a map without a level set (OFF / ON) is untouched by clear
    Map bare(7);
    const unsigned long b0 = bare.PedLevelsEpoch();
    bare.clear(200.0, 3000);
    CHECK(bare.PedLevelsEpoch() == b0, "clear on a map without a level set: epoch unchanged (no EPOCH row: checked in the log below)");

    // 3. control: without a clear the levels are still carried (s R c + t)
    Map kept(8);
    {
        std::unique_lock<std::mutex> lk(kept.PedLevelsMutex());
        Feed(kept.PedLevels(P), -1.4, -7.4);
        kept.PedLevels(P).TakeEvents();
    }
    double z0 = 0;
    {
        std::unique_lock<std::mutex> lk(kept.PedLevelsMutex());
        z0 = kept.PedLevels(P).Levels()[0].z;
    }
    kept.ApplyScaledRotation(Timu, 1.1f, true, "IMU_INIT0", 300.0, 4000);
    {
        std::unique_lock<std::mutex> lk(kept.PedLevelsMutex());
        const auto &Ls = kept.PedLevels(P).Levels();
        CHECK(Ls.size() == 2 && std::abs(Ls[0].z - (1.1 * z0 + Timu.translation()(2))) < 1e-4, "control: an uncleared set is still carried through the similarity");
    }

    // event log: one RESET_ACTIVE_MAP EPOCH row (map 0 only); no CARRY row of map 0 after the reset; map 8 carries 2
    const std::string base = "/tmp/p9_level_reset_test";
    PedestrianBA::DumpLevelsLog(base + ".log");
    std::ifstream f(base + "_events.log");
    std::string l;
    int bad = 0, resetRows = 0, resetRowsOther = 0, carry0 = 0, carry8 = 0;
    double resetT = 0;
    while (std::getline(f, l))
    {
        if (l.empty() || l[0] == '#') continue;
        std::istringstream ss(l);
        std::vector<std::string> v;
        std::string tok;
        while (ss >> tok) v.push_back(tok);
        if (v.size() != 13) { ++bad; continue; }
        if (v[0] == "EPOCH" && v[12] == "RESET_ACTIVE_MAP") { if (v[3] == std::to_string(m->GetId())) { ++resetRows; resetT = std::stod(v[1]); } else ++resetRowsOther; }
        if (v[0] == "CARRY") { if (v[3] == std::to_string(m->GetId())) ++carry0; else if (v[3] == "8") ++carry8; }
    }
    CHECK(bad == 0, "every event row has 13 fields");
    CHECK(resetRows == 1 && resetRowsOther == 0 && std::abs(resetT - 100.5) < 1e-6, "one EPOCH row RESET_ACTIVE_MAP, at the reset's stamp, only for the map that had levels");
    CHECK(carry0 == 0, "no CARRY row for the reset map (no stale level reaches the IMU-init similarity)");
    CHECK(carry8 == 2, "control map carries its 2 levels");
    std::cout << (failures ? "FAILURES: " : "all passed, failures = ") << failures << "\n";
    return failures ? 1 : 0;
}
