// P9-C5: unit tests of PedestrianLevels.h (standalone: g++ -std=c++17 -I../include test_levels.cc). Test parameters are arbitrary test values,
// not the calibrated constants of constants_c5.md.
#include <iostream>
#include <random>
#include "PedestrianLevels.h"

using namespace ORB_SLAM3::ped;
static int failures = 0;
#define CHECK(c, m) do { if (!(c)) { ++failures; std::cout << "FAIL: " << m << " (line " << __LINE__ << ")\n"; } else std::cout << "ok:   " << m << "\n"; } while (0)

static LevelParams TP()
{
    LevelParams P;
    P.sigmaA = 0.05; P.sigmaPt = 0.03; P.sMax = 0.2;
    P.assocK = 3; P.createK = 2; P.mergeK = 2;
    P.createMinContacts = 10; P.createMinTracks = 2;
    P.transWindowS = 1.5; P.transMinTravelM = 0.5; P.transMinContacts = 4;
    P.ransacThrK = 2; P.ransacRadiusM = 1.0; P.ransacMinInliers = 5; P.ransacMaxOffsetK = 3;
    P.ransacIters = RansacIterations(0.99, 0.5);
    return P;
}

// walk of `n` contacts at 6 Hz, speed v m/s along +x, height h0 + slope * distance
static std::vector<Contact> Walk(int track, int n, double v, double h0, double slope, double t0 = 0, std::mt19937 *g = nullptr, double noise = 0)
{
    std::vector<Contact> cs;
    std::normal_distribution<double> N(0, noise > 0 ? noise : 1);
    for (int i = 0; i < n; ++i)
    {
        Contact c;
        c.track = track; c.kf = static_cast<unsigned long>(i); c.t = t0 + i / 6.0; c.x = v * c.t; c.y = 0.1 * track;
        c.h = h0 + slope * (v * (c.t - t0)) + (g && noise > 0 ? N(*g) : 0.0);
        cs.push_back(c);
    }
    return cs;
}

int main()
{
    const LevelParams P = TP();
    std::mt19937 g(5);
    CHECK(RansacIterations(0.99, 0.5) == 7 && DesignLevelParams(1,1,1).ransacIters == 7, "RANSAC iterations for 99% at 50% inliers, 1-D model = 7");

    // ---- transitional
    { auto c = Walk(1, 30, 1.2, 0.0, 0.0, 0, &g, 0.015); FlagTransitional(c, P); int n = 0; for (auto &x : c) n += x.transitional; CHECK(n == 0, "flat walk (noise 1.5 cm) is not flagged"); }
    { auto c = Walk(1, 30, 0.6, 0.0, 0.175 / 0.28, 0, &g, 0.015); FlagTransitional(c, P); int n = 0; for (auto &x : c) n += x.transitional; CHECK(n >= 27, "stairs (slope 0.625) are flagged"); }
    { auto c = Walk(1, 30, 0.0, 0.0, 0.0, 0, &g, 0.05); FlagTransitional(c, P); int n = 0; for (auto &x : c) n += x.transitional; CHECK(n == 0, "standing still (noisy heights) is not flagged"); }
    { auto c = Walk(1, 30, 0.2, 0.0, 0.0, 0, &g, 0.0); for (size_t i = 0; i < c.size(); ++i) c[i].h = 0.4 * (i % 2); FlagTransitional(c, P); int n = 0; for (auto &x : c) n += x.transitional; CHECK(n == 0, "slow shuffle (travel < 0.5 m per window) with jumping heights is not flagged");}

    // ---- creation / association / merging
    { LevelSet S(P); auto a = Walk(1, 12, 1.0, 0.02, 0, 0, &g, 0.01); S.Update(a); CHECK(S.Levels().empty() && S.PoolSize() == 12, "12 contacts from one track create no level");
      auto b = Walk(2, 12, 1.0, 0.02, 0, 0, &g, 0.01); S.Update(b); CHECK(S.Levels().size() == 1 && std::abs(S.Levels()[0].z - 0.02) < 0.01, "contacts from 2 tracks create one level at the median");
      auto c = Walk(3, 5, 1.0, 0.04, 0, 0, &g, 0.005); S.Update(c); int as = 0; for (auto &x : c) as += x.level == 0; CHECK(as == 5, "new contacts within 3 sigma_a associate to the level");
      Contact far; far.track = 4; far.h = 0.5; far.t = 0; CHECK(S.Associate(far) == -1, "a contact 0.48 m away is unassigned"); }
    { LevelSet S(P); auto a = Walk(1, 12, 1.0, 0.0, 0, 0, &g, 0.01), b = Walk(2, 12, 1.0, 0.0, 0, 0, &g, 0.01), c = Walk(3, 12, 1.0, -0.8, 0, 0, &g, 0.01), d = Walk(4, 12, 1.0, -0.8, 0, 0, &g, 0.01);
      for (auto *v : {&a, &b, &c, &d}) S.Update(*v);
      CHECK(S.Levels().size() == 2, "two levels 0.8 m apart are both created"); S.SetHeight(S.Levels()[1].id, S.Levels()[0].z + 0.04); S.MergeClose();
      CHECK(S.Levels().size() == 1 && S.Merges() == 1, "two levels closer than 2 sigma_a merge"); }
    { LevelSet S(P); auto a = Walk(1, 12, 1.0, 0.0, 0, 0, &g, 0.01), b = Walk(2, 12, 1.0, 0.0, 0, 0, &g, 0.01); for (auto &x : a) x.transitional = true; for (auto &x : b) x.transitional = true;
      S.Update(a); S.Update(b); CHECK(S.Levels().empty() && S.PoolSize() == 0, "transitional contacts never enter the pool nor create a level"); }
    { LevelSet S(P); std::vector<Contact> none; S.Update(none); CHECK(S.Levels().empty(), "a furniture cluster (points only, no contacts) never becomes a level: the level set has no point input"); }

    { LevelSet S(P); auto a = Walk(1, 12, 1.0, 0.0, 0, 0, &g, 0.01), b = Walk(2, 12, 1.0, 0.0, 0, 0, &g, 0.01); S.Update(a); S.Update(b);
      const int n1 = S.Levels()[0].nContacts; auto a2 = a, b2 = b; S.Update(a2); S.Update(b2);
      CHECK(S.Levels().size() == 1 && S.Levels()[0].nContacts == n1, "overlapping BA windows: the same (keyframe, track) contacts are not counted twice");
      auto lone = Walk(7, 12, 1.0, 1.5, 0, 0, &g, 0.01); S.Update(lone); auto lone2 = lone; S.Update(lone2); CHECK(S.PoolSize() == 12, "overlapping windows: the same contacts are not pooled twice"); }

    // ---- RANSAC
    {
        std::vector<Contact> cs = Walk(1, 6, 1.0, 0.0, 0, 0); std::vector<PointZ> pts; int id = 0; std::normal_distribution<double> N(0, 0.03); std::uniform_real_distribution<double> U(-1, 1);
        for (int i = 0; i < 100; ++i) { PointZ p{id++, 2.5 + U(g), U(g) * 0.5, 0.0 + N(g)}; pts.push_back(p); }          // floor, 100
        for (int i = 0; i < 60; ++i) { PointZ p{id++, 2.5 + U(g), U(g) * 0.5, 0.75 + 0.02 * U(g)}; pts.push_back(p); }   // furniture, 60 (cluster at +0.75)
        for (int i = 0; i < 70; ++i) { PointZ p{id++, 2.5 + U(g), U(g) * 0.5, 2.0 * U(g)}; pts.push_back(p); }          // ~30% outliers
        auto f = RansacLevelPoints(pts, cs, 0.02, P); CHECK(f.ok && std::abs(f.z - 0.0) < P.sigmaPt, "floor + furniture cluster + 30% outliers: floor height recovered within sigma_pt");
        int onFurn = 0; for (int i : f.inliers) onFurn += i >= 100 && i < 160; CHECK(onFurn == 0, "no furniture point is an inlier of the floor");
        auto h = RansacLevelPoints(pts, cs, 0.75, P); CHECK(!h.ok, "seeded at furniture height with 100 floor vs 60 furniture points the floor consensus wins and is rejected (0.75 m > 3 sigma_a from z_l)");
        std::vector<PointZ> noFloor(pts.begin() + 100, pts.end() - 70); auto n = RansacLevelPoints(noFloor, cs, 0.0, P); CHECK(!n.ok && n.inliers.empty(), "with no floor points no F2 is created");
        auto none = RansacLevelPoints(pts, std::vector<Contact>(), 0.0, P); CHECK(!none.ok, "no contacts -> no candidate points -> no F2");
    }
    std::cout << (failures ? "FAILURES: " : "all passed, failures = ") << failures << "\n";
    return failures ? 1 : 0;
}
