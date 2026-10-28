// P9-C5: unit tests of PedestrianLevels.h (standalone: g++ -std=c++17 -I../include test_levels.cc). Test parameters are arbitrary test values,
// not the calibrated constants of constants_c5.md.
#include <array>
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
    P.createMinContacts = 10; P.createMinTracks = 2; P.createMinExtentM = 1.0; P.edgeGuardS = 1.5;
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

    // ---- transitional (the C5 path-length slope rule, restored in P9-L Part 4)
    { auto c = Walk(1, 30, 1.2, 0.0, 0.0, 0, &g, 0.015); FlagTransitional(c, P); int n = 0; for (auto &x : c) n += x.transitional; CHECK(n == 0, "flat walk (noise 1.5 cm) is not flagged"); }
    { auto c = Walk(1, 30, 0.6, 0.0, 0.175 / 0.28, 0, &g, 0.015); FlagTransitional(c, P); int n = 0; for (auto &x : c) n += x.transitional; CHECK(n >= 27, "stairs (slope 0.625) are flagged"); }
    { auto c = Walk(1, 30, 0.0, 0.0, 0.0, 0, &g, 0.05); FlagTransitional(c, P); int n = 0; for (auto &x : c) n += x.transitional; CHECK(n == 0, "standing still (noisy heights) is not flagged"); }
    { auto c = Walk(1, 30, 0.2, 0.0, 0.0, 0, &g, 0.0); for (size_t i = 0; i < c.size(); ++i) c[i].h = 0.4 * (i % 2); FlagTransitional(c, P); int n = 0; for (auto &x : c) n += x.transitional; CHECK(n == 0, "slow shuffle (travel < 0.5 m per window) with jumping heights is not flagged");}

    // ---- P9-L step 2: foot-validity gate
    CHECK(FootContactValid(250.0, 200.0, 100.0, 0.08), "foot pixel 5 px above the mask bottom (root_v 200 + 0.55 * 100 = 255) is kept at tau 0.08 (0.05 maskH)");
    CHECK(!FootContactValid(240.0, 200.0, 100.0, 0.08), "foot pixel 15 px above the mask bottom (0.15 maskH) is rejected at tau 0.08");
    CHECK(FootContactValid(262.0, 200.0, 100.0, 0.08), "foot pixel below the mask bottom is kept");
    CHECK(FootContactValid(100.0, 200.0, 100.0, 0.0), "tau = 0 switches the gate off");
    CHECK(FootContactValid(100.0, 200.0, 0.0, 0.08), "a person without a mask height is not gated");

    // ---- P9-L step 4: epoch guard (reset between Add and WriteBack)
    { LevelStore st; LevelSet &S = st.Get(P); auto a = Walk(1, 12, 1.0, 0.0, 0, 0, &g, 0.01), b = Walk(2, 12, 1.0, 0.0, 0, 0, &g, 0.01); S.Update(a); S.Update(b);
      CHECK(st.Get(P).Levels().size() == 1, "epoch test setup: one level at z ~ 0");
      const unsigned long e0 = st.Epoch(); std::vector<std::pair<int, double>> solved = {{st.Get(P).Levels()[0].id, 0.07}}; long skipped = 0;
      CHECK(ApplyLevelHeights(st, e0, solved, P, &skipped) && std::abs(st.Get(P).Levels()[0].z - 0.07) < 1e-12 && skipped == 0, "same epoch: the solved height is written");
      // a BA reads the set (epoch e0), then the map is reset and a new set is built whose level 0 is at -1.3 m
      st.Reset(); LevelSet &N = st.Get(P); auto c = Walk(3, 12, 1.0, -1.3, 0, 0, &g, 0.01), d = Walk(4, 12, 1.0, -1.3, 0, 0, &g, 0.01); N.Update(c); N.Update(d);
      CHECK(st.Epoch() == e0 + 1 && N.Levels().size() == 1 && N.Levels()[0].id == 0, "reset advances the epoch; the rebuilt set restarts its ids at 0");
      const double zNew = N.Levels()[0].z; std::vector<std::pair<int, double>> stale = {{0, 0.5}};
      CHECK(!ApplyLevelHeights(st, e0, stale, P, &skipped) && skipped == 1 && std::abs(st.Get(P).Levels()[0].z - zNew) < 1e-12, "reset between Add and WriteBack: nothing is written into the new level 0, the skip is counted");
      std::vector<std::pair<int, double>> gone = {{7, 0.1}}; CHECK(ApplyLevelHeights(st, st.Epoch(), gone, P, &skipped) && skipped == 2, "same epoch but the level id was merged away: skipped and counted, no exception"); }

    // ---- P9-L step 5: founding contacts
    { LevelSet S(P); auto a = Walk(1, 12, 1.0, 0.0, 0, 0, &g, 0.01), b = Walk(2, 12, 1.0, 0.0, 0, 0, &g, 0.01); for (auto &x : a) { x.footU = 300 + x.kf; x.footV = 280; } S.Update(a); S.Update(b);
      CHECK(S.Levels().size() == 1 && S.Levels()[0].founding.size() >= 10, "a created level carries its founding contacts (>= 10)");
      int withPx = 0; for (auto &c : S.Levels()[0].founding) withPx += c.footV == 280; CHECK(withPx > 0, "founding contacts keep their contact pixel");
      std::set<int> tr; for (auto &c : S.Levels()[0].founding) tr.insert(c.track); CHECK(tr.size() >= 2, "the founding contacts come from >= 2 tracks"); }

    // ---- creation / association / merging
    { LevelSet S(P); auto a = Walk(1, 12, 1.0, 0.02, 0, 0, &g, 0.01); S.Update(a); CHECK(S.Levels().empty() && S.PoolSize() == 12, "12 contacts from one track create no level");
      auto b = Walk(2, 12, 1.0, 0.02, 0, 0, &g, 0.01); S.Update(b); CHECK(S.Levels().size() == 1 && std::abs(S.Levels()[0].z - 0.02) < 0.01, "contacts from 2 tracks create one level at the median");
      auto c = Walk(3, 5, 1.0, 0.04, 0, 0, &g, 0.005); S.Update(c); int as = 0; for (auto &x : c) as += x.level == 0; CHECK(as == 5, "new contacts within 3 sigma_a associate to the level");
      Contact far; far.track = 4; far.h = 0.5; far.t = 0; CHECK(S.Associate(far) == -1, "a contact 0.48 m away is unassigned"); }
    { LevelSet S(P); auto a = Walk(1, 12, 1.0, 0.0, 0, 0, &g, 0.01), b = Walk(2, 12, 1.0, 0.0, 0, 0, &g, 0.01), c = Walk(3, 12, 1.0, -0.8, 0, 0, &g, 0.01), d = Walk(4, 12, 1.0, -0.8, 0, 0, &g, 0.01);
      for (auto *v : {&a, &b, &c, &d}) S.Update(*v);
      CHECK(S.Levels().size() == 2, "two levels 0.8 m apart are both created"); S.SetHeight(S.Levels()[1].id, S.Levels()[0].z + 0.04); S.MergeClose();
      CHECK(S.Levels().size() == 1 && S.Merges() == 1, "two levels closer than 2 sigma_a merge"); }
    // ---- P9-H A2.3: CREATE / MERGE event records (logging only)
    { LevelSet S(P); auto a = Walk(1, 12, 1.0, 0.0, 0, 0, &g, 0.01), b = Walk(2, 12, 1.0, 0.0, 0, 0, &g, 0.01), c = Walk(3, 12, 1.0, -0.8, 0, 0, &g, 0.01), d = Walk(4, 12, 1.0, -0.8, 0, 0, &g, 0.01);
      for (auto *v : {&a, &b, &c, &d}) S.Update(*v);
      auto ev = S.TakeEvents();
      CHECK(ev.size() == 2 && !ev[0].merge && !ev[1].merge && ev[0].id == 0 && ev[1].id == 1 && ev[0].n >= 10 && std::abs(ev[1].z - ev[0].z + 0.8) < 0.05, "two CREATE events, ids 0 and 1, heights and contact counts recorded");
      CHECK(S.TakeEvents().empty(), "TakeEvents drains the record");
      const double z0 = S.Levels()[0].z; S.SetHeight(S.Levels()[1].id, z0 + 0.04); S.MergeClose();
      ev = S.TakeEvents();
      CHECK(ev.size() == 1 && ev[0].merge && ev[0].id == 0 && ev[0].otherId == 1 && std::abs(ev[0].z - z0) < 1e-12 && std::abs(ev[0].otherZ - (z0 + 0.04)) < 1e-12 && ev[0].n >= 10 && ev[0].otherN >= 10,
            "MERGE event: survivor = lower id, both heights and contact counts are the values before the merge"); }
    { LevelSet S(P); auto a = Walk(1, 12, 1.0, 0.0, 0, 0, &g, 0.01), b = Walk(2, 12, 1.0, 0.0, 0, 0, &g, 0.01); for (auto &x : a) x.transitional = true; for (auto &x : b) x.transitional = true;
      S.Update(a); S.Update(b); CHECK(S.Levels().empty() && S.PoolSize() == 0, "transitional contacts never enter the pool nor create a level"); }
    { LevelSet S(P); std::vector<Contact> none; S.Update(none); CHECK(S.Levels().empty(), "a furniture cluster (points only, no contacts) never becomes a level: the level set has no point input"); }

    { LevelSet S(P); auto a = Walk(1, 12, 1.0, 0.0, 0, 0, &g, 0.01), b = Walk(2, 12, 1.0, 0.0, 0, 0, &g, 0.01); S.Update(a); S.Update(b);
      const int n1 = S.Levels()[0].nContacts; auto a2 = a, b2 = b; S.Update(a2); S.Update(b2);
      CHECK(S.Levels().size() == 1 && S.Levels()[0].nContacts == n1, "overlapping BA windows: the same (keyframe, track) contacts are not counted twice");
      auto lone = Walk(7, 12, 1.0, 1.5, 0, 0, &g, 0.01); S.Update(lone); auto lone2 = lone; S.Update(lone2); CHECK(S.PoolSize() == 12, "overlapping windows: the same contacts are not pooled twice"); }

    // ---- c5_gate_revision.md: creation guard
    { LevelSet S(P); std::vector<Contact> cs; std::uniform_real_distribution<double> U(0, 0.28);
      for (int tr = 1; tr <= 2; ++tr) for (int i = 0; i < 12; ++i) { Contact c; c.track = tr; c.kf = i; c.t = i / 6.0; c.x = 9.0 + U(g); c.y = 0.1 * tr; c.h = -0.175 + 0.01 * (i % 3 - 1); cs.push_back(c); }
      S.Update(cs); CHECK(S.Levels().empty() && S.PoolSize() == 24, "a stair-tread cluster (24 contacts of 2 tracks, 0.28 m deep) never creates a level and stays pooled"); }
    { LevelSet S(P); auto a = Walk(1, 12, 0.3, 0.0, 0, 0, &g, 0.01), b = Walk(2, 12, 0.3, 0.0, 0, 0, &g, 0.01); S.Update(a); S.Update(b);
      CHECK(S.Levels().empty(), "12+12 contacts spanning 0.6 m (< 1.0 m) create no level"); auto c = Walk(1, 12, 1.0, 0.0, 0, 20, &g, 0.01); for (auto &x : c) x.kf += 100; S.Update(c);
      CHECK(S.Levels().size() == 1, "once the pooled contacts of the cluster span >= 1.0 m an ordinary level is created"); }
    { LevelSet S(P); std::vector<Contact> cs; auto w1 = Walk(1, 6, 1.0, 0.0, 0, 0, &g, 0.005); cs = w1;       // t 0 .. 0.83 s, flat
      for (int i = 0; i < 6; ++i) { Contact c; c.track = 1; c.kf = 50 + i; c.t = 3.0 + i / 6.0; c.x = 4 + i * 0.2; c.y = 0; c.h = -0.2 * i; c.transitional = true; cs.push_back(c); }   // flagged at t 3 .. 3.83
      Contact near; near.track = 1; near.kf = 70; near.t = 4.8; near.x = 5.5; near.y = 0; near.h = -1.0; cs.push_back(near);                             // 0.97 s after the last flagged contact -> guarded
      Contact far; far.track = 1; far.kf = 71; far.t = 6.0; far.x = 6.5; far.y = 0; far.h = -1.0; cs.push_back(far);                                     // 2.17 s after -> pooled
      S.Update(cs); CHECK(S.PoolSize() == 6 + 1, "edge guard: a contact within 1.5 s of a flagged contact of its track is not pooled (6 flat + 1 far pooled, the near one not)"); }
    for (double tf : {1.2, 1.8})
    { LevelSet S(P); auto a = Walk(1, 4, 1.0, 0.0, 0, 0, &g, 0.005); S.Update(a); CHECK(S.PoolSize() == 4, "edge guard setup: 4 contacts pooled (t = 0, 1/6, 1/3, 1/2 s)"); std::vector<Contact> cs = a; Contact f; f.track = 1; f.kf = 90; f.t = tf; f.x = 2; f.y = 0; f.h = -0.5; f.transitional = true; cs.push_back(f);
      S.Update(cs); const size_t expect = tf == 1.2 ? 0 : 2;   // 1.2 s: all four within 1.5 s; 1.8 s: only t = 1/3 and 1/2 s (1.47, 1.30 s)
      CHECK(S.PoolSize() == expect, std::string("edge guard is retrospective: flag at t = ") + std::to_string(tf) + " s leaves " + std::to_string(expect) + " pooled contacts (got " + std::to_string(S.PoolSize()) + ")"); }

    // ---- P9-L Part 4 step 1b: poster detector
    {
        PosterParams PP{3.0, 5.0, 1.5};
        auto feed = [&](PosterDetector &d, int track, double extent, double speedCam, double seconds, unsigned long epoch = 0, double range = 6.0)
        {   // 10 Hz states: the root wanders over `extent` m (triangle wave along x), the camera moves at speedCam m/s along x, `range` m away
            for (int i = 0; i <= static_cast<int>(seconds * 10); ++i)
            {
                const double t = i * 0.1, ph = std::fmod(t / seconds * 2.0, 2.0), w = ph < 1.0 ? ph : 2.0 - ph;
                d.Observe(track, epoch, 1000 + i, t, 5.0 + extent * w, 0.0, 1.0, speedCam * t, -range, 1.5);
            }
        };
        { PosterDetector d(PP); feed(d, 1, 0.8, 1.0, 4.0); CHECK(d.IsPoster(1), "a static root (0.8 m wander) seen for 4 s with the camera moving 4 m at 6 m range is a poster"); }
        { PosterDetector d(PP); feed(d, 1, 4.0, 1.0, 4.0); CHECK(!d.IsPoster(1), "a root that moves 4 m is not a poster"); }
        { PosterDetector d(PP); feed(d, 1, 0.5, 1.0, 2.0); CHECK(!d.IsPoster(1), "2 s are too short (min 3 s)"); }
        { PosterDetector d(PP); feed(d, 1, 0.5, 0.05, 5.0); CHECK(!d.IsPoster(1), "a static root with no camera parallax (camera moved 0.25 m at 6 m = 2.4 deg) is not rejected: a person standing still"); }
        { PosterDetector d(PP); feed(d, 1, 0.5, 1.0, 2.5, 0); feed(d, 1, 0.5, 1.0, 2.5, 1); CHECK(!d.IsPoster(1), "a map reset (new epoch) restarts the record: 2.5 s + 2.5 s do not add up"); }
        { PosterDetector d(PP); feed(d, 1, 0.8, 1.0, 4.0); feed(d, 1, 6.0, 1.0, 2.0, 1); CHECK(d.IsPoster(1), "the decision is latched for the sequence"); }
        { PosterDetector d(PP); feed(d, 1, 0.8, 1.0, 4.0); feed(d, 2, 3.0, 1.0, 4.0); CHECK(d.IsPoster(1) && !d.IsPoster(2), "decisions are per track"); }
    }

    // ---- P9-H A5 (gate K1): levels persist across epochs
    {
        auto setup = [&](LevelStore &st, double zLow) { // two levels: z ~ 0 and z ~ zLow, contacts of kf 0..29 spread over x
            LevelSet &S = st.Get(P);
            auto a = Walk(1, 30, 1.0, 0.0, 0, 0, &g, 0.01), b = Walk(2, 30, 1.0, 0.0, 0, 0, &g, 0.01), c = Walk(3, 30, 1.0, zLow, 0, 0, &g, 0.01), d = Walk(4, 30, 1.0, zLow, 0, 0, &g, 0.01);
            S.Update(a); S.Update(b); S.Update(c); S.Update(d);
            return &S;
        };
        std::array<double, 9> R; // Rz(0.5) Ry(0.02) Rx(-0.01): a yaw plus a small gravity tilt, as the IMU stages produce
        {
            const double cz = std::cos(0.5), sz = std::sin(0.5), cy = std::cos(0.02), sy = std::sin(0.02), cx = std::cos(-0.01), sx = std::sin(-0.01);
            const double Rz[9] = {cz, -sz, 0, sz, cz, 0, 0, 0, 1}, Ry[9] = {cy, 0, sy, 0, 1, 0, -sy, 0, cy}, Rx[9] = {1, 0, 0, 0, cx, -sx, 0, sx, cx};
            double T1[9] = {0}, T2[9] = {0};
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) for (int k = 0; k < 3; ++k) T1[3 * i + j] += Rz[3 * i + k] * Ry[3 * k + j];
            for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) for (int k = 0; k < 3; ++k) T2[3 * i + j] += T1[3 * i + k] * Rx[3 * k + j];
            for (int i = 0; i < 9; ++i) R[i] = T2[i];
        }
        const std::array<double, 3> t = {0.3, -0.2, 0.9};
        const double s = 1.17;
        { LevelStore st; LevelSet *S = setup(st, -1.0); CHECK(S->Levels().size() == 2, "carry setup: two levels");
          for (int i = 0; i < 6; ++i) { Contact u; u.track = 7; u.kf = 100 + i; u.t = i; u.x = 9 + i; u.y = 9; u.h = 5.0; std::vector<Contact> v{u}; S->Update(v); }   // unassigned contacts: a pool
          CHECK(S->PoolSize() > 0, "carry setup: the pool holds unassigned contacts");
          std::vector<std::array<double, 3>> before; for (const Level &L : S->Levels()) before.push_back({L.mx, L.my, L.z});
          S->TakeEvents();
          const auto rows = S->CarrySimilarity(R, t, s);
          CHECK(rows.size() == 2 && S->Levels().size() == 2 && S->PoolSize() == 0, "similarity carry keeps both levels, the pool is dropped");
          double worst = 0;
          for (size_t i = 0; i < 2; ++i)
          {
              const auto &b = before[i]; const Level &L = S->Levels()[i];
              const double ex = s * (R[0] * b[0] + R[1] * b[1] + R[2] * b[2]) + t[0], ey = s * (R[3] * b[0] + R[4] * b[1] + R[5] * b[2]) + t[1], ez = s * (R[6] * b[0] + R[7] * b[1] + R[8] * b[2]) + t[2];
              worst = std::max({worst, std::abs(L.mx - ex), std::abs(L.my - ey), std::abs(L.z - ez), std::abs(rows[i].zAfter - ez), std::abs(rows[i].zBefore - b[2])});
          }
          CHECK(worst < 1e-9, "carried (x, y, z) equals s R c + t within 1e-9 (worst " << worst << ")");
        }
        { LevelStore st; LevelSet *S = setup(st, -1.0); const int n0 = S->Levels()[0].nContacts, id0 = S->Levels()[0].id; S->TakeEvents();
          S->CarrySimilarity(R, t, s); CHECK(S->Levels()[0].nContacts == n0 && S->Levels()[0].id == id0, "a carried level keeps its id and contact count"); }
        { LevelStore st; LevelSet &S = st.Get(P); const auto rows = S.CarrySimilarity(R, t, s); const auto rows2 = S.CarryShift({{1, {0, 0, 1}}}); CHECK(rows.empty() && rows2.empty() && S.Levels().empty() && S.TakeEvents().empty(), "with no levels a carry changes nothing and logs nothing"); }
        { LevelStore st; LevelSet *S = setup(st, -0.30); CHECK(S->Levels().size() == 2, "merge-after-carry setup: two levels 0.30 m apart (> 2 sigma_a = 0.10)"); S->TakeEvents();
          const std::array<double, 9> I = {1, 0, 0, 0, 1, 0, 0, 0, 1}; S->CarrySimilarity(I, {0, 0, 0}, 0.2);   // the gap shrinks to 0.06 m
          const auto ev = S->TakeEvents(); CHECK(S->Levels().size() == 1 && ev.size() == 1 && ev[0].merge, "the merge rule is applied once after a carry (MERGE event, one level left)"); }
        { LevelStore st; LevelSet *S = setup(st, -1.0); S->TakeEvents(); std::vector<double> z0; for (const Level &L : S->Levels()) z0.push_back(L.z);
          std::map<unsigned long, std::array<double, 3>> delta; for (unsigned long kf = 0; kf < 30; ++kf) delta[kf] = {0.05, -0.02, 0.35 + (kf < 4 ? 1.5 : 0.0)};   // 4 of 30 anchors are outliers: the median is robust
          const auto rows = S->CarryShift(delta);
          CHECK(rows.size() == 2 && !rows[0].dropped && !rows[1].dropped && std::abs(S->Levels()[0].z - (z0[0] + 0.35)) < 0.01 && std::abs(S->Levels()[1].z - (z0[1] + 0.35)) < 0.01, "a loop correction with a known anchor shift of 0.35 m moves every level by 0.35 m (within 0.01 m)");
          CHECK(rows[0].nAnchors > 0 && rows[1].nAnchors > 0, "the anchors that survive are counted"); }
        { LevelStore st; LevelSet *S = setup(st, -1.0); S->TakeEvents(); std::map<unsigned long, std::array<double, 3>> delta = {{5000, {0, 0, 1.0}}};
          const auto rows = S->CarryShift(delta); CHECK(rows.size() == 2 && rows[0].dropped && rows[1].dropped && S->Levels().empty(), "levels whose anchors are all culled are dropped"); }
        { LevelStore st; LevelSet *S = setup(st, -1.0); const unsigned long e = st.Epoch(); st.Advance(); CHECK(st.Epoch() == e + 1 && st.Existing() == S && S->Levels().size() == 2, "Advance increments the epoch and keeps the set (Reset still destroys it)"); st.Reset(); CHECK(st.Existing() == nullptr && st.Epoch() == e + 2, "Reset destroys the set and increments the epoch"); }
    }

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
