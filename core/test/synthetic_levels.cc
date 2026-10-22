// P9-C5 step 4: synthetic gate of floor levels (F1 contact + F2 points on the level + F3 platform height), scenarios S1..S6 of task_C5.md / patch_stairs_ransac.md.
// Production edges (PedestrianFactors.h: EdgePedLevelPx, EdgePedPointOnLevel, EdgePedPlatformHeight, VertexFloorLevel, EdgePedLogRange with the bias vertex, EdgePedReproj,
// EdgePedWna) and the production level logic (PedestrianLevels.h: FlagTransitional, LevelSet, RansacLevelPoints), in the window pattern of LocalInertialBA (10 free states,
// older ones fixed, 10 LM iterations, estimates and the level set persisted between calls) as synthetic_floor_px.cc. Camera poses are exact and fixed (as in C3b/C3c).
//
//  walkers   4 tracks per seed, per-track bias b_true in {-5%, 0, +5%, 0} (stature H = H0 exp(b)), root 0.55 H above their floor; horizontal motion = an Ornstein-Uhlenbeck-like
//            walk around a region centre (spring + damping + white-noise acceleration, q = 0.1386), speed ~0.8 m/s. Feet: stride model of synthetic_floor_px.cc (anti-phase swing
//            0.30 m, lateral 0.12 m, keypoint height above the sole: stance 0.03 +- 0.015, swing 0.08 +- 0.05; mean of the two ~ delta_v 0.053); contact pixel = midpoint of the
//            two feet's lowest keypoints with (6.0, 3.0) px noise.
//  range     l_px = f cL H / z_c with log noise 0.031; the model carries b (prior sigma 0.0422), H0 = 1.70 fixed.
//  points    textured level: 3000 points on the floor, z = level + N(0, sigma_pt) (the map-point noise), uniformly over the walking regions; S5: + a furniture cluster at
//            +0.75 m (top of a table, 0.9 x 0.9 m, 120 points) next to the walkers, no feet on it. Points are fixed vertices (map points are not re-estimated here).
//  camera    at (cx, cy, level_cam + h_cam), h_cam = 1.4895 + N(0, 0.036) per keyframe (the diagnosed height scatter), looking +x pitched down by PITCH deg.
//  scenarios S1 one textured floor; S2 one textureless floor; S3 two levels (second 0.8 m below), walkers on both, lower textureless; S4 camera climbing 1.5 m over 300
//            keyframes along y, walkers on two levels (z 0 textured, z 1.5 textureless); S5 = S1 + furniture cluster; S6 two levels 2.8 m apart joined by stairs
//            (16 x 17.5 cm, 28 cm treads), two walkers on the stairs, two on the levels, camera on the upper level.
//  candidates none | levels | levels_platform
// usage: synthetic_levels SCEN(S1..S6|S0nofeet) CAND NKF NSEEDS SIGMA_A SIGMA_PT SMAX   -> one line of key=value pairs (and per-track rows with PER_TRACK=1)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <vector>

#include <Eigen/Sparse>

#include "PedestrianFactors.h"
#include "PedestrianLevels.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"

using namespace ORB_SLAM3;
using namespace ORB_SLAM3::ped;

static const double F = 415.69219381653056, CXP = 360.0, CYP = 240.0;
static const double CL = 0.855387, SIGB = 0.0422, QV = 0.1386, LOGSIG = 0.031, PXSIG = 1.5, H0 = 1.70;
static const double HCAM = 1.4895, HCAM_SCATTER = 0.036, DELTA_V = 0.053, CPX_U = 6.0, CPX_V = 3.0;
static const double kHuber2D = std::sqrt(5.991), kHuber1D = std::sqrt(3.841);
static const double DT = 0.1;
static const int NTRK = 4;
// keypoint height above the sole: stance 0.03, swing SWING_BASE + 0.05 |N|; the mean of the two feet = (0.03 + SWING_BASE + 0.05 * 0.7979) / 2 = delta_v = 0.053 (spec: the vertical
// offset matches delta_v; the C3c harness comment said 0.053 but its 0.08 + 0.05|N| swing gives 0.075)
static const double SWING_BASE = 2 * DELTA_V - 0.03 - 0.05 * 0.79788456;

struct Terrain
{
    // true floor height under (x, y); stairs: descending along +x from x0 over 16 treads of 0.28 m, risers 0.175 m
    bool stairs = false;
    double zUpper = 0, zLower = 0, x0 = 9.0;
    double zl2 = -1e9, xSplit = 1e9;       // S3 / S4: a second level for x >= xSplit (S3) -- S4 uses the walker's own level id instead
    double Floor(double x) const
    {
        if (!stairs)
            return x >= xSplit ? zl2 : zUpper;
        if (x <= x0)
            return zUpper;
        const int step = static_cast<int>(std::floor((x - x0) / 0.28));
        if (step >= 16)
            return zLower;
        return zUpper - 0.175 * (step + 1);
    }
    bool OnStairs(double x) const { return stairs && x > x0 && x < x0 + 16 * 0.28; }
};

struct KF
{
    Eigen::Matrix3d Rcw;
    Eigen::Vector3d tcw, cam;
    double t;
};

struct Obs
{
    bool vis = false;
    Eigen::Vector3d p;        // true root
    double u = 0, v = 0, lpx = 0, mu = 0, mv = 0;
    bool hasFoot = false;
    bool stairContact = false;
    double floorTrue = 0;
};

struct World
{
    std::vector<KF> kf;
    std::vector<std::vector<Obs>> obs; // [track][kf]
    std::vector<double> bTrue;
    std::vector<int> levelOf;           // S3/S4/S6: nominal level of the track (0 upper, 1 lower)
    std::vector<Eigen::Vector3d> pts;   // map points
    std::vector<double> levelsTrue;
};

static bool Project(const KF &k, const Eigen::Vector3d &Xw, Eigen::Vector2d &px, double &depth)
{
    const Eigen::Vector3d Xc = k.Rcw * Xw + k.tcw;
    if (Xc(2) < 1.0 || Xc(2) > 40.0)
        return false;
    px << F * Xc(0) / Xc(2) + CXP, F * Xc(1) / Xc(2) + CYP;
    depth = Xc(2);
    return px(0) > 0 && px(0) < 720 && px(1) > 0 && px(1) < 480;
}

static Eigen::Matrix3d CamRot(double pitchDeg)
{
    // camera x = world -y, camera y = down (pitched), camera z = forward (+x pitched down)
    const double a = pitchDeg * M_PI / 180.0;
    const Eigen::Vector3d z(std::cos(a), 0, -std::sin(a)), x(0, -1, 0), y = z.cross(x);
    Eigen::Matrix3d Rwc;
    Rwc.col(0) = x; Rwc.col(1) = y; Rwc.col(2) = z;
    return Rwc.transpose();
}

struct Scen
{
    std::string name;
    Terrain terrain;
    std::vector<Eigen::Vector2d> region;   // per walker centre
    std::vector<int> level;                // per walker: 0 upper, 1 lower
    std::vector<double> bTrue = {-0.05, 0.0, 0.05, 0.0};
    bool texturedUpper = true, texturedLower = false, furniture = false, noFeet = false, camClimb = false, stairWalkers = false;
    double pitch = 12.0;
    double lowerZ = -0.8;                  // S3: second level; S4: 1.5 (the upper); S6: -2.8
    std::vector<double> trueLevels;
};

static Scen MakeScen(const std::string &n)
{
    Scen s;
    s.name = n;
    s.region = {{5.5, -1.0}, {6.5, 1.0}, {7.5, -0.5}, {6.0, 0.5}};
    s.level = {0, 0, 0, 0};
    s.trueLevels = {0.0};
    if (n == "S2")
        s.texturedUpper = false;
    if (n == "S0nofeet")
        s.noFeet = true;
    if (n == "S5")
        s.furniture = true;
    if (n == "S3")
    {
        s.terrain.xSplit = 9.0;
        s.terrain.zl2 = -0.8;
        s.region = {{5.0, -1.0}, {6.5, 1.0}, {11.0, 1.0}, {12.0, -1.0}};
        s.level = {0, 0, 1, 1};
        s.bTrue = {-0.05, 0.05, 0.05, -0.05};
        s.trueLevels = {0.0, -0.8};
        s.texturedLower = false;
    }
    if (n == "S4")
    {
        s.camClimb = true;
        s.region = {{7.0, 1.0}, {8.0, 2.0}, {7.0, 5.0}, {8.0, 4.5}};   // levels A (0) at small y, B (1.5) at large y
        s.level = {0, 0, 1, 1};
        s.bTrue = {-0.05, 0.05, 0.05, -0.05};
        s.trueLevels = {0.0, 1.5};
        s.texturedLower = false;           // the upper level (B, z = 1.5) is the textureless one; "lower" below = level index 1
    }
    if (n == "S6")
    {
        s.terrain.stairs = true;
        s.terrain.zUpper = 0.0;
        s.terrain.zLower = -2.8;
        s.terrain.x0 = 9.0;
        s.stairWalkers = true;
        s.region = {{5.5, -1.0}, {6.5, 1.0}, {11.5, 0.0}, {11.5, 0.5}};   // walkers 2, 3 walk the stairs between x = 7.5 and x = 16
        s.level = {0, 0, 2, 2};
        s.bTrue = {-0.05, 0.05, 0.05, -0.05};
        s.trueLevels = {0.0, -2.8};
        s.texturedLower = false;
        s.pitch = 14.0;
    }
    return s;
}

static bool MakeWorld(const Scen &S, int seed, int nkf, double sigmaPt, World &W)
{
    std::mt19937 rng(seed * 7919 + 17);
    std::normal_distribution<double> N(0, 1);
    std::uniform_real_distribution<double> U01(0, 1);
    W = World();
    W.bTrue = S.bTrue;
    W.levelOf = S.level;
    W.levelsTrue = S.trueLevels;
    const Eigen::Matrix3d Rcw0 = CamRot(S.pitch);
    // walkers
    struct Wk { Eigen::Vector2d p, v; double hL = 0, hR = 0, phase; double stairPhase; };
    std::vector<Wk> wk(NTRK);
    for (int w = 0; w < NTRK; ++w)
    {
        wk[w].p = S.region[w] + Eigen::Vector2d(0.3 * N(rng), 0.3 * N(rng));
        wk[w].v = Eigen::Vector2d(0.2 * N(rng), (N(rng) > 0 ? 0.8 : -0.8));
        wk[w].phase = 6.28318 * U01(rng);
        wk[w].stairPhase = 6.28318 * U01(rng);
    }
    W.obs.assign(NTRK, std::vector<Obs>(nkf));
    for (int k = 0; k < nkf; ++k)
    {
        KF kf;
        kf.t = k * DT;
        const double camLevel = S.camClimb ? 1.5 * k / 300.0 : 0.0;
        const double camY = S.camClimb ? 0.02 * k : 0.0;
        const double hcam = HCAM + HCAM_SCATTER * N(rng);
        kf.cam = Eigen::Vector3d(0.0, camY, camLevel + hcam);
        kf.Rcw = Rcw0;
        kf.tcw = -kf.Rcw * kf.cam;
        for (int w = 0; w < NTRK; ++w)
        {
            Wk &a = wk[w];
            Eigen::Vector2d centre = S.region[w];
            if (S.stairWalkers && S.level[w] == 2)
                centre = Eigen::Vector2d(11.5 + 4.2 * std::sin(0.045 * kf.t * 2 * M_PI / 2.0 + a.stairPhase), 0.0 + 0.4 * w);   // sweeps x in [7.3, 15.7]: up, down, ...
            const double kS = S.stairWalkers && S.level[w] == 2 ? 0.0 : 0.25;
            Eigen::Vector2d acc = -kS * (a.p - centre) - 0.8 * a.v;
            if (S.stairWalkers && S.level[w] == 2)
            {
                // follow the sweeping centre at ~0.6 m/s: a PD controller on the centre
                acc = 4.0 * (centre - a.p) - 2.5 * a.v;
            }
            const double q = std::sqrt(QV * DT);
            a.v += DT * acc + q * Eigen::Vector2d(N(rng), N(rng));
            a.p += DT * a.v;
            Obs &o = W.obs[w][k];
            const double fl = S.camClimb ? (S.level[w] == 1 ? 1.5 : 0.0) : S.terrain.Floor(a.p(0));
            o.floorTrue = fl;
            o.stairContact = S.terrain.OnStairs(a.p(0));
            const double H = H0 * std::exp(S.bTrue[w]);
            o.p = Eigen::Vector3d(a.p(0), a.p(1), fl + 0.55 * H);
            Eigen::Vector2d px;
            double z;
            if (!Project(kf, o.p, px, z))
                continue;
            o.vis = true;
            const double range = (kf.Rcw * o.p + kf.tcw)(2);
            o.u = px(0) + PXSIG * N(rng);
            o.v = px(1) + PXSIG * N(rng);
            o.lpx = F * CL * H / range * std::exp(LOGSIG * N(rng));
            const Eigen::Vector2d dir = a.v.norm() > 0.2 ? Eigen::Vector2d(a.v.normalized()) : Eigen::Vector2d(0, 1);
            const Eigen::Vector2d lat(-dir(1), dir(0));
            a.phase += 2 * M_PI * 1.8 * DT;
            const double sw = 0.30 * std::sin(a.phase);
            const Eigen::Vector2d fL = a.p + dir * sw + lat * 0.12, fR = a.p - dir * sw - lat * 0.12;
            a.hL = 0.6 * a.hL + 0.8 * 0.015 * N(rng);
            a.hR = 0.6 * a.hR + 0.8 * 0.015 * N(rng);
            const bool leftSwing = std::cos(a.phase) > 0;
            const double zL = std::max(0.0, (leftSwing ? SWING_BASE + 0.05 * std::abs(N(rng)) : 0.03) + a.hL), zR = std::max(0.0, (leftSwing ? 0.03 : SWING_BASE + 0.05 * std::abs(N(rng))) + a.hR);
            Eigen::Vector2d pL, pR;
            double dzL, dzR;
            const double flL = S.camClimb ? fl : S.terrain.Floor(fL(0)), flR = S.camClimb ? fl : S.terrain.Floor(fR(0));
            const bool vL = Project(kf, Eigen::Vector3d(fL(0), fL(1), flL + zL), pL, dzL), vR = Project(kf, Eigen::Vector3d(fR(0), fR(1), flR + zR), pR, dzR);
            o.hasFoot = vL && vR && !S.noFeet;
            if (o.hasFoot)
            {
                const Eigen::Vector2d mid = 0.5 * (pL + pR);
                o.mu = mid(0) + CPX_U * N(rng);
                o.mv = mid(1) + CPX_V * N(rng);
            }
        }
        W.kf.push_back(kf);
    }
    // map points: floor points of the textured levels over the walking regions (+ furniture)
    std::uniform_real_distribution<double> ux(2.5, 17.0), uy(-3.0, 7.0);
    auto addFloor = [&](double zFloor, double x0, double x1, int n) {
        std::uniform_real_distribution<double> uxx(x0, x1);
        for (int j = 0; j < n; ++j)
            W.pts.emplace_back(uxx(rng), uy(rng), zFloor + sigmaPt * N(rng));
    };
    if (S.name == "S3")
    {
        if (S.texturedUpper) addFloor(0.0, 2.5, 9.0, 3000);
        if (S.texturedLower) addFloor(-0.8, 9.0, 17.0, 3000);
    }
    else if (S.name == "S4")
    {
        if (S.texturedUpper) addFloor(0.0, 2.5, 17.0, 3000);          // level A (z = 0) textured
        if (S.texturedLower) addFloor(1.5, 2.5, 17.0, 3000);
    }
    else if (S.name == "S6")
    {
        addFloor(0.0, 2.5, 9.0, 3000);                              // upper level textured
        // stairs treads and the lower floor are textureless / not floor points of a level: treads carry points at the tread heights (distractors for the RANSAC)
        for (int s = 0; s < 16; ++s)
            for (int j = 0; j < 20; ++j)
                W.pts.emplace_back(9.0 + 0.28 * s + 0.28 * U01(rng), uy(rng), -0.175 * (s + 1) + sigmaPt * N(rng));
    }
    else if (S.texturedUpper)
        addFloor(0.0, 2.5, 12.0, 3000);
    if (S.furniture)
        for (int j = 0; j < 120; ++j)
            W.pts.emplace_back(6.0 + 0.9 * U01(rng), 0.9 * U01(rng), 0.75 + 0.02 * N(rng));
    return true;
}

struct TrackStat
{
    std::vector<double> errPct;     // signed (range - truth) / truth, per keyframe i >= 10
    double bHat = 0;
};

int main(int argc, char **argv)
{
    if (argc < 8)
    {
        std::cerr << "usage: synthetic_levels SCEN CAND NKF NSEEDS SIGMA_A SIGMA_PT SMAX\n";
        return 2;
    }
    const std::string scen = argv[1], cand = argv[2];
    const int NKF = std::atoi(argv[3]), nseeds = std::atoi(argv[4]);
    const double sigmaA = std::atof(argv[5]), sigmaPt = std::atof(argv[6]), sMax = std::atof(argv[7]);
    const bool useLevels = cand == "levels" || cand == "levels_platform", usePlat = cand == "levels_platform";
    if (!(cand == "none" || useLevels))
    {
        std::cerr << "CAND must be none | levels | levels_platform\n";
        return 2;
    }
    const Scen S = MakeScen(scen);
    const LevelParams LP = DesignLevelParams(sigmaA, sigmaPt, sMax);
    const bool perTrack = std::getenv("PER_TRACK") != nullptr;

    std::vector<double> trackMedSigned, trackMedAbs, trackMedAbsPm5, trackMedAbsPm5Upper;
    long nStairC = 0, nStairFlag = 0, nLevelC = 0, nLevelFlag = 0;
    int seedsLevelAtStep = 0, seedsFurnitureLevel = 0, seedsAnyLevelAt = 0, nLevelsNoContactsCheck = 0;
    std::vector<double> levelErr;       // |z - nearest true| of every final level with >= 20 contacts
    long nLevelsTotal = 0, nLevelsBad = 0, nSeedsWithLevels = 0;
    std::vector<double> bHatAbs;
    std::map<int, std::vector<double>> byLenAbs;
    for (int seed = 1; seed <= nseeds; ++seed)
    {
        World W;
        MakeWorld(S, seed, NKF, sigmaPt, W);
        const int n = NKF;
        std::vector<std::vector<Vec6>> est(NTRK, std::vector<Vec6>(n, Vec6::Zero()));
        std::vector<std::vector<bool>> has(NTRK, std::vector<bool>(n, false));
        std::vector<double> bPers(NTRK, 0.0);
        LevelSet LS(LP);
        std::map<std::pair<int, int>, bool> flagLast; // (kf, track) -> transitional at the last call
        std::map<std::pair<int, int>, int> levelLast;
        for (int i = std::min(2, n - 1); i < n; ++i)
        {
            const int first = std::max(0, i - 200), firstFree = std::max(first, i - 9);
            g2o::SparseOptimizer opt;
            auto *linear = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
            auto *bs = new g2o::BlockSolverX(linear);
            auto *alg = new g2o::OptimizationAlgorithmLevenberg(bs);
            alg->setUserLambdaInit(1e0);
            opt.setAlgorithm(alg);
            opt.setVerbose(false);
            int id = 0;
            std::vector<g2o::VertexSE3Expmap *> poses;
            for (int k = first; k <= i; ++k)
            {
                auto *v = new g2o::VertexSE3Expmap();
                v->setId(id++);
                v->setEstimate(g2o::SE3Quat(W.kf[k].Rcw, W.kf[k].tcw));
                v->setFixed(true);
                opt.addVertex(v);
                poses.push_back(v);
            }
            std::vector<std::vector<VertexPedState *>> st(NTRK, std::vector<VertexPedState *>(n, nullptr));
            std::vector<VertexPedBias *> vb(NTRK, nullptr);
            for (int w = 0; w < NTRK; ++w)
            {
                bool any = false;
                for (int k = first; k <= i; ++k)
                    any = any || W.obs[w][k].vis;
                if (!any)
                    continue;
                if (cand != "none")
                {
                    vb[w] = new VertexPedBias();
                    vb[w]->setId(id++);
                    vb[w]->setEstimate(bPers[w]);
                    opt.addVertex(vb[w]);
                    auto *pr = new EdgePedBiasPrior();
                    pr->setVertex(0, vb[w]);
                    pr->setMeasurement(0.0);
                    pr->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (SIGB * SIGB));
                    opt.addEdge(pr);
                }
                int prev = -1;
                for (int k = first; k <= i; ++k)
                {
                    const Obs &o = W.obs[w][k];
                    if (!o.vis)
                        continue;
                    if (!has[w][k])
                    {
                        const Eigen::Vector3d b((o.u - CXP) / F, (o.v - CYP) / F, 1.0);
                        const double z = F * CL * H0 / o.lpx;
                        Vec6 x;
                        x << W.kf[k].Rcw.transpose() * (z * b - W.kf[k].tcw), Eigen::Vector3d::Zero();
                        est[w][k] = x;
                        has[w][k] = true;
                    }
                    auto *s = new VertexPedState();
                    s->setId(id++);
                    s->setEstimate(est[w][k]);
                    s->setFixed(k < firstFree);
                    opt.addVertex(s);
                    st[w][k] = s;
                    if (k >= firstFree)
                    {
                        auto *e = new EdgePedReproj<VertexPedState, g2o::VertexSE3Expmap>(F, F, CXP, CYP);
                        e->setVertex(0, s);
                        e->setVertex(1, poses[k - first]);
                        e->setMeasurement(Eigen::Vector2d(o.u, o.v));
                        e->setInformation(Eigen::Matrix2d::Identity() / (PXSIG * PXSIG));
                        auto *rk = new g2o::RobustKernelHuber();
                        rk->setDelta(kHuber2D);
                        e->setRobustKernel(rk);
                        opt.addEdge(e);
                    }
                    {
                        auto *e = new EdgePedLogRange<g2o::VertexSE3Expmap>(vb[w] != nullptr);
                        e->setVertex(0, s);
                        e->setVertex(1, poses[k - first]);
                        if (vb[w])
                            e->setVertex(2, vb[w]);
                        e->setMeasurement(std::log(F) + std::log(CL) + std::log(H0) - std::log(o.lpx));
                        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (LOGSIG * LOGSIG));
                        auto *rk = new g2o::RobustKernelHuber();
                        rk->setDelta(kHuber1D);
                        e->setRobustKernel(rk);
                        opt.addEdge(e);
                    }
                    if (prev >= 0 && k >= firstFree)
                    {
                        auto *e = new EdgePedWna();
                        e->setVertex(0, st[w][prev]);
                        e->setVertex(1, s);
                        e->setMeasurement(Vec6::Zero());
                        e->SetDt(W.kf[k].t - W.kf[prev].t, QV);
                        opt.addEdge(e);
                    }
                    prev = k;
                }
            }
            // ---------------------------------------------------------------- levels (the logic of PedestrianBA.cc, levels section)
            std::vector<VertexFloorLevel *> lvVert;
            if (useLevels)
            {
                std::vector<Contact> cs;
                std::vector<std::pair<int, int>> who; // (track, kf)
                for (int w = 0; w < NTRK; ++w)
                    for (int k = first; k <= i; ++k)
                    {
                        const Obs &o = W.obs[w][k];
                        if (!o.vis || !o.hasFoot || !st[w][k])
                            continue;
                        const KF &kf = W.kf[k];
                        const Eigen::Vector3d bray((o.mu - CXP) / F, (o.mv - CYP) / F, 1.0);
                        const Eigen::Vector3d dw = kf.Rcw.transpose() * bray;
                        const Eigen::Vector3d p0 = st[w][k]->p();
                        const double dh2 = dw(0) * dw(0) + dw(1) * dw(1);
                        if (dw(2) / dw.norm() > -0.02 || dh2 < 1e-12)
                            continue;
                        const double sr = (dw(0) * (p0(0) - kf.cam(0)) + dw(1) * (p0(1) - kf.cam(1))) / dh2;
                        if (!(sr > 0))
                            continue;
                        Contact c;
                        c.track = w;
                        c.kf = static_cast<unsigned long>(k);
                        c.t = kf.t;
                        c.x = p0(0);
                        c.y = p0(1);
                        c.h = kf.cam(2) + sr * dw(2) - DELTA_V;
                        cs.push_back(c);
                        who.push_back({w, k});
                    }
                FlagTransitional(cs, LP);
                LS.Update(cs);
                for (size_t c = 0; c < cs.size(); ++c)
                {
                    flagLast[{who[c].second, who[c].first}] = cs[c].transitional;
                    levelLast[{who[c].second, who[c].first}] = cs[c].level;
                }
                std::map<int, std::vector<size_t>> lc;
                for (size_t c = 0; c < cs.size(); ++c)
                    if (cs[c].level >= 0 && who[c].second >= firstFree)
                        lc[cs[c].level].push_back(c);
                std::map<int, VertexFloorLevel *> lv;
                for (const auto &kv : lc)
                {
                    double z0 = 0;
                    for (const auto &L : LS.Levels())
                        if (L.id == kv.first)
                            z0 = L.z;
                    auto *vl = new VertexFloorLevel();
                    vl->setId(id++);
                    vl->setEstimate(z0);
                    opt.addVertex(vl);
                    lv[kv.first] = vl;
                    for (size_t c : kv.second)
                    {
                        const int w = who[c].first, k = who[c].second;
                        const Obs &o = W.obs[w][k];
                        auto *f1 = new EdgePedLevelPx<g2o::VertexSE3Expmap>(F, F, CXP, CYP, o.mu, o.mv, DELTA_V);
                        f1->setVertex(0, st[w][k]);
                        f1->setVertex(1, poses[k - first]);
                        f1->setVertex(2, vl);
                        Eigen::Matrix2d I = Eigen::Matrix2d::Zero();
                        I(0, 0) = 1.0 / (CPX_U * CPX_U);
                        I(1, 1) = 1.0 / (CPX_V * CPX_V);
                        f1->setInformation(I);
                        auto *rk = new g2o::RobustKernelHuber();
                        rk->setDelta(kHuber2D);
                        f1->setRobustKernel(rk);
                        opt.addEdge(f1);
                    }
                    // F2
                    std::vector<PointZ> pz;
                    for (size_t j = 0; j < W.pts.size(); ++j)
                        pz.push_back({static_cast<int>(j), W.pts[j](0), W.pts[j](1), W.pts[j](2)});
                    std::vector<Contact> mine;
                    for (size_t c : kv.second)
                        mine.push_back(cs[c]);
                    const PlaneFit fit = RansacLevelPoints(pz, mine, z0, LP);
                    if (fit.ok)
                    {
                        std::vector<std::pair<double, int>> order;
                        for (int j : fit.inliers)
                        {
                            double dmin = 1e18;
                            for (const auto &m : mine)
                                dmin = std::min(dmin, std::hypot(W.pts[j](0) - m.x, W.pts[j](1) - m.y));
                            order.push_back({dmin, j});
                        }
                        std::sort(order.begin(), order.end());
                        if (order.size() > 50)
                            order.resize(50);
                        for (const auto &od : order)
                        {
                            auto *pv = new g2o::VertexSBAPointXYZ();
                            pv->setId(id++);
                            pv->setEstimate(W.pts[od.second]);
                            pv->setFixed(true);
                            opt.addVertex(pv);
                            auto *f2 = new EdgePedPointOnLevel();
                            f2->setVertex(0, pv);
                            f2->setVertex(1, vl);
                            f2->setMeasurement(0.0);
                            f2->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (sigmaPt * sigmaPt));
                            auto *rk = new g2o::RobustKernelHuber();
                            rk->setDelta(kHuber1D);
                            f2->setRobustKernel(rk);
                            opt.addEdge(f2);
                        }
                    }
                }
                if (usePlat)
                    for (int k = first; k <= i; ++k)
                    {
                        const double zc = W.kf[k].cam(2);
                        int best = -1;
                        double bd = 3.0 * sigmaA;
                        for (const auto &kv : lv)
                        {
                            const double d = std::abs(zc - kv.second->estimate() - HCAM);
                            if (d < bd) { bd = d; best = kv.first; }
                        }
                        if (best < 0)
                            continue;
                        auto *f3 = new EdgePedPlatformHeight<g2o::VertexSE3Expmap>();
                        f3->setVertex(0, poses[k - first]);
                        f3->setVertex(1, lv[best]);
                        f3->setMeasurement(HCAM);
                        f3->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (HCAM_SCATTER * HCAM_SCATTER));
                        auto *rk = new g2o::RobustKernelHuber();
                        rk->setDelta(kHuber1D);
                        f3->setRobustKernel(rk);
                        opt.addEdge(f3);
                    }
                for (const auto &kv : lv)
                    lvVert.push_back(kv.second);
                // keep id -> vertex for write-back
                opt.initializeOptimization();
                opt.optimize(10);
                for (const auto &kv : lv)
                    LS.SetHeight(kv.first, kv.second->estimate());
            }
            else
            {
                opt.initializeOptimization();
                opt.optimize(10);
            }
            for (int w = 0; w < NTRK; ++w)
            {
                for (int k = firstFree; k <= i; ++k)
                    if (st[w][k])
                        est[w][k] = st[w][k]->estimate();
                if (vb[w])
                    bPers[w] = vb[w]->estimate();
            }
            (void)lvVert;
        }
        // ---------------------------------------------------------------- per-seed metrics
        for (int w = 0; w < NTRK; ++w)
        {
            std::vector<double> sg;
            for (int k = std::min(10, n - 1); k < n; ++k)
            {
                const Obs &o = W.obs[w][k];
                if (!o.vis || !has[w][k] || o.stairContact)   // S6 criterion 8: the range error is judged on level-ground samples (stairs give no floor factor)
                    continue;
                const Eigen::Vector3d Xc = W.kf[k].Rcw * est[w][k].head<3>() + W.kf[k].tcw, Xt = W.kf[k].Rcw * o.p + W.kf[k].tcw;
                // S6 / S3 / S4 criteria 1-3 are judged on contacts on level ground; the range error is judged for all samples of the track in the scenarios other than S6
                sg.push_back((Xc.norm() - Xt.norm()) / Xt.norm() * 100.0);
            }
            if (sg.size() < 5)
                continue;
            std::vector<double> ab(sg);
            for (double &x : ab) x = std::abs(x);
            std::sort(sg.begin(), sg.end());
            std::sort(ab.begin(), ab.end());
            const double ms = sg[sg.size() / 2], ma = ab[ab.size() / 2];
            trackMedSigned.push_back(ms);
            trackMedAbs.push_back(ma);
            if (std::abs(W.bTrue[w]) > 0.04)
            {
                trackMedAbsPm5.push_back(ma);
                if (W.levelOf[w] == 0)
                    trackMedAbsPm5Upper.push_back(ma);
            }
            bHatAbs.push_back(std::abs(bPers[w]));
            if (perTrack)
                std::printf("TRACK seed=%d w=%d b_true=%+.2f b_hat=%+.4f med_signed_pct=%+.3f med_abs_pct=%.3f\n", seed, w, W.bTrue[w], bPers[w], ms, ma);
        }
        // levels at the end
        bool furnLevel = false, stepLevel = false;
        for (const auto &L : LS.Levels())
        {
            double dn = 1e9;
            for (double zt : W.levelsTrue)
                dn = std::min(dn, std::abs(L.z - zt));
            if (std::getenv("DEBUG_LEVELS"))
                std::printf("LEVEL seed=%d z=%+.4f nearest_true_err=%+.4f n=%d\n", seed, L.z, L.z - W.levelsTrue[std::abs(L.z - W.levelsTrue[0]) < std::abs(L.z - W.levelsTrue.back()) ? 0 : W.levelsTrue.size() - 1], L.nContacts);
            if (L.nContacts >= 20)
            {
                ++nLevelsTotal;
                levelErr.push_back(dn);
                if (dn >= 0.05)
                    ++nLevelsBad;
            }
            if (S.furniture && std::abs(L.z - 0.75) < 0.10)
                furnLevel = true;
            if (dn > 0.10)
                stepLevel = true;
        }
        if (!LS.Levels().empty())
            ++nSeedsWithLevels;
        seedsFurnitureLevel += furnLevel;
        seedsLevelAtStep += stepLevel;
        if (S.noFeet && !LS.Levels().empty())
            ++nLevelsNoContactsCheck;
        // S6 flag rates (the last call's flag of every contact)
        for (const auto &kv : flagLast)
        {
            const int k = kv.first.first, w = kv.first.second;
            const bool stair = W.obs[w][k].stairContact;
            (stair ? nStairC : nLevelC) += 1;
            if (kv.second)
                (stair ? nStairFlag : nLevelFlag) += 1;
        }
    }
    auto med = [](std::vector<double> v) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
    auto sd = [](const std::vector<double> &v) { if (v.size() < 2) return 0.0; double m = 0; for (double x : v) m += x; m /= v.size(); double s = 0; for (double x : v) s += (x - m) * (x - m); return std::sqrt(s / (v.size() - 1)); };
    std::sort(levelErr.begin(), levelErr.end());
    std::printf("scen=%s cand=%s NKF=%d seeds=%d tracks=%zu spread_sd_pct=%.4f median_abs_pct_all=%.4f median_abs_pct_pm5=%.4f median_abs_pct_pm5_upper=%.4f median_signed_pct=%.4f "
                "levels_ge20=%ld levels_bad_5cm=%ld max_level_err_m=%.4f seeds_with_levels=%ld seeds_furniture_level=%d seeds_level_off_truth_10cm=%d "
                "stair_contacts=%ld stair_flagged=%ld level_contacts=%ld level_flagged=%ld median_abs_bhat=%.4f seeds_levels_when_nofeet=%d\n",
                scen.c_str(), cand.c_str(), NKF, nseeds, trackMedSigned.size(), sd(trackMedSigned), med(trackMedAbs), med(trackMedAbsPm5), med(trackMedAbsPm5Upper), med(trackMedSigned), nLevelsTotal,
                nLevelsBad, levelErr.empty() ? 0.0 : levelErr.back(), nSeedsWithLevels, seedsFurnitureLevel, seedsLevelAtStep, nStairC, nStairFlag, nLevelC, nLevelFlag, med(bHatAbs), nLevelsNoContactsCheck);
    return 0;
}
