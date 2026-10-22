// P9-C4 step 3: synthetic check of the prediction-based innovation gate with the PRODUCTION code: the gate itself (PedestrianGate.h: InnovationTest, GateStep)
// and the production edges (EdgePedReproj, EdgePedLogRange, EdgePedWna with the active prior), in the window pattern of the C3b harness
// (synthetic_floor_contact.cc): a window of 10 free states, older ones fixed, 10 LM iterations, state estimates persisted between calls; camera poses exact and fixed.
// Per new keyframe measurement: gate decision against the persisted state of the last accepted keyframe, then the window solve over the kept measurements
// (a rejected measurement creates no state and no factor; a re-initialisation breaks the WNA chain).
//   walker: root at 0.55 H, white-noise acceleration q (x, y) exact discretisation (z constant), camera ~7 m behind it, 1.5 m above the floor.
//   measurement: root pixel (1.5 px), l_px = f cL H / z with log noise LOGSIG_DATA (0.031 = the model's R by default).
// scenarios (a seed = one world): clean | idswitch | outliers
//   clean     known negative: rejection share <= 2 %, 0 re-initialisations
//   idswitch  known positive: at keyframe K0 the track's measurements come from a second walker 3 m (horizontal) from the first; re-initialised within 4 keyframes (K0..K0+3)
//   outliers  5 % of the measurements from keyframe index 2 on (past the free keyframes) have l_px scaled so that the range is off by +-30 %; share of them rejected
// P9-C4 II: the gate carries the covariance scales s_k (k = 3, 4, 5, >=6; argument "s3,s4,s5,s6", default 1,1,1,1) and the data can carry the real-data log-range noise:
//   logsigData = within-track sigma of an AR(1) (correlation time tauC seconds, phi = exp(-dt/tauC)), plus a per-track constant bias ~ N(0, biasSigma^2) (the graph has no bias state in "none").
// usage: synthetic_gate SCENARIO PRIOR(iso|aniso) NSEEDS [dt=0.2 nkf=60 logsigData=0.031 p0scale=1 scales=1,1,1,1 tauC=0 biasSigma=0]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "PedestrianFactors.h"
#include "PedestrianGate.h"
#include "Pedestrians.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"

using namespace ORB_SLAM3;
using namespace ORB_SLAM3::ped;

static const double F = 415.69219381653056, CXP = 360.0, CYP = 240.0, CL = 0.855387, H0 = 1.70, PXSIG = 1.5, QXY = 0.1386;
static const double kHuber2D = std::sqrt(5.991), kHuber1D = std::sqrt(3.841);

static Eigen::Matrix3d LookAt(const Eigen::Vector3d &c, const Eigen::Vector3d &target)
{
    Eigen::Vector3d z = (target - c).normalized();
    Eigen::Vector3d x = z.cross(Eigen::Vector3d::UnitZ()).normalized();
    Eigen::Vector3d y = z.cross(x);
    Eigen::Matrix3d Rwc;
    Rwc.col(0) = x; Rwc.col(1) = y; Rwc.col(2) = z;
    return Rwc.transpose();
}

struct Walk
{
    std::vector<Eigen::Vector3d> p;
};

static Walk MakeWalk(std::mt19937 &rng, int n, double dt, const Eigen::Vector3d &start)
{
    std::normal_distribution<double> N(0, 1);
    const double dt3 = dt * dt * dt / 3, dt2 = dt * dt / 2;
    Eigen::Matrix2d Q;
    Q << dt3, dt2, dt2, dt;
    Q *= QXY;
    const Eigen::Matrix2d L = Eigen::LLT<Eigen::Matrix2d>(Q + 1e-15 * Eigen::Matrix2d::Identity()).matrixL();
    Eigen::Vector3d p = start, v(0.2 * N(rng), (N(rng) > 0 ? 1.0 : -1.0) + 0.2 * N(rng), 0);
    Walk w;
    for (int k = 0; k < n; ++k)
    {
        w.p.push_back(p);
        for (int a = 0; a < 2; ++a)
        {
            const Eigen::Vector2d wn = L * Eigen::Vector2d(N(rng), N(rng));
            const double pa = p(a) + dt * v(a) + wn(0), va = v(a) + wn(1);
            p(a) = pa;
            v(a) = va;
        }
    }
    return w;
}

struct Meas
{
    Eigen::Matrix3d Rcw;
    Eigen::Vector3d tcw;
    double u, v, lpx;
    bool injected = false;
};

static bool Visible(const Eigen::Matrix3d &Rcw, const Eigen::Vector3d &tcw, const Eigen::Vector3d &X, double &z)
{
    const Eigen::Vector3d Xc = Rcw * X + tcw;
    z = Xc(2);
    const double u = F * Xc(0) / Xc(2) + CXP, v = F * Xc(1) / Xc(2) + CYP;
    return Xc(2) > 1.5 && Xc(2) < 40 && u > 0 && u < 720 && v > 0 && v < 480;
}

struct Entry
{
    int k;          // keyframe index
    bool reinit;
};

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        std::cerr << "usage: synthetic_gate SCENARIO(clean|idswitch|outliers) PRIOR(iso|aniso) NSEEDS [dt nkf logsigData p0scale]\n";
        return 2;
    }
    const std::string scen = argv[1], prior = argv[2];
    const int nseeds = std::atoi(argv[3]);
    const double dt = argc > 4 ? std::atof(argv[4]) : 0.2;
    const int NKF = argc > 5 ? std::atoi(argv[5]) : 60;
    const double logsigData = argc > 6 ? std::atof(argv[6]) : 0.031;
    const double p0s = argc > 7 ? std::atof(argv[7]) : 1.0;
    double sc[4] = {1, 1, 1, 1};
    if (argc > 8)
        std::sscanf(argv[8], "%lf,%lf,%lf,%lf", &sc[0], &sc[1], &sc[2], &sc[3]);
    const double tauC = argc > 9 ? std::atof(argv[9]) : 0.0, biasSigma = argc > 10 ? std::atof(argv[10]) : 0.0;
    PedestrianParams P;
    P.rootPxSigma = PXSIG;
    P.logDepthSigma = 0.031;
    P.qv = QXY;
    if (prior == "aniso")
    {
        P.motionPrior = PedestrianParams::MotionPrior::Anisotropic;
        P.qvH = 0.1383;
        P.qvZ = 0.0096;
    }
    P.innovChi2Dof2 = 9.21034;
    P.innovChi2Dof3 = 11.34487;
    P.innovFreeKeyframes = 2;
    P.innovReinitRejections = 3;
    P.innovP0PosXY = 0.2044 * p0s;
    P.innovP0PosZ = 0.0586 * p0s;
    P.innovP0VelXY = 0.1851 * p0s;
    P.innovP0VelZ = 0.0752 * p0s;
    for (int q = 0; q < 4; ++q)
        P.innovScale[q] = sc[q];
    const int K0 = 30;
    long gated = 0, rejected = 0, reinits = 0, seedsWithReinit = 0, nInj = 0, nInjRej = 0, nInjGateable = 0, nInjGateableRej = 0, seedsOk = 0, seedsReinitIn4 = 0;
    std::vector<int> reinitDelay;
    for (int seed = 1; seed <= nseeds; ++seed)
    {
        std::mt19937 rng(seed * 7919 + 17);
        std::normal_distribution<double> N(0, 1);
        std::uniform_real_distribution<double> Uni(0, 1);
        std::vector<Meas> M;
        bool ok = false;
        for (int attempt = 0; attempt < 500 && !ok; ++attempt)
        {
            M.clear();
            const double Ht = H0;
            Walk w1 = MakeWalk(rng, NKF, dt, Eigen::Vector3d(8.0, N(rng), 0.55 * Ht));
            Walk w2;
            if (scen == "idswitch")
            {
                const double ang = 2 * M_PI * Uni(rng);
                w2 = MakeWalk(rng, NKF, dt, w1.p[K0] + 3.0 * Eigen::Vector3d(std::cos(ang), std::sin(ang), 0));
                // the second walker is the track from K0 on: its start is 3 m (horizontal) from the first walker's position at K0
            }
            ok = true;
            double ar = logsigData * N(rng); // stationary start
            const double phi = tauC > 0 ? std::exp(-dt / tauC) : 0.0;
            const double bTrack = biasSigma * N(rng);
            for (int k = 0; k < NKF && ok; ++k)
            {
                const double th = 0.35 * std::sin(2 * M_PI * k * dt / 20.0), dist = 7.0;
                const Eigen::Vector3d c = w1.p[k] + Eigen::Vector3d(-dist * std::cos(th), -dist * std::sin(th), 1.5 - w1.p[k](2));
                Meas m;
                m.Rcw = LookAt(c, w1.p[k]);
                m.tcw = -m.Rcw * c;
                const Eigen::Vector3d X = (scen == "idswitch" && k >= K0) ? w2.p[k - K0 + 0] : w1.p[k];
                double z;
                if (!Visible(m.Rcw, m.tcw, X, z) || (X - c).norm() < 2.0)
                {
                    ok = false;
                    break;
                }
                const Eigen::Vector3d Xc = m.Rcw * X + m.tcw;
                m.u = F * Xc(0) / Xc(2) + CXP + PXSIG * N(rng);
                m.v = F * Xc(1) / Xc(2) + CYP + PXSIG * N(rng);
                ar = phi * ar + std::sqrt(1 - phi * phi) * logsigData * N(rng); // phi = 0: white noise of sigma logsigData
                m.lpx = F * CL * Ht / Xc(2) * std::exp(ar + bTrack);
                if (scen == "outliers" && k >= P.innovFreeKeyframes && Uni(rng) < 0.05)
                {
                    m.lpx /= (Uni(rng) < 0.5 ? 1.3 : 0.7); // range x1.3 or x0.7
                    m.injected = true;
                }
                M.push_back(m);
            }
        }
        if (!ok)
        {
            std::cerr << "no admissible world for seed " << seed << "\n";
            return 3;
        }
        // ---- run: gate + window solve
        std::vector<Entry> kept;
        std::vector<Vec6> est(NKF, Vec6::Zero());
        std::vector<bool> has(NKF, false);
        GateChain ch;
        int lastAcc = -1;
        int firstReinit = -1;
        long seedRej = 0, seedGated = 0, seedReinit = 0;
        for (int i = 0; i < NKF; ++i)
        {
            const Meas &m = M[i];
            const double logz = std::log(F) + std::log(CL) + std::log(H0) - std::log(m.lpx);
            GateTest t;
            bool havePred = lastAcc >= 0 && has[lastAcc];
            if (havePred)
                t = InnovationTest(P, est[lastAcc].head<3>(), est[lastAcc].tail<3>(), (i - lastAcc) * dt, m.Rcw, m.tcw, F, F, CXP, CYP, m.u, m.v, true, logz, P.innovScale[std::min(std::max(ch.idx + 1, 3), 6) - 3]);
            const PedGateRec r = GateStep(P, ch, havePred, t, (i - lastAcc) * dt, 0);
            if (r.tested)
            {
                ++seedGated;
                if (r.status != PedGateRec::ACCEPTED)
                    ++seedRej;
            }
            if (m.injected)
            {
                ++nInj;
                if (r.status == PedGateRec::REJECTED)
                    ++nInjRej;
                if (r.tested)
                {
                    ++nInjGateable;
                    if (r.status == PedGateRec::REJECTED)
                        ++nInjGateableRej;
                }
            }
            if (r.status == PedGateRec::REINIT)
            {
                ++seedReinit;
                if (firstReinit < 0)
                    firstReinit = i;
            }
            if (r.status == PedGateRec::REJECTED)
                continue;
            kept.push_back({i, r.status == PedGateRec::REINIT});
            lastAcc = i;
            // window solve over the kept entries: last 10 free, older fixed (up to 200 back)
            const int n = static_cast<int>(kept.size());
            const int first = std::max(0, n - 200), firstFree = std::max(first, n - 10);
            g2o::SparseOptimizer opt;
            auto *linear = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
            auto *bs = new g2o::BlockSolverX(linear);
            auto *alg = new g2o::OptimizationAlgorithmLevenberg(bs);
            alg->setUserLambdaInit(1e0);
            opt.setAlgorithm(alg);
            opt.setVerbose(false);
            int id = 0;
            std::vector<g2o::VertexSE3Expmap *> poses;
            std::vector<VertexPedState *> st;
            for (int q = first; q < n; ++q)
            {
                const Meas &mq = M[kept[q].k];
                auto *vp = new g2o::VertexSE3Expmap();
                vp->setId(id++);
                vp->setEstimate(g2o::SE3Quat(mq.Rcw, mq.tcw));
                vp->setFixed(true);
                opt.addVertex(vp);
                poses.push_back(vp);
            }
            for (int q = first; q < n; ++q)
            {
                const int k = kept[q].k;
                if (!has[k])
                {
                    const Eigen::Vector3d b((M[k].u - CXP) / F, (M[k].v - CYP) / F, 1.0);
                    const double z = F * CL * H0 / M[k].lpx;
                    Vec6 x;
                    x << M[k].Rcw.transpose() * (z * b - M[k].tcw), Eigen::Vector3d::Zero();
                    est[k] = x;
                    has[k] = true;
                }
                auto *s = new VertexPedState();
                s->setId(id++);
                s->setEstimate(est[k]);
                s->setFixed(q < firstFree);
                opt.addVertex(s);
                st.push_back(s);
            }
            for (int q = firstFree; q < n; ++q)
            {
                const int k = kept[q].k;
                auto *e = new EdgePedReproj<VertexPedState, g2o::VertexSE3Expmap>(F, F, CXP, CYP);
                e->setVertex(0, st[q - first]);
                e->setVertex(1, poses[q - first]);
                e->setMeasurement(Eigen::Vector2d(M[k].u, M[k].v));
                e->setInformation(Eigen::Matrix2d::Identity() / (PXSIG * PXSIG));
                auto *rk = new g2o::RobustKernelHuber();
                rk->setDelta(kHuber2D);
                e->setRobustKernel(rk);
                opt.addEdge(e);
                auto *er = new EdgePedLogRange<g2o::VertexSE3Expmap>(false);
                er->setVertex(0, st[q - first]);
                er->setVertex(1, poses[q - first]);
                er->setMeasurement(std::log(F) + std::log(CL) + std::log(H0) - std::log(M[k].lpx));
                er->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (0.031 * 0.031));
                auto *rk1 = new g2o::RobustKernelHuber();
                rk1->setDelta(kHuber1D);
                er->setRobustKernel(rk1);
                opt.addEdge(er);
            }
            for (int q = first + 1; q < n; ++q)
            {
                if (q < firstFree && q - 1 < firstFree)
                    continue;
                if (kept[q].reinit)
                    continue;
                auto *e = new EdgePedWna();
                e->setVertex(0, st[q - 1 - first]);
                e->setVertex(1, st[q - first]);
                e->setMeasurement(Vec6::Zero());
                e->SetDt((kept[q].k - kept[q - 1].k) * dt, ActiveQ(P));
                opt.addEdge(e);
            }
            opt.initializeOptimization();
            opt.optimize(10);
            for (int q = firstFree; q < n; ++q)
                est[kept[q].k] = st[q - first]->estimate();
        }
        gated += seedGated;
        rejected += seedRej;
        reinits += seedReinit;
        if (seedReinit > 0)
            ++seedsWithReinit;
        if (scen == "idswitch")
        {
            const bool in4 = firstReinit >= K0 && firstReinit <= K0 + 3;
            seedsReinitIn4 += in4;
            if (firstReinit >= 0)
                reinitDelay.push_back(firstReinit - K0);
        }
        (void)seedsOk;
    }
    std::printf("scenario %s prior %s seeds %d dt %.2f nkf %d logsigData %.4f p0scale %.2f scales %.4g,%.4g,%.4g,%.4g tauC %.2f biasSigma %.4f\n", scen.c_str(), prior.c_str(), nseeds, dt, NKF, logsigData, p0s, sc[0], sc[1], sc[2], sc[3], tauC, biasSigma);
    if (scen == "clean")
    {
        const double share = gated ? static_cast<double>(rejected) / gated : 0.0;
        std::printf("tested measurements %ld, rejected(+reinit) %ld = %.3f %%, re-initialised tracks %ld (seeds with one: %ld)\n", gated, rejected, 100 * share, reinits, seedsWithReinit);
        const bool pass = share <= 0.02 && reinits == 0;
        std::printf("CHECK clean (<= 2 %% rejected, 0 re-initialisations): %s\n", pass ? "PASS" : "FAIL");
        return pass ? 0 : 1;
    }
    if (scen == "idswitch")
    {
        std::printf("re-initialised within 4 keyframes of the switch: %ld / %d = %.1f %%; delays (keyframes after K0): ", seedsReinitIn4, nseeds, 100.0 * seedsReinitIn4 / nseeds);
        for (int d : reinitDelay)
            std::printf("%d ", d);
        std::printf("\n");
        const bool pass = seedsReinitIn4 >= 0.9 * nseeds;
        std::printf("CHECK idswitch (>= 90 %% re-initialised within 4 keyframes): %s\n", pass ? "PASS" : "FAIL");
        return pass ? 0 : 1;
    }
    const double share = nInj ? static_cast<double>(nInjRej) / nInj : 0.0;
    std::printf("injected outliers %ld (all past the free keyframes), rejected %ld = %.1f %%; among those the gate tested %ld, rejected %ld; clean-measurement rejection %ld of %ld tested\n", nInj, nInjRej, 100 * share,
                nInjGateable, nInjGateableRej, rejected, gated);
    const bool pass = share >= 0.80;
    std::printf("CHECK outliers (>= 80 %% of the injected rejected): %s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
