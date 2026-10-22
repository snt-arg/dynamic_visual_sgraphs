// P9-C3c step 3: synthetic gate of the midpoint floor contact (A' + b, B' + b) WITH the diagnosed errors, and the validity check that the old range-space A + b / B + b fail under
// the same errors. Production edges (PedestrianFactors.h), production gates (PedestrianFloorGate.h) and the production helper PedestrianBA::RegionalFloor, in the window pattern
// of LocalInertialBA (10 free states, older ones fixed, 10 LM iterations, estimates persisted), as synthetic_floor_contact.cc.
//   walker: white-noise acceleration q = 0.1386, root height 0.55 H, stature H = H0 exp(b_true); camera follows at ~7 m, 1.4 m above a flat floor z = 0 (exact poses, fixed).
//   range measurement: l_px = f cL H / z_c with log noise 0.031 (the model's R); H0 = 1.70 is fixed in every model; modes with +b carry the per-track bias vertex (prior sigma 0.0422).
//   feet (diagnosed error 4 and the stride): left / right foot = root_xy +- (fore-aft swing A sin(phase), lateral +-0.12 m) in the walking direction, anti-phase; keypoint height above the sole:
//     stance foot 0.03 +- 0.015, swing foot 0.08 +- 0.05 (mean of the two ~ 0.053 = delta_v); the lowest keypoint of a foot is what the detector returns.
//   old modes (none / A / B): contact = the lower foot in the image (largest v), range-space residual EdgePedFloorContact, delta = 0.036, sigma radial / tangential 0.358 / 0.2755 m,
//     Variant A z_f from the regional estimator, B from the k = 5 nearest live points (band 0.15 m, radius 1 m), no gates but the 1.1 deg grazing guard -- as C3b.
//   new modes (Ap / Bp): contact = midpoint of the two feet's lowest keypoints (pixel noise (6.0, 3.0) px added), EdgePedFloorPx, delta_v = 0.053, z_f corrected by -ZFBIAS_CAL,
//     gates: raised-surface (z_f - global floor > 0.10), elevation >= MINELEV, Bp: >= 5 qualifying points within +-0.05 m of the corrected z_f and 1 m, scatter < 0.02 m.
//   injected z_f errors: z_f_used = 0 + N(ZFBIAS, ZFSIG^2) (bias + noise) for every contact; a raised-surface tail: with probability RAISED_P (0.10 A-type, 0.47 B-type) a contact at range > 10 m
//     gets z_f raised by a height drawn from RAISED_H (office data, argument list); static points of B' carry z noise 0.02 m.
// usage: synthetic_floor_px MODE(none|A|B|Ap|Bp) B_TRUE NKF NSEEDS TAIL(A|B) MINELEV_DEG [feet=1 raisedHeights=0.2,0.35,0.5,0.8 zfbiasCal=0.02]
// prints (tab separated): median |range error| [m], [%], median signed [%], median |b_hat| at 30 / 100 / 300, edges
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <random>
#include <sstream>
#include <vector>

#include <Eigen/Sparse>

#include "PedestrianBA.h"
#include "PedestrianFactors.h"
#include "PedestrianFloorGate.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"

using namespace ORB_SLAM3;
using namespace ORB_SLAM3::ped;

static const double F = 415.69219381653056, CXP = 360.0, CYP = 240.0;
static const double CL = 0.855387, SIGB = 0.0422, QV = 0.1386, LOGSIG = 0.031, PXSIG = 1.5, H0 = 1.70, CAMH = 1.4;
static const double kHuber2D = std::sqrt(5.991), kHuber1D = std::sqrt(3.841);
static const double ZFBIAS = 0.02, ZFSIG = 0.02;                 // injected regional z_f error (diagnosed: +2 cm, sigma 2 cm)
static const double CPX_U = 6.0, CPX_V = 3.0;                   // contact pixel noise (5b rule)
static const double POINT_Z_SIGMA_B = 0.02, POINT_PX_SIGMA = 0.5;
static const int NPOINTS = 6000;

struct KF
{
    double t;
    Eigen::Matrix3d Rcw;
    Eigen::Vector3d tcw, p;
    double u, v, lpx;
    double fu, fv;    // lowest-foot pixel (old modes), no extra pixel noise beyond PXSIG
    double mu, mv;    // midpoint pixel with (6, 3) px noise (new modes)
    bool hasFoot;     // both feet visible
    double zfErr;     // injected z_f error for this contact (bias + noise [+ raised tail])
    bool raised;
};

struct World
{
    std::vector<KF> kf;
    std::vector<Eigen::Vector3d> pts;
    std::vector<std::vector<std::pair<int, Eigen::Vector2d>>> obs;
};

static Eigen::Matrix3d LookAt(const Eigen::Vector3d &c, const Eigen::Vector3d &target)
{
    Eigen::Vector3d z = (target - c).normalized();
    Eigen::Vector3d x = z.cross(Eigen::Vector3d::UnitZ()).normalized();
    Eigen::Vector3d y = z.cross(x);
    Eigen::Matrix3d Rwc;
    Rwc.col(0) = x; Rwc.col(1) = y; Rwc.col(2) = z;
    return Rwc.transpose();
}

static bool Project(const KF &k, const Eigen::Vector3d &Xw, Eigen::Vector2d &px, double &depth)
{
    const Eigen::Vector3d Xc = k.Rcw * Xw + k.tcw;
    if (Xc(2) < 1.0 || Xc(2) > 40.0)
        return false;
    px << F * Xc(0) / Xc(2) + CXP, F * Xc(1) / Xc(2) + CYP;
    depth = Xc(2);
    return px(0) > 0 && px(0) < 720 && px(1) > 0 && px(1) < 480;
}

static std::vector<double> gRaisedH = {0.2, 0.35, 0.5, 0.8};
static double gRaisedP = 0.10;
static bool gB = false;       // point z noise 0.02 (B') vs 0.05 (old B, as C3b)
static bool MakeWorld(int seed, double Htrue, World &W, int n, double dt = 0.1)
{
    std::mt19937 rng(seed * 7919 + 13);
    std::normal_distribution<double> N(0, 1);
    std::uniform_real_distribution<double> U01(0, 1);
    for (int attempt = 0; attempt < 3000; ++attempt)
    {
        W = World();
        const double dt3 = dt * dt * dt / 3, dt2 = dt * dt / 2;
        Eigen::Matrix2d Q;
        Q << dt3, dt2, dt2, dt;
        Q *= QV;
        const Eigen::Matrix2d L = Eigen::LLT<Eigen::Matrix2d>(Q + 1e-15 * Eigen::Matrix2d::Identity()).matrixL();
        Eigen::Vector3d p(8.0, N(rng), 0.55 * Htrue), v(0, (N(rng) > 0 ? 1.0 : -1.0), 0);
        v(0) += 0.2 * N(rng);
        v(1) += 0.2 * N(rng);
        double hL = 0.0, hR = 0.0;           // keypoint height AR(1) noise per foot
        double phase = 6.28318 * U01(rng);
        bool ok = true;
        for (int k = 0; k < n && ok; ++k)
        {
            KF kf;
            kf.t = k * dt;
            kf.p = p;
            const double th = 0.35 * std::sin(2 * M_PI * kf.t / 20.0);
            const double dist = 7.0;
            const Eigen::Vector3d c = p + Eigen::Vector3d(-dist * std::cos(th), -dist * std::sin(th), CAMH - p(2));
            kf.Rcw = LookAt(c, p);
            kf.tcw = -kf.Rcw * c;
            Eigen::Vector2d px;
            double z;
            if (!Project(kf, p, px, z) || (p - c).norm() < 2.0)
                ok = false;
            const double range = (kf.Rcw * p + kf.tcw)(2);
            kf.u = px(0) + PXSIG * N(rng);
            kf.v = px(1) + PXSIG * N(rng);
            kf.lpx = F * CL * Htrue / range * std::exp(LOGSIG * N(rng));
            // feet: walking direction, anti-phase swing, lateral offset
            const Eigen::Vector2d dir = v.head<2>().norm() > 0.2 ? Eigen::Vector2d(v.head<2>().normalized()) : Eigen::Vector2d(0, 1);
            const Eigen::Vector2d lat(-dir(1), dir(0));
            phase += 2 * M_PI * 1.8 * dt;      // step frequency ~1.8 Hz
            const double sw = 0.30 * std::sin(phase);
            const Eigen::Vector2d fL = p.head<2>() + dir * sw + lat * 0.12, fR = p.head<2>() - dir * sw - lat * 0.12;
            hL = 0.6 * hL + 0.8 * 0.015 * N(rng);
            hR = 0.6 * hR + 0.8 * 0.015 * N(rng);
            const bool leftSwing = std::cos(phase) > 0;   // swing foot = the one moving forward fastest
            const double zL = std::max(0.0, (leftSwing ? 0.08 + 0.05 * std::abs(N(rng)) : 0.03) + hL), zR = std::max(0.0, (leftSwing ? 0.03 : 0.08 + 0.05 * std::abs(N(rng))) + hR);
            // lowest keypoint pixel of each foot: the foot's point at its keypoint height; pixel noise added separately
            Eigen::Vector2d pL, pR;
            double dzL, dzR;
            const bool visL = Project(kf, Eigen::Vector3d(fL(0), fL(1), zL), pL, dzL), visR = Project(kf, Eigen::Vector3d(fR(0), fR(1), zR), pR, dzR);
            kf.hasFoot = visL && visR;
            if (kf.hasFoot)
            {
                const Eigen::Vector2d lowest = pL(1) > pR(1) ? pL : pR;          // the lower foot in the image (largest v)
                kf.fu = lowest(0) + PXSIG * N(rng);
                kf.fv = lowest(1) + PXSIG * N(rng);
                const Eigen::Vector2d mid = 0.5 * (pL + pR);
                kf.mu = mid(0) + CPX_U * N(rng);
                kf.mv = mid(1) + CPX_V * N(rng);
            }
            // z_f error of this contact: bias + noise; raised-surface tail for far contacts
            kf.raised = false;
            kf.zfErr = ZFBIAS + ZFSIG * N(rng);
            if (range > 10.0 && U01(rng) < gRaisedP)
            {
                kf.raised = true;
                kf.zfErr = gRaisedH[static_cast<size_t>(U01(rng) * gRaisedH.size()) % gRaisedH.size()] + ZFSIG * N(rng);
            }
            W.kf.push_back(kf);
            for (int a = 0; a < 2; ++a)
            {
                const Eigen::Vector2d w = L * Eigen::Vector2d(N(rng), N(rng));
                const double pa = p(a) + dt * v(a) + w(0), va = v(a) + w(1);
                p(a) = pa;
                v(a) = va;
            }
            p(2) = 0.55 * Htrue;
        }
        if (!ok)
            continue;
        Eigen::Vector2d lo(1e9, 1e9), hi(-1e9, -1e9);
        for (const KF &k : W.kf)
            for (int a = 0; a < 2; ++a)
            {
                lo(a) = std::min(lo(a), k.p(a));
                hi(a) = std::max(hi(a), k.p(a));
            }
        std::uniform_real_distribution<double> ux(lo(0) - 10, hi(0) + 3), uy(lo(1) - 6, hi(1) + 6);
        for (int j = 0; j < NPOINTS; ++j)
            W.pts.emplace_back(ux(rng), uy(rng), 0.02 * N(rng));
        W.obs.assign(W.pts.size(), {});
        for (size_t j = 0; j < W.pts.size(); ++j)
            for (size_t i = 0; i < W.kf.size(); ++i)
            {
                Eigen::Vector2d px;
                double z;
                if (Project(W.kf[i], W.pts[j], px, z))
                    W.obs[j].push_back({static_cast<int>(i), Eigen::Vector2d(px(0) + POINT_PX_SIGMA * N(rng), px(1) + POINT_PX_SIGMA * N(rng))});
            }
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    if (argc < 7)
    {
        std::cerr << "usage: synthetic_floor_px MODE(none|A|B|Ap|Bp) B_TRUE NKF NSEEDS TAIL(A|B) MINELEV_DEG [feet raisedHeights zfbiasCal]\n";
        return 2;
    }
    const std::string mode = argv[1];
    const double bTrue = std::atof(argv[2]);
    const double Htrue = H0 * std::exp(bTrue);
    const int NKF = std::atoi(argv[3]), nseeds = std::atoi(argv[4]);
    gRaisedP = std::string(argv[5]) == "B" ? 0.47 : 0.10;
    const double minElev = std::atof(argv[6]);
    const bool feet = (argc > 7 ? std::atoi(argv[7]) : 1) != 0;
    if (argc > 8)
    {
        gRaisedH.clear();
        std::stringstream ss(argv[8]);
        std::string tok;
        while (std::getline(ss, tok, ','))
            gRaisedH.push_back(std::atof(tok.c_str()));
    }
    const double zfBiasCal = argc > 9 ? std::atof(argv[9]) : 0.02;
    const bool oldA = mode == "A", oldB = mode == "B", newA = mode == "Ap", newB = mode == "Bp";
    const bool useBias = mode != "none";
    const bool joint = oldB || newB;
    gB = newB;
    const double ptZnoise = newB ? POINT_Z_SIGMA_B : 0.05;
    const int kNb = 5;
    PedestrianParams P;       // gate parameters (production values of config/variants/*_px)
    P.floorRaisedMaxM = 0.10;
    P.floorMinElevDeg = minElev;
    P.floorNeighbours = 5;
    P.floorPlaneSigmaMaxM = 0.02;
    std::vector<double> errAll, rangeAbsM, rangeAbsPct, rangeSigned, bHat30, bHat100, bHatEnd;
    long nOpp = 0, nEdges = 0, nGatedA = 0, nGatedB = 0, nGatedElev = 0;
    for (int seed = 1; seed <= nseeds; ++seed)
    {
        World W;
        if (!MakeWorld(seed, Htrue, W, NKF))
        {
            std::cerr << "no admissible world for seed " << seed << "\n";
            return 3;
        }
        const int n = static_cast<int>(W.kf.size());
        std::vector<Vec6> est(n, Vec6::Zero());
        std::vector<bool> has(n, false);
        double bPers = 0.0;
        std::vector<Eigen::Vector3d> ptEst;
        {
            std::mt19937 r2(seed * 31 + 5);
            std::normal_distribution<double> N(0, 1);
            for (const auto &p : W.pts)
                ptEst.push_back(p + Eigen::Vector3d(0.05 * N(r2), 0.05 * N(r2), ptZnoise * N(r2)));
        }
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
            std::vector<VertexPedState *> st;
            for (int k = first; k <= i; ++k)
            {
                if (!has[k])
                {
                    const Eigen::Vector3d b((W.kf[k].u - CXP) / F, (W.kf[k].v - CYP) / F, 1.0);
                    const double z = F * CL * H0 / W.kf[k].lpx;
                    Vec6 x;
                    x << W.kf[k].Rcw.transpose() * (z * b - W.kf[k].tcw), Eigen::Vector3d::Zero();
                    est[k] = x;
                    has[k] = true;
                }
                auto *s = new VertexPedState();
                s->setId(id++);
                s->setEstimate(est[k]);
                s->setFixed(k < firstFree);
                opt.addVertex(s);
                st.push_back(s);
            }
            VertexPedBias *vb = nullptr;
            if (useBias)
            {
                vb = new VertexPedBias();
                vb->setId(id++);
                vb->setEstimate(bPers);
                opt.addVertex(vb);
                auto *pr = new EdgePedBiasPrior();
                pr->setVertex(0, vb);
                pr->setMeasurement(0.0);
                pr->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (SIGB * SIGB));
                opt.addEdge(pr);
            }
            for (int k = first; k <= i; ++k)
            {
                auto *s = st[k - first];
                if (k >= firstFree)
                {
                    auto *e = new EdgePedReproj<VertexPedState, g2o::VertexSE3Expmap>(F, F, CXP, CYP);
                    e->setVertex(0, s);
                    e->setVertex(1, poses[k - first]);
                    e->setMeasurement(Eigen::Vector2d(W.kf[k].u, W.kf[k].v));
                    e->setInformation(Eigen::Matrix2d::Identity() / (PXSIG * PXSIG));
                    auto *rk = new g2o::RobustKernelHuber();
                    rk->setDelta(kHuber2D);
                    e->setRobustKernel(rk);
                    opt.addEdge(e);
                }
                {
                    auto *e = new EdgePedLogRange<g2o::VertexSE3Expmap>(vb != nullptr);
                    e->setVertex(0, s);
                    e->setVertex(1, poses[k - first]);
                    if (vb)
                        e->setVertex(2, vb);
                    e->setMeasurement(std::log(F) + std::log(CL) + std::log(H0) - std::log(W.kf[k].lpx));
                    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (LOGSIG * LOGSIG));
                    auto *rk = new g2o::RobustKernelHuber();
                    rk->setDelta(kHuber1D);
                    e->setRobustKernel(rk);
                    opt.addEdge(e);
                }
                if (k > first && k >= firstFree)
                {
                    auto *e = new EdgePedWna();
                    e->setVertex(0, st[k - 1 - first]);
                    e->setVertex(1, s);
                    e->setMeasurement(Vec6::Zero());
                    e->SetDt(W.kf[k].t - W.kf[k - 1].t, QV);
                    opt.addEdge(e);
                }
            }
            std::vector<g2o::VertexSBAPointXYZ *> ptv(W.pts.size(), nullptr);
            if (mode != "none" && feet)
            {
                for (int k = firstFree; k <= i; ++k)
                {
                    if (!W.kf[k].hasFoot)
                        continue;
                    ++nOpp;
                    auto *s = st[k - first];
                    const Eigen::Vector3d cam = -W.kf[k].Rcw.transpose() * W.kf[k].tcw;
                    const bool px = newA || newB;
                    const double cu = px ? W.kf[k].mu : W.kf[k].fu, cv = px ? W.kf[k].mv : W.kf[k].fv;
                    // regional z_f from the (noisy) map points, plus the injected contact-level error (bias + noise, raised-surface tail)
                    auto fe = PedestrianBA::RegionalFloor(ptEst, s->p().head<2>(), cam(2), px ? 6.0 : 3.0, 0.05, 0.3, 30, 0.25);
                    if (!fe.ok)
                        continue;
                    const double zfRaw = fe.zf + W.kf[k].zfErr;
                    const Eigen::Vector3d bray((cu - CXP) / F, (cv - CYP) / F, 1.0);
                    const Eigen::Vector3d dw = W.kf[k].Rcw.transpose() * bray;
                    if (!px && dw(2) / dw.norm() > -0.02)
                        continue;
                    std::vector<int> nb;
                    double zfUse = zfRaw;
                    if (px)
                    {
                        const double elev = std::asin(std::max(-1.0, std::min(1.0, -dw(2) / dw.norm()))) * 180.0 / M_PI;
                        const int stA = ped::FloorStageA(P, zfRaw, 0.0 /* global lowest floor mode: the flat floor z = 0 */, elev);
                        if (stA == 2) { ++nGatedA; continue; }
                        if (stA == 3) { ++nGatedElev; continue; }
                        zfUse = zfRaw - zfBiasCal;
                    }
                    if (joint)
                    {
                        const double zq = zfUse;
                        const double band = px ? 0.05 : 0.15;
                        const Eigen::Vector2d c2 = s->p().head<2>();
                        std::vector<std::pair<double, int>> cand;
                        std::vector<double> zs;
                        for (size_t j = 0; j < ptEst.size(); ++j)
                        {
                            if (std::abs(ptEst[j](2) - zq) > band)
                                continue;
                            const double d = (ptEst[j].head<2>() - (px ? c2 : Eigen::Vector2d((cam + ((zq - cam(2)) / dw(2)) * dw).head<2>())).eval()).norm();
                            if (d <= 1.0)
                            {
                                cand.push_back({d, static_cast<int>(j)});
                                zs.push_back(ptEst[j](2));
                            }
                        }
                        if (px)
                        {
                            double m = 0, ss = 0;
                            for (double z : zs) m += z;
                            if (!zs.empty()) m /= zs.size();
                            for (double z : zs) ss += (z - m) * (z - m);
                            const double sig = zs.empty() ? 0.0 : std::sqrt(ss / zs.size());
                            if (ped::FloorStageB(P, static_cast<int>(zs.size()), sig) != 0) { ++nGatedB; continue; }
                        }
                        std::sort(cand.begin(), cand.end());
                        if (static_cast<int>(cand.size()) < kNb)
                            continue;
                        for (int q = 0; q < kNb; ++q)
                            nb.push_back(cand[q].second);
                        for (int j : nb)
                            if (!ptv[j])
                            {
                                auto *pv = new g2o::VertexSBAPointXYZ();
                                pv->setId(id++);
                                pv->setEstimate(ptEst[j]);
                                opt.addVertex(pv);
                                ptv[j] = pv;
                                for (const auto &ob : W.obs[j])
                                {
                                    if (ob.first < first || ob.first > i)
                                        continue;
                                    auto *re = new EdgePedReproj<g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>(F, F, CXP, CYP);
                                    re->setVertex(0, pv);
                                    re->setVertex(1, poses[ob.first - first]);
                                    re->setMeasurement(ob.second);
                                    re->setInformation(Eigen::Matrix2d::Identity() / (POINT_PX_SIGMA * POINT_PX_SIGMA));
                                    auto *rk = new g2o::RobustKernelHuber();
                                    rk->setDelta(kHuber2D);
                                    re->setRobustKernel(rk);
                                    opt.addEdge(re);
                                }
                            }
                    }
                    if (px)
                    {
                        auto *fl = new EdgePedFloorPx<g2o::VertexSE3Expmap>(F, F, CXP, CYP, cu, cv, static_cast<int>(nb.size()), zfUse, 0.053, /*poseGradient=*/newB);
                        fl->setVertex(0, s);
                        fl->setVertex(1, poses[k - first]);
                        for (size_t q = 0; q < nb.size(); ++q)
                            fl->setVertex(2 + q, ptv[nb[q]]);
                        fl->setMeasurement(Eigen::Vector2d(cu, cv));
                        Eigen::Matrix2d I = Eigen::Matrix2d::Zero();
                        I(0, 0) = 1.0 / (CPX_U * CPX_U);
                        I(1, 1) = 1.0 / (CPX_V * CPX_V);
                        fl->setInformation(I);
                        auto *rk = new g2o::RobustKernelHuber();
                        rk->setDelta(kHuber2D);
                        fl->setRobustKernel(rk);
                        opt.addEdge(fl);
                    }
                    else
                    {
                        const Eigen::Vector3d G0 = cam + ((zfUse - cam(2)) / dw(2)) * dw;
                        auto *fl = new EdgePedFloorContact<g2o::VertexSE3Expmap>(F, F, CXP, CYP, cu, cv, static_cast<int>(nb.size()), zfUse, 0.036, /*poseGradient=*/oldB);
                        fl->setVertex(0, s);
                        fl->setVertex(1, poses[k - first]);
                        for (size_t q = 0; q < nb.size(); ++q)
                            fl->setVertex(2 + q, ptv[nb[q]]);
                        fl->setMeasurement(Eigen::Vector2d(cu, cv));
                        const double phi = std::atan2(G0(1) - cam(1), G0(0) - cam(0));
                        const Eigen::Vector2d ur(std::cos(phi), std::sin(phi)), ut(-std::sin(phi), std::cos(phi));
                        fl->setInformation(ur * ur.transpose() / (0.358 * 0.358) + ut * ut.transpose() / (0.2755 * 0.2755));
                        auto *rk = new g2o::RobustKernelHuber();
                        rk->setDelta(kHuber2D);
                        fl->setRobustKernel(rk);
                        opt.addEdge(fl);
                    }
                    ++nEdges;
                }
            }
            opt.initializeOptimization();
            opt.optimize(10);
            for (int k = firstFree; k <= i; ++k)
                est[k] = st[k - first]->estimate();
            if (vb)
                bPers = vb->estimate();
            if (i == 29) bHat30.push_back(bPers - bTrue);
            if (i == 99) bHat100.push_back(bPers - bTrue);
            for (size_t j = 0; j < ptv.size(); ++j)
                if (ptv[j])
                    ptEst[j] = ptv[j]->estimate();
        }
        bHatEnd.push_back(bPers);   // |b_hat| (the estimate itself) for the no-contact criterion
        for (int i = std::min(10, n - 1); i < n; ++i)
        {
            const Eigen::Vector3d Xc = W.kf[i].Rcw * est[i].head<3>() + W.kf[i].tcw, Xt = W.kf[i].Rcw * W.kf[i].p + W.kf[i].tcw;
            rangeSigned.push_back((Xc.norm() - Xt.norm()) / Xt.norm() * 100.0);
            rangeAbsM.push_back(std::abs(Xc.norm() - Xt.norm()));
            rangeAbsPct.push_back(std::abs(Xc.norm() - Xt.norm()) / Xt.norm() * 100.0);
        }
    }
    auto med = [](std::vector<double> v) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
    auto medabs = [&](std::vector<double> v) { for (double &x : v) x = std::abs(x); return med(v); };
    std::printf("%s\tfeet=%d\ttail=%s\tb_true=%+.3f\tNKF=%d\tseeds=%d\tmedian_abs_range_err_m=%.4f\tmedian_abs_range_err_pct=%.3f\tmedian_signed_pct=%.3f\tmedian_abs_bhat_err_at30=%.4f\tat100=%.4f\tmedian_abs_bhat_end=%.4f\t"
                "edges=%ld/%ld\tgated_raised=%ld\tgated_elev=%ld\tgated_plane=%ld\n",
                mode.c_str(), feet ? 1 : 0, argv[5], bTrue, NKF, nseeds, med(rangeAbsM), med(rangeAbsPct), med(rangeSigned), medabs(bHat30), medabs(bHat100), medabs(bHatEnd), nEdges, nOpp, nGatedA, nGatedElev, nGatedB);
    return 0;
}
