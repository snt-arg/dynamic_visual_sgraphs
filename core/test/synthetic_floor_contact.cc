// P9-C3 step 1: synthetic test of the floor-contact factors with the PRODUCTION edges (PedestrianFactors.h) and the production helper
// PedestrianBA::RegionalFloor, in the window pattern of LocalInertialBA (10 free states, older ones fixed, 10 LM iterations, state estimates persisted).
//   * walker: white-noise-acceleration motion (q_v = 0.1386, exact discretisation) at root height 0.55 H, stature H in {1.55, 1.70, 1.85}; H0 = 1.70 is fixed in
//     every model (the frozen configuration: root reprojection + log-range with H0 + WNA)
//   * camera: follows the walker at ~7 m (+-0.35 rad swing), 1.5 m above a flat floor z = 0, exact poses, FIXED in the graph (as in the 5d synthetic)
//   * measurements: root pixel (1.5 px), l_px = f cL H / range (log noise 0.031, the nose->heel length scales with the true stature), lowest foot pixel of a foot point
//     whose horizontal offset to the root and whose height above the floor are random (AR(1), calibrated scales, see FOOT_* below) + 1.5 px
//   * static points: floor points (z ~ N(0, 0.02)) seen by the fixed cameras with 0.5 px noise; they are free vertices (Variant B reads their z; Variant A only the
//     regional estimate of their heights)
// modes: frozen | A (regional z_f, pedestrian only) | B (joint: plane through the k nearest live floor points)
// P9-C3b: the range model is the depth-z model (erratum) and Variants A / B carry the per-track log-scale bias vertex b (prior N(0, SIGB^2)); the
// walker's stature is H0 exp(b_true). usage: synthetic_floor_contact MODE B_TRUE NKF NSEEDS [bias=1 feet=1 sigRad sigTan delta k logsigModel]
//   MODE none | A | B; b only for A / B (bias=1); feet=0: the feet are never visible (no floor edge)
// prints (one line per call, tab separated): mode H seeds n_states | median 3-D error, RMS, mean signed range error / range [%], median depth-ray error [m]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <random>
#include <vector>

#include <Eigen/Sparse>

#include "PedestrianBA.h"
#include "PedestrianFactors.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"

using namespace ORB_SLAM3;
using namespace ORB_SLAM3::ped;

static const double F = 415.69219381653056, CXP = 360.0, CYP = 240.0;
static const double CL = 0.855387, SIGB = 0.0422, QV = 0.1386, LOGSIG = 0.031, PXSIG = 1.5, H0 = 1.70;
static const double kHuber2D = std::sqrt(5.991), kHuber1D = std::sqrt(3.841);
// foot model (synthetic scales of the same order as the calibrated 0c residuals)
static const double FOOT_DELTA_MEAN = 0.036, FOOT_H_SIGMA = 0.044, FOOT_H_PHI = 0.8;   // keypoint height above the sole
static const double FOOT_RAD_SIGMA = 0.10, FOOT_TAN_SIGMA = 0.10, FOOT_XY_PHI = 0.6;   // horizontal foot offset to the root (stride), AR(1)
static const double POINT_Z_SIGMA = 0.02, POINT_PX_SIGMA = 0.5;
static const int NPOINTS = 6000;   // ~12 floor points / m2 in the swept corridor (the office maps hold ~70 000 points)

struct KF
{
    double t;
    Eigen::Matrix3d Rcw;
    Eigen::Vector3d tcw, p;
    double u, v, lpx, fu, fv;
    bool hasFoot;
};

struct World
{
    std::vector<KF> kf;
    std::vector<Eigen::Vector3d> pts;                     // true static floor points
    std::vector<std::vector<std::pair<int, Eigen::Vector2d>>> obs; // per point: (keyframe, pixel)
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

static bool MakeWorld(int seed, double Htrue, World &W, int n, double dt = 0.1)
{
    std::mt19937 rng(seed * 7919 + 13);
    std::normal_distribution<double> N(0, 1);
    for (int attempt = 0; attempt < 3000; ++attempt)
    {
        W = World();
        const double dt3 = dt * dt * dt / 3, dt2 = dt * dt / 2;
        Eigen::Matrix2d Q;
        Q << dt3, dt2, dt2, dt;
        Q *= QV;
        Eigen::LLT<Eigen::Matrix2d> llt(Q + 1e-15 * Eigen::Matrix2d::Identity());
        const Eigen::Matrix2d L = llt.matrixL();
        Eigen::Vector3d p(8.0, N(rng), 0.55 * Htrue), v(0, (N(rng) > 0 ? 1.0 : -1.0), 0);
        v(0) += 0.2 * N(rng);
        v(1) += 0.2 * N(rng);
        // foot-offset / height AR(1) state
        double rad = 0, tan_ = 0, hz = 0;
        const double sa = std::sqrt(1 - FOOT_XY_PHI * FOOT_XY_PHI), sh = std::sqrt(1 - FOOT_H_PHI * FOOT_H_PHI);
        bool ok = true;
        std::vector<Eigen::Vector3d> traj;
        for (int k = 0; k < n && ok; ++k)
        {
            KF kf;
            kf.t = k * dt;
            kf.p = p;
            const double th = 0.35 * std::sin(2 * M_PI * kf.t / 20.0);
            const double dist = 7.0;
            const Eigen::Vector3d c = p + Eigen::Vector3d(-dist * std::cos(th), -dist * std::sin(th), 1.5 - p(2));
            kf.Rcw = LookAt(c, p);
            kf.tcw = -kf.Rcw * c;
            Eigen::Vector2d px;
            double z;
            if (!Project(kf, p, px, z) || (p - c).norm() < 2.0)
                ok = false;
            const double range = (kf.Rcw * p + kf.tcw)(2); // optical-axis depth z (erratum)
            kf.u = px(0) + PXSIG * N(rng);
            kf.v = px(1) + PXSIG * N(rng);
            kf.lpx = F * CL * Htrue / range * std::exp(LOGSIG * N(rng));
            // foot: horizontal offset along / across the camera->root direction, height above the floor
            rad = FOOT_XY_PHI * rad + sa * FOOT_RAD_SIGMA * N(rng);
            tan_ = FOOT_XY_PHI * tan_ + sa * FOOT_TAN_SIGMA * N(rng);
            hz = FOOT_H_PHI * hz + sh * FOOT_H_SIGMA * N(rng);
            Eigen::Vector2d ur = (p - c).head<2>().normalized(), ut(-ur(1), ur(0));
            Eigen::Vector3d foot;
            foot.head<2>() = p.head<2>() + rad * ur + tan_ * ut;
            foot(2) = std::max(0.0, FOOT_DELTA_MEAN + hz);
            Eigen::Vector2d fpx;
            double fz;
            kf.hasFoot = Project(kf, foot, fpx, fz);
            kf.fu = fpx(0) + PXSIG * N(rng);
            kf.fv = fpx(1) + PXSIG * N(rng);
            W.kf.push_back(kf);
            // WNA step
            for (int a = 0; a < 3; ++a)
            {
                const Eigen::Vector2d w = L * Eigen::Vector2d(N(rng), N(rng));
                const double pa = p(a) + dt * v(a) + (a < 2 ? w(0) : 0.0), va = v(a) + (a < 2 ? w(1) : 0.0);
                p(a) = pa;
                v(a) = va;
            }
            p(2) = 0.55 * Htrue;
            v(2) = 0;
        }
        if (!ok)
            continue;
        // static floor points around the corridor swept by the walker and the camera
        Eigen::Vector2d lo(1e9, 1e9), hi(-1e9, -1e9);
        for (const KF &k : W.kf)
            for (int a = 0; a < 2; ++a)
            {
                lo(a) = std::min(lo(a), k.p(a));
                hi(a) = std::max(hi(a), k.p(a));
            }
        std::uniform_real_distribution<double> ux(lo(0) - 10, hi(0) + 3), uy(lo(1) - 6, hi(1) + 6);
        for (int j = 0; j < NPOINTS; ++j)
            W.pts.emplace_back(ux(rng), uy(rng), POINT_Z_SIGMA * N(rng));
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

// P9-C3b gate erratum: marginal posterior sigma of the bias from the Gaussian information matrix (Huber removed) of the graph at its current estimates.
// Jacobians by central differences of the production computeError. onlyEdgesWithFreeState: drop the edges that touch no free state vertex (the range
// edges of the fixed states), keeping the bias prior. Returns sqrt([H^-1]_bb) over the free vertices.
static double InfoSigmaB(g2o::SparseOptimizer &opt, g2o::OptimizableGraph::Vertex *vb, bool onlyEdgesWithFreeState)
{
    std::map<const g2o::OptimizableGraph::Vertex *, int> off;
    int dim = 0;
    for (const auto &kv : opt.vertices())
    {
        auto *v = static_cast<g2o::OptimizableGraph::Vertex *>(kv.second);
        if (v->fixed())
            continue;
        off[v] = dim;
        dim += v->dimension();
    }
    std::vector<Eigen::Triplet<double>> T;
    const double eps = 1e-6;
    for (auto *eb : opt.edges())
    {
        auto *e = static_cast<g2o::OptimizableGraph::Edge *>(eb);
        e->setRobustKernel(nullptr);
        if (onlyEdgesWithFreeState && !dynamic_cast<EdgePedBiasPrior *>(e))
        {
            // drop the edges that carry the bias but no free state vertex (the range edges of the fixed states)
            bool hasState = false, hasBias = false;
            for (auto *hv : e->vertices())
            {
                auto *v = static_cast<g2o::OptimizableGraph::Vertex *>(hv);
                if (dynamic_cast<VertexPedState *>(v) && !v->fixed())
                    hasState = true;
                if (v == vb)
                    hasBias = true;
            }
            if (hasBias && !hasState)
                continue;
        }
        const int m = e->dimension();
        std::vector<g2o::OptimizableGraph::Vertex *> fv;
        std::vector<int> col0;
        int nc = 0;
        for (auto *hv : e->vertices())
        {
            auto *v = static_cast<g2o::OptimizableGraph::Vertex *>(hv);
            if (v && !v->fixed())
            {
                fv.push_back(v);
                col0.push_back(nc);
                nc += v->dimension();
            }
        }
        if (fv.empty())
            continue;
        Eigen::MatrixXd J(m, nc);
        for (size_t a = 0; a < fv.size(); ++a)
            for (int d = 0; d < fv[a]->dimension(); ++d)
            {
                std::vector<double> dl(fv[a]->dimension(), 0.0);
                Eigen::VectorXd ep(m), em(m);
                dl[d] = eps;
                fv[a]->push();
                fv[a]->oplus(dl.data());
                e->computeError();
                ep = Eigen::Map<const Eigen::VectorXd>(e->errorData(), m);
                fv[a]->pop();
                dl[d] = -eps;
                fv[a]->push();
                fv[a]->oplus(dl.data());
                e->computeError();
                em = Eigen::Map<const Eigen::VectorXd>(e->errorData(), m);
                fv[a]->pop();
                J.col(col0[a] + d) = (ep - em) / (2 * eps);
            }
        e->computeError();
        const Eigen::MatrixXd Om = Eigen::Map<const Eigen::MatrixXd>(e->informationData(), m, m);
        const Eigen::MatrixXd Hb = J.transpose() * Om * J;
        for (size_t a = 0; a < fv.size(); ++a)
            for (size_t c = 0; c < fv.size(); ++c)
                for (int i = 0; i < fv[a]->dimension(); ++i)
                    for (int j = 0; j < fv[c]->dimension(); ++j)
                        T.emplace_back(off[fv[a]] + i, off[fv[c]] + j, Hb(col0[a] + i, col0[c] + j));
    }
    Eigen::SparseMatrix<double> H(dim, dim);
    H.setFromTriplets(T.begin(), T.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> ldlt(H);
    if (ldlt.info() != Eigen::Success)
        return -1.0;
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(dim);
    const int ib = off[vb];
    rhs(ib) = 1.0;
    const Eigen::VectorXd x = ldlt.solve(rhs);
    return x(ib) > 0 ? std::sqrt(x(ib)) : -1.0;
}

int main(int argc, char **argv)
{
    if (argc < 5)
    {
        std::cerr << "usage: synthetic_floor_contact MODE(none|A|B) B_TRUE NKF NSEEDS [bias feet sigRad sigTan delta k logsigModel]\n";
        return 2;
    }
    const std::string mode = argv[1];
    const double bTrue = std::atof(argv[2]);
    const double Htrue = H0 * std::exp(bTrue);
    const int NKF = std::atoi(argv[3]);
    const int nseeds = std::atoi(argv[4]);
    const bool useBias = (argc > 5 ? std::atoi(argv[5]) : 1) != 0 && mode != "none";
    const bool feet = (argc > 6 ? std::atoi(argv[6]) : 1) != 0;
    const double sigRad = argc > 7 ? std::atof(argv[7]) : 0.358, sigTan = argc > 8 ? std::atof(argv[8]) : 0.2755, delta = argc > 9 ? std::atof(argv[9]) : 0.036;
    const int kNb = argc > 10 ? std::atoi(argv[10]) : 5;
    const bool infoOn = (argc > 12 ? std::atoi(argv[12]) : 0) != 0 && mode != "none" && useBias; // gate erratum: sigma_b,post of the final window
    std::vector<double> sigI, sigII, sigIII;
    const double logsigModel = argc > 11 ? std::atof(argv[11]) : LOGSIG;   // sigma_log of the range factor in the MODEL (the data always use LOGSIG = 0.031)
    const bool useA = mode == "A", useB = mode == "B";
    std::vector<double> errAll, rangeSigned, rangeAbsM, rangeAbsPct, bErr, bHat30, bHat100;
    long nOpp = 0, nEdges = 0;
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
                ptEst.push_back(p + Eigen::Vector3d(0.05 * N(r2), 0.05 * N(r2), 0.05 * N(r2))); // converged map points: ~5 cm
        }
        for (int i = std::min(2, n - 1); i < n; ++i)
        for (int pass = 0; pass < ((infoOn && i == n - 1) ? 2 : 1); ++pass)
        {
            const bool infoPass = pass == 1; // (II): every state of the window free, at the truth, no solve
            std::vector<Vec6> estB = est;
            std::vector<bool> hasB = has;
            const double bPersB = bPers;
            const std::vector<Eigen::Vector3d> ptEstB = ptEst;
            const long nEdgesB = nEdges, nOppB = nOpp;
            if (infoPass)
            {
                for (int k = 0; k < n; ++k)
                {
                    est[k] << W.kf[k].p, Eigen::Vector3d::Zero();
                    has[k] = true;
                }
                bPers = bTrue;
                ptEst = W.pts;
            }
            const int first = std::max(0, i - 200), firstFree = infoPass ? first : std::max(first, i - 9);
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
                    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (logsigModel * logsigModel));
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
            // ---- floor contact
            std::vector<g2o::VertexSBAPointXYZ *> ptv(W.pts.size(), nullptr);
            if ((useA || useB) && feet)
            {
                for (int k = firstFree; k <= i; ++k)
                {
                    if (!W.kf[k].hasFoot)
                        continue;
                    ++nOpp;
                    auto *s = st[k - first];
                    const Eigen::Vector3d cam = -W.kf[k].Rcw.transpose() * W.kf[k].tcw;
                    const auto fe = PedestrianBA::RegionalFloor(ptEst, s->p().head<2>(), cam(2), 3.0, 0.05, 0.3, 30, 0.25);
                    if (!fe.ok)
                        continue;
                    const Eigen::Vector3d bray((W.kf[k].fu - CXP) / F, (W.kf[k].fv - CYP) / F, 1.0);
                    const Eigen::Vector3d dw = W.kf[k].Rcw.transpose() * bray;
                    if (dw(2) / dw.norm() > -0.02)
                        continue;
                    const Eigen::Vector3d G0 = cam + ((fe.zf - cam(2)) / dw(2)) * dw;
                    std::vector<int> nb;
                    if (useB)
                    {
                        std::vector<std::pair<double, int>> cand;
                        for (size_t j = 0; j < ptEst.size(); ++j)
                        {
                            if (std::abs(ptEst[j](2) - fe.zf) > 0.15)
                                continue;
                            const double d = (ptEst[j].head<2>() - G0.head<2>()).norm();
                            if (d <= 1.0)
                                cand.push_back({d, static_cast<int>(j)});
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
                    auto *fl = new EdgePedFloorContact<g2o::VertexSE3Expmap>(F, F, CXP, CYP, W.kf[k].fu, W.kf[k].fv, static_cast<int>(nb.size()), fe.zf, delta, /*poseGradient=*/useB);
                    fl->setVertex(0, s);
                    fl->setVertex(1, poses[k - first]);
                    for (size_t q = 0; q < nb.size(); ++q)
                        fl->setVertex(2 + q, ptv[nb[q]]);
                    fl->setMeasurement(Eigen::Vector2d(W.kf[k].fu, W.kf[k].fv));
                    const double phi = std::atan2(G0(1) - cam(1), G0(0) - cam(0));
                    const Eigen::Vector2d ur(std::cos(phi), std::sin(phi)), ut(-std::sin(phi), std::cos(phi));
                    fl->setInformation(ur * ur.transpose() / (sigRad * sigRad) + ut * ut.transpose() / (sigTan * sigTan));
                    auto *rk = new g2o::RobustKernelHuber();
                    rk->setDelta(kHuber2D);
                    fl->setRobustKernel(rk);
                    opt.addEdge(fl);
                    ++nEdges;
                }
            }
            if (!infoPass)
            {
                opt.initializeOptimization();
                opt.optimize(10);
            }
            for (int k = firstFree; k <= i; ++k)
                est[k] = st[k - first]->estimate();
            if (vb)
                bPers = vb->estimate();
            if (i == 29 && !infoPass) bHat30.push_back(bPers - bTrue);
            if (i == 99 && !infoPass) bHat100.push_back(bPers - bTrue);
            for (size_t j = 0; j < ptv.size(); ++j)
                if (ptv[j])
                    ptEst[j] = ptv[j]->estimate();
            if (infoOn && i == n - 1)
            {
                // the graph as built for the last keyframe, free vertices at the truth
                for (int k = first; k <= i; ++k)
                {
                    Vec6 x;
                    x << W.kf[k].p, Eigen::Vector3d::Zero();
                    st[k - first]->setEstimate(x);
                }
                vb->setEstimate(bTrue);
                for (size_t j = 0; j < ptv.size(); ++j)
                    if (ptv[j])
                        ptv[j]->setEstimate(W.pts[j]);
                if (!infoPass)
                {
                    sigI.push_back(InfoSigmaB(opt, vb, false));
                    sigIII.push_back(InfoSigmaB(opt, vb, true));
                }
                else
                    sigII.push_back(InfoSigmaB(opt, vb, false));
            }
            if (infoPass)
            {
                est = estB;
                has = hasB;
                bPers = bPersB;
                ptEst = ptEstB;
                nEdges = nEdgesB;
                nOpp = nOppB;
            }
        }
        bErr.push_back(bPers - bTrue);
        for (int i = std::min(10, n - 1); i < n; ++i) // skip the first states (no history)
        {
            errAll.push_back((est[i].head<3>() - W.kf[i].p).norm());
            const Eigen::Vector3d Xc = W.kf[i].Rcw * est[i].head<3>() + W.kf[i].tcw, Xt = W.kf[i].Rcw * W.kf[i].p + W.kf[i].tcw;
            rangeSigned.push_back((Xc.norm() - Xt.norm()) / Xt.norm() * 100.0);
            rangeAbsM.push_back(std::abs(Xc.norm() - Xt.norm()));
            rangeAbsPct.push_back(std::abs(Xc.norm() - Xt.norm()) / Xt.norm() * 100.0);
        }
    }
    auto med = [](std::vector<double> v) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return v[v.size() / 2]; };
    auto medabs = [&](std::vector<double> v) { for (double &x : v) x = std::abs(x); return med(v); };
    std::printf("%s%s\tfeet=%d\tb_true=%+.3f\tNKF=%d\tseeds=%d\tn=%zu\tmedian_abs_range_err_m=%.4f\tmedian_abs_range_err_pct=%.3f\tmedian_signed_range_err_pct=%.3f\t"
                "sigb_post_I=%.5f\tsigb_post_II=%.5f\tsigb_post_III=%.5f\t"
                "median_abs_bhat_err=%.4f\tmedian_bhat_err=%+.4f\tmedian_abs_bhat_err_at30=%.4f\tat100=%.4f\tmedian_abs_bhat=%.4f\tfloor_edges=%ld/%ld\n",
                mode.c_str(), useBias ? "+b" : "", feet ? 1 : 0, bTrue, NKF, nseeds, rangeAbsM.size(), med(rangeAbsM), med(rangeAbsPct), med(rangeSigned),
                med(sigI), med(sigII), med(sigIII), medabs(bErr), med(bErr), medabs(bHat30), medabs(bHat100), useBias ? medabs([&] { std::vector<double> v; for (double e : bErr) v.push_back(e + bTrue); return v; }()) : 0.0, nEdges, nOpp);
    return 0;
}
