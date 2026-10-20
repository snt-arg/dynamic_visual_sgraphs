// P9-C2 5d: synthetic identifiability test of the pedestrian graph with the PRODUCTION edges (PedestrianFactors.h) and the construction of
// PedestrianBA.cc (src/PedestrianBA.cc:60-210: state vertices, root reprojection + log-range with Huber, WNA between consecutive states, stature
// vertex + prior in the pre-5d version; since P9-C2 5d the stature is the constant H0 = HM). Camera poses are exact and fixed. Two modes:
//   batch   : one graph over all keyframes, everything free
//   window  : the production pattern -- at keyframe i the last Nfree states are free, the older ones are fixed (but still in the graph with their range
//             edges, which constrain the stature vertex exactly as in LocalInertialBA), the stature is free and persisted between calls, 10 LM iterations
// usage: synthetic_stature <file> <mode batch|window> [Nfree=10] [stature_mean=1.70] [stature_sigma=0.09] [cL] [qv] [logsig] [pxsig]
// prints: final_stature  stature@30 @100 @300  rms_pos_err_last_50  (window mode: stature after the call at keyframes 30, 100, 300)
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include "PedestrianFactors.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"

using namespace ORB_SLAM3;
using namespace ORB_SLAM3::ped;

struct KF
{
    double t;
    Eigen::Matrix3d Rcw;
    Eigen::Vector3d tcw;
    double u, v, lpx;
    Eigen::Vector3d p, vel;
};
const double F = 415.69219381653056, CXP = 360.0, CYP = 240.0;
double CL = 0.9026550273636589, QV = 0.1386, LOGSIG = 0.031, PXSIG = 1.5, HM = 1.70, HS = 0.09;
const double kHuber2D = std::sqrt(5.991), kHuber1D = std::sqrt(3.841);

// state estimates persisted between calls (the keyframe's PedState / Map::mPedStature)
std::vector<Vec6> est;
std::vector<bool> has;
double Hpers;

Vec6 init_state(const KF &k, double H)
{
    const Eigen::Vector3d b((k.u - CXP) / F, (k.v - CYP) / F, 1.0);
    const double z = (F * CL * H / k.lpx) / b.norm();
    Vec6 x;
    x << k.Rcw.transpose() * (z * b - k.tcw), Eigen::Vector3d::Zero();
    return x;
}

// one BA call over keyframes [first, last]; the states with index >= firstFree are free
double solve(const std::vector<KF> &kf, int first, int last, int firstFree, int iters)
{
    g2o::SparseOptimizer opt;
    auto *linear = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    auto *bs = new g2o::BlockSolverX(linear);
    auto *alg = new g2o::OptimizationAlgorithmLevenberg(bs);
    alg->setUserLambdaInit(1e0);
    opt.setAlgorithm(alg);
    opt.setVerbose(false);
    int id = 0;
    std::vector<g2o::VertexSE3Expmap *> poses;
    for (int i = first; i <= last; ++i)
    {
        auto *v = new g2o::VertexSE3Expmap();
        v->setId(id++);
        v->setEstimate(g2o::SE3Quat(kf[i].Rcw, kf[i].tcw));
        v->setFixed(true);
        opt.addVertex(v);
        poses.push_back(v);
    }
    std::vector<VertexPedState *> st;
    for (int i = first; i <= last; ++i)
    {
        if (!has[i]) { est[i] = init_state(kf[i], HM); has[i] = true; }
        auto *s = new VertexPedState();
        s->setId(id++);
        s->setEstimate(est[i]);
        s->setFixed(i < firstFree);
        opt.addVertex(s);
        st.push_back(s);
    }
    for (int i = first; i <= last; ++i)
    {
        auto *s = st[i - first];
        if (i >= firstFree) // root reprojection: skipped when state and pose are both fixed (PedestrianBA.cc:170-176)
        {
            auto *e = new EdgePedReproj<VertexPedState, g2o::VertexSE3Expmap>(F, F, CXP, CYP);
            e->setVertex(0, s);
            e->setVertex(1, poses[i - first]);
            e->setMeasurement(Eigen::Vector2d(kf[i].u, kf[i].v));
            e->setInformation(Eigen::Matrix2d::Identity() / (PXSIG * PXSIG));
            auto *rk = new g2o::RobustKernelHuber();
            rk->setDelta(kHuber2D);
            e->setRobustKernel(rk);
            opt.addEdge(e);
        }
        {   // log range: added for every keyframe, fixed ones too (PedestrianBA.cc:195-209); it then constrains only the stature
            auto *e = new EdgePedLogRange<g2o::VertexSE3Expmap>();
            e->setVertex(0, s);
            e->setVertex(1, poses[i - first]);
            e->setMeasurement(std::log(F) + std::log(CL) + std::log(HM) - std::log(kf[i].lpx));
            e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (LOGSIG * LOGSIG));
            auto *rk = new g2o::RobustKernelHuber();
            rk->setDelta(kHuber1D);
            e->setRobustKernel(rk);
            opt.addEdge(e);
        }
        if (i > first && i >= firstFree) // WNA between consecutive states; not both fixed (PedestrianBA.cc:215-231)
        {
            auto *e = new EdgePedWna();
            e->setVertex(0, st[i - 1 - first]);
            e->setVertex(1, s);
            e->setMeasurement(Vec6::Zero());
            e->SetDt(kf[i].t - kf[i - 1].t, QV);
            opt.addEdge(e);
        }
    }
    opt.initializeOptimization();
    opt.optimize(iters);
    for (int i = first; i <= last; ++i)
        if (i >= firstFree) est[i] = st[i - first]->estimate();
    Hpers = HM;
    return Hpers;
}

int main(int argc, char **argv)
{
    if (argc < 3) { std::cerr << "usage\n"; return 2; }
    const std::string mode = argv[2];
    const int Nfree = argc > 3 ? std::atoi(argv[3]) : 10;
    if (argc > 4) HM = std::atof(argv[4]);
    if (argc > 5) HS = std::atof(argv[5]);
    if (argc > 6) CL = std::atof(argv[6]);
    if (argc > 7) QV = std::atof(argv[7]);
    if (argc > 8) LOGSIG = std::atof(argv[8]);
    if (argc > 9) PXSIG = std::atof(argv[9]);
    std::ifstream f(argv[1]);
    int n;
    double Htrue;
    f >> n >> Htrue;
    std::vector<KF> kf(n);
    for (auto &k : kf)
    {
        f >> k.t;
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) f >> k.Rcw(r, c);
        f >> k.tcw(0) >> k.tcw(1) >> k.tcw(2) >> k.u >> k.v >> k.lpx >> k.p(0) >> k.p(1) >> k.p(2) >> k.vel(0) >> k.vel(1) >> k.vel(2);
    }
    est.assign(n, Vec6::Zero());
    has.assign(n, false);
    Hpers = HM;
    double H30 = 0, H100 = 0, H300 = 0;
    if (mode == "batch")
    {
        solve(kf, 0, n - 1, 0, 50);
        H300 = Hpers;
    }
    else
    {
        for (int i = 2; i < n; ++i)
        {
            const int first = std::max(0, i - 200), firstFree = std::max(first, i - Nfree + 1);
            solve(kf, first, i, firstFree, 10);
            if (i == 29) H30 = Hpers;
            if (i == 99) H100 = Hpers;
        }
        H300 = Hpers;
    }
    double se = 0; int m = 0;
    for (int i = std::max(0, n - 50); i < n; ++i) { se += (est[i].head<3>() - kf[i].p).squaredNorm(); ++m; }
    std::printf("%.5f %.5f %.5f %.5f %.4f\n", Hpers, H30, H100, H300, std::sqrt(se / std::max(m, 1)));
    return 0;
}
