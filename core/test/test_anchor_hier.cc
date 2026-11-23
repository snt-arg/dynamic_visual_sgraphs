// Hierarchical stature anchor (Pedestrians.ScaleAnchorMode: hier; order of 23 Nov 2026), gate 1 unit tests.
// Toy: a monocular map (8 keyframes on a line, 80 static points, noise-free reprojection edges, keyframe 0 fixed), so its scale is set by the anchor alone.
// Pedestrian observations: per track j and observation i, z_metric = z_true * exp(kappa_true + eta_j + eps_ji); measurement = log z_metric (row ratio 1).
//   0. EdgePedScaleAnchorH: analytic Jacobians == central differences.
//   1. kappa: all tracks share kappa_true = log 1.06 (symmetric per-track offsets and per-observation noise, so the population median is exactly kappa_true): after one inactive call
//      (no edges: fewer than 5 tracks known) the store's kappa is within 0.5 % of +6 %; a noisy population (200 tracks, sigma_person 0.173, sigma_obs 0.072) is reported, not gated.
//   2. with kappa set, the hier solve leaves the map scale unchanged (|s - 1| < 0.5 %).
//   3. the map is scaled by 1.10 after kappa is set: the hier solve (kappa held) pulls the scale back to within 1 % of 1.
//   Reported, not gated: the fixed anchor (sigma 0.05, no kappa, no eta) from the same +10 % start, which pulls the scale to the absolute stature reference (about 1.06).
// usage: test_anchor_hier
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <random>
#include <vector>
#include <Eigen/Dense>

#include "PedAnchorKappa.h"
#include "PedestrianFactors.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

using namespace ORB_SLAM3;
using namespace ORB_SLAM3::ped;

static int failures = 0;
#define CHECK(cond, msg)                                 \
    do                                                   \
    {                                                    \
        if (!(cond))                                     \
        {                                                \
            std::cerr << "FAIL: " << msg << std::endl;   \
            ++failures;                                  \
        }                                                \
        else                                             \
            std::cout << "ok:   " << msg << std::endl;   \
    } while (0)

static const double FX = 500, FY = 500, CX = 320, CY = 240, SIG_OBS = 0.0718, SIG_PERSON = 0.1733, KAPPA_TRUE = std::log(1.06);
static const int NKF = 8, NPT = 80;

struct Obs
{
    int track, kf, pt;
    double meas; // log z_metric
};

struct World
{
    std::vector<Eigen::Vector3d> c;   // camera centres (identity rotation)
    std::vector<Eigen::Vector3d> X;   // points
};

static World TrueWorld()
{
    World w;
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> ux(-3, 6), uy(-1, 1.5), uz(3, 10);
    for (int k = 0; k < NKF; ++k)
        w.c.push_back(Eigen::Vector3d(0.4 * k, 0, 0));
    for (int i = 0; i < NPT; ++i)
        w.X.push_back(Eigen::Vector3d(ux(rng), uy(rng), uz(rng)));
    return w;
}

static World Scaled(const World &w, double s)
{
    World o = w;
    for (auto &c : o.c)
        c *= s;
    for (auto &x : o.X)
        x *= s;
    return o;
}

// symmetric design: per-track offsets and per-observation noise with median exactly 0
static std::vector<Obs> SymObs(const World &w)
{
    const double eta[10] = {-0.22, -0.16, -0.10, -0.04, 0, 0, 0.04, 0.10, 0.16, 0.22};
    const double eps[6] = {-0.09, -0.05, -0.02, 0.02, 0.05, 0.09};
    std::vector<Obs> o;
    for (int j = 0; j < 10; ++j)
        for (int i = 0; i < 6; ++i)
        {
            const int kf = (j + i) % NKF, pt = (j * 7 + i * 3) % NPT;
            const double z = w.X[pt](2) - w.c[kf](2);
            o.push_back({100 + j, kf, pt, std::log(z) + KAPPA_TRUE + eta[j] + eps[(i + j) % 6]});
        }
    return o;
}

struct Graph
{
    g2o::SparseOptimizer opt;
    std::vector<g2o::VertexSE3Expmap *> kf;
    std::vector<g2o::VertexSBAPointXYZ *> pt;
};

static void Build(Graph &G, const World &truth, const World &init)
{
    auto *ls = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    auto *bs = new g2o::BlockSolverX(ls);
    G.opt.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(bs));
    for (int k = 0; k < NKF; ++k)
    {
        auto *v = new g2o::VertexSE3Expmap();
        v->setId(k);
        v->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), -init.c[k]));
        v->setFixed(k == 0);
        G.opt.addVertex(v);
        G.kf.push_back(v);
    }
    for (int i = 0; i < NPT; ++i)
    {
        auto *v = new g2o::VertexSBAPointXYZ();
        v->setId(NKF + i);
        v->setEstimate(init.X[i]);
        v->setMarginalized(true);
        G.opt.addVertex(v);
        G.pt.push_back(v);
    }
    for (int k = 0; k < NKF; ++k)
        for (int i = 0; i < NPT; ++i)
        {
            const Eigen::Vector3d p = truth.X[i] - truth.c[k];
            auto *e = new g2o::EdgeSE3ProjectXYZ();
            e->setVertex(0, G.pt[i]);
            e->setVertex(1, G.kf[k]);
            e->setMeasurement(Eigen::Vector2d(FX * p(0) / p(2) + CX, FY * p(1) / p(2) + CY));
            e->setInformation(Eigen::Matrix2d::Identity());
            e->fx = FX, e->fy = FY, e->cx = CX, e->cy = CY;
            G.opt.addEdge(e);
        }
}

static double LogZc(const Graph &G, const Obs &o)
{
    const Eigen::Vector3d p = G.kf[o.kf]->estimate().map(G.pt[o.pt]->estimate());
    return std::log(p(2));
}

static double MapScale(const Graph &G, const World &truth)
{
    auto centre = [](const g2o::VertexSE3Expmap *v) { return Eigen::Vector3d(v->estimate().inverse().translation()); };
    return (centre(G.kf[NKF - 1]) - centre(G.kf[0])).norm() / (truth.c[NKF - 1] - truth.c[0]).norm();
}

// per-track median x = meas - log z_c at the current estimates (what WriteBack feeds the store)
static std::map<int, double> TrackMedians(const Graph &G, const std::vector<Obs> &obs)
{
    std::map<int, std::vector<double>> xs;
    for (const Obs &o : obs)
        xs[o.track].push_back(o.meas - LogZc(G, o));
    std::map<int, double> m;
    for (auto &kv : xs)
        m[kv.first] = MedianOf(kv.second);
    return m;
}

// the hier edges of one call, as PedestrianBA::AddImpl builds them (eta per track, prior N(0, sigma_person^2), Huber 1.345 sigma_obs)
static void AddHier(Graph &G, const std::vector<Obs> &obs, double kappa)
{
    int id = NKF + NPT;
    std::map<int, VertexPedBias *> eta;
    for (const Obs &o : obs)
        if (!eta.count(o.track))
        {
            auto *v = new VertexPedBias();
            v->setId(id++);
            v->setEstimate(0.0);
            G.opt.addVertex(v);
            auto *pr = new EdgePedBiasPrior();
            pr->setVertex(0, v);
            pr->setMeasurement(0.0);
            pr->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (SIG_PERSON * SIG_PERSON));
            G.opt.addEdge(pr);
            eta[o.track] = v;
        }
    for (const Obs &o : obs)
    {
        auto *e = new EdgePedScaleAnchorH<g2o::VertexSE3Expmap>();
        e->setVertex(0, G.pt[o.pt]);
        e->setVertex(1, G.kf[o.kf]);
        e->setVertex(2, eta[o.track]);
        e->setMeasurement(o.meas - kappa);
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (SIG_OBS * SIG_OBS));
        auto *rk = new g2o::RobustKernelHuber();
        rk->setDelta(1.345);
        e->setRobustKernel(rk);
        G.opt.addEdge(e);
    }
}

static void AddFixed(Graph &G, const std::vector<Obs> &obs, double sig)
{
    for (const Obs &o : obs)
    {
        auto *e = new EdgePedScaleAnchor<g2o::VertexSE3Expmap>();
        e->setVertex(0, G.pt[o.pt]);
        e->setVertex(1, G.kf[o.kf]);
        e->setMeasurement(o.meas);
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (sig * sig));
        auto *rk = new g2o::RobustKernelHuber();
        rk->setDelta(1.345);
        e->setRobustKernel(rk);
        G.opt.addEdge(e);
    }
}

static void Solve(Graph &G)
{
    G.opt.initializeOptimization();
    G.opt.optimize(50);
}

static void TestJacobians()
{
    auto *pt = new g2o::VertexSBAPointXYZ();
    pt->setEstimate(Eigen::Vector3d(0.3, -0.2, 4.0));
    auto *kf = new g2o::VertexSE3Expmap();
    kf->setEstimate(g2o::SE3Quat(Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d(0.2, 1, 0.1).normalized())), Eigen::Vector3d(0.1, 0.05, -0.2)));
    auto *et = new VertexPedBias();
    et->setEstimate(0.07);
    EdgePedScaleAnchorH<g2o::VertexSE3Expmap> e;
    e.setVertex(0, pt);
    e.setVertex(1, kf);
    e.setVertex(2, et);
    e.setMeasurement(1.2);
    e.setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    g2o::JacobianWorkspace ws; // g2o's Jacobian members are Eigen::Maps into a workspace (as test_pedestrian_factors.cc Analytic)
    ws.updateSize(&e);
    ws.allocate();
    static_cast<g2o::OptimizableGraph::Edge &>(e).linearizeOplus(ws); // base-class overload (hidden by the override)
    const double h = 1e-6;
    double maxd = 0;
    g2o::OptimizableGraph::Vertex *vs[3] = {pt, kf, et};
    for (int vi = 0; vi < 3; ++vi)
    {
        const int dim = vs[vi]->dimension();
        Eigen::MatrixXd J = e.Jac(vi);
        for (int d = 0; d < dim; ++d)
        {
            std::vector<double> u(dim, 0.0);
            u[d] = h;
            vs[vi]->push();
            vs[vi]->oplus(u.data());
            e.computeError();
            const double ep = e.error()(0);
            vs[vi]->pop();
            u[d] = -h;
            vs[vi]->push();
            vs[vi]->oplus(u.data());
            e.computeError();
            const double em = e.error()(0);
            vs[vi]->pop();
            maxd = std::max(maxd, std::abs((ep - em) / (2 * h) - J(0, d)));
        }
    }
    CHECK(maxd < 1e-6, "EdgePedScaleAnchorH: analytic Jacobians == central differences (max |diff| " << maxd << ")");
    delete pt;
    delete kf;
    delete et;
}

int main()
{
    TestJacobians();
    const World truth = TrueWorld();
    const std::vector<Obs> obs = SymObs(truth);

    // 1. kappa: call 1 is inactive (no edges: the store is empty), its observations feed kappa at the (true-scale) estimates
    AnchorKappaStore store;
    CHECK(store.N() < 5, "store empty before the first call: anchor inactive");
    {
        Graph G;
        Build(G, truth, truth);
        Solve(G);
        store.Update(TrackMedians(G, obs));
    }
    const double k1 = store.Kappa();
    std::printf("kappa after the first call: %.6f (exp - 1 = %+.4f %%), tracks %d\n", k1, 100 * (std::exp(k1) - 1), store.N());
    CHECK(store.N() >= 5 && std::abs(std::exp(k1) / 1.06 - 1) < 0.005, "kappa converges to the shared +6 % offset within 0.5 %");

    // reported: noisy population
    {
        std::mt19937 rng(7);
        std::normal_distribution<double> np(0, SIG_PERSON), no(0, SIG_OBS);
        AnchorKappaStore s2;
        for (int j = 0; j < 200; ++j)
        {
            const double e = np(rng);
            std::vector<double> x;
            for (int i = 0; i < 6; ++i)
                x.push_back(KAPPA_TRUE + e + no(rng));
            s2.Update({{j, MedianOf(x)}});
        }
        std::printf("reported: noisy population (200 tracks, 6 obs each): kappa %+.4f %% against +6.0000 %%\n", 100 * (std::exp(s2.Kappa()) - 1));
    }

    // 2. kappa set: the hier solve leaves the scale unchanged
    {
        Graph G;
        Build(G, truth, truth);
        AddHier(G, obs, k1);
        Solve(G);
        const double s = MapScale(G, truth);
        std::printf("hier solve from the true scale: map scale %.6f\n", s);
        CHECK(std::abs(s - 1) < 0.005, "with kappa set the hier solve leaves the map scale unchanged (|s - 1| < 0.5 %)");
        store.Update(TrackMedians(G, obs));
        std::printf("kappa after the second call: %+.4f %%\n", 100 * (std::exp(store.Kappa()) - 1));
    }

    // 3. +10 % map scale change after kappa is set: pulled back
    {
        Graph G;
        Build(G, truth, Scaled(truth, 1.10));
        std::printf("start scale %.6f\n", MapScale(G, truth));
        AddHier(G, obs, store.Kappa());
        Solve(G);
        const double s = MapScale(G, truth);
        std::printf("hier solve from +10 %%: map scale %.6f\n", s);
        CHECK(std::abs(s - 1) < 0.01, "a +10 % map scale change after kappa is set is pulled back to within 1 % of the original scale");
    }

    // reported: fixed anchor from the same start
    {
        Graph G;
        Build(G, truth, Scaled(truth, 1.10));
        AddFixed(G, obs, 0.05);
        Solve(G);
        std::printf("reported: fixed anchor (sigma 0.05) from +10 %%: map scale %.6f (the absolute stature reference is 1.06)\n", MapScale(G, truth));
    }

    // carry through a similarity and reset
    {
        AnchorKappaStore s3;
        s3.Update({{1, 0.10}, {2, 0.20}, {3, 0.30}});
        s3.Shift(-std::log(1.5));
        CHECK(std::abs(s3.Kappa() - (0.20 - std::log(1.5))) < 1e-12, "similarity s: every stored x and kappa shift by -log s");
        s3.Update({{2, 0.0}});
        CHECK(s3.N() == 3, "a track is counted once (its latest value replaces the earlier one)");
        s3.Clear();
        CHECK(s3.N() == 0 && s3.Kappa() == 0.0, "reset clears the store");
    }

    std::cout << (failures ? "FAILED " : "PASSED ") << failures << std::endl;
    return failures ? 1 : 0;
}
