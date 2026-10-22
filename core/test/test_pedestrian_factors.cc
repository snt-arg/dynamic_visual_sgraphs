// P9-C unit tests for the pedestrian factors (core/include/PedestrianFactors.h, core/src/PedestrianBA.cc).
//   1. analytic Jacobian == central-difference Jacobian, one test per factor (both camera-pose vertex types where the edge is templated)
//   2. absorption: a person observed only by camera-relative factors leaves the camera block of the information matrix unchanged
//      (P2 result as a unit test), and a positive control: motion-prior coupling does reach the camera
//   3. flag off: nothing is created and no file is opened
//   usage: test_pedestrian_factors
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdlib>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "PedestrianBA.h"
#include "PedestrianFactors.h"
#include "PedestrianFloorGate.h"
#include "PedestrianGate.h"
#include "Pedestrians.h"

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

static std::mt19937 rng(7);
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static Eigen::Vector3d V3(double s) { return Eigen::Vector3d(U(-s, s), U(-s, s), U(-s, s)); }

static Eigen::Matrix3d RandRot()
{
    Eigen::Vector3d w = V3(0.6);
    return Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix();
}

// ---- camera pose vertices ---------------------------------------------------------------------------------------
static g2o::VertexSE3Expmap *MakeSE3(const Eigen::Matrix3d &R, const Eigen::Vector3d &t)
{
    auto *v = new g2o::VertexSE3Expmap();
    v->setEstimate(g2o::SE3Quat(R, t));
    return v;
}

static VertexPose *MakeImu(const Eigen::Matrix3d &Rcw, const Eigen::Vector3d &tcw)
{
    // camera-IMU extrinsics: a fixed non-trivial rotation (TESSE-like axis permutation plus offset)
    Eigen::Matrix3d Rbc;
    Rbc << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Vector3d tbc(0.05, -0.02, 0.03);
    auto *v = new VertexPose();
    ImuCamPose p;
    p.pCamera.resize(1, nullptr);
    p.SetParam({Rcw}, {tcw}, {Rbc}, {tbc}, 0.0);
    v->setEstimate(p);
    return v;
}

// ---- generic numerical Jacobian ---------------------------------------------------------------------------------
static Eigen::MatrixXd NumJac(g2o::OptimizableGraph::Edge *e, int vi, double h = 1e-6)
{
    auto *v = static_cast<g2o::OptimizableGraph::Vertex *>(e->vertex(vi));
    const int d = v->dimension();
    e->computeError();
    const int m = e->dimension();
    Eigen::MatrixXd J(m, d);
    for (int k = 0; k < d; ++k)
    {
        Eigen::VectorXd delta = Eigen::VectorXd::Zero(d);
        delta(k) = h;
        v->push();
        v->oplus(delta.data());
        e->computeError();
        Eigen::VectorXd ep = Eigen::Map<const Eigen::VectorXd>(e->errorData(), m);
        v->pop();
        delta(k) = -h;
        v->push();
        v->oplus(delta.data());
        e->computeError();
        Eigen::VectorXd em = Eigen::Map<const Eigen::VectorXd>(e->errorData(), m);
        v->pop();
        J.col(k) = (ep - em) / (2 * h);
    }
    e->computeError();
    return J;
}

template <class E>
static bool TryBinary(g2o::OptimizableGraph::Edge *e, int vi, Eigen::MatrixXd &out)
{
    auto *b = dynamic_cast<E *>(e);
    if (!b)
        return false;
    out = vi == 0 ? Eigen::MatrixXd(b->jacobianOplusXi()) : Eigen::MatrixXd(b->jacobianOplusXj());
    return true;
}

template <class E>
static bool TryUnary(g2o::OptimizableGraph::Edge *e, int, Eigen::MatrixXd &out)
{
    auto *b = dynamic_cast<E *>(e);
    if (!b)
        return false;
    out = Eigen::MatrixXd(b->jacobianOplusXi());
    return true;
}

template <class E>
static bool TryMulti(g2o::OptimizableGraph::Edge *e, int vi, Eigen::MatrixXd &out)
{
    auto *b = dynamic_cast<E *>(e);
    if (!b)
        return false;
    out = b->Jac(vi);
    return true;
}

static Eigen::MatrixXd Analytic(g2o::OptimizableGraph::Edge *e, int vi)
{
    // g2o's Jacobian members are Eigen::Maps into a JacobianWorkspace: allocate it, then call the workspace overload
    g2o::JacobianWorkspace ws;
    ws.updateSize(e);
    ws.allocate();
    e->linearizeOplus(ws);
    Eigen::MatrixXd J;
    typedef g2o::VertexSE3Expmap SE3;
    typedef g2o::VertexSBAPointXYZ Pt;
    if (TryBinary<g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPedState, SE3>>(e, vi, J) ||
        TryBinary<g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexPedState, VertexPose>>(e, vi, J) ||
        TryBinary<g2o::BaseBinaryEdge<2, Eigen::Vector2d, Pt, SE3>>(e, vi, J) ||
        TryBinary<g2o::BaseBinaryEdge<2, Eigen::Vector2d, Pt, VertexPose>>(e, vi, J) || TryBinary<EdgePedWna>(e, vi, J) ||
        TryMulti<EdgePedLogRange<SE3>>(e, vi, J) || TryMulti<EdgePedLogRange<VertexPose>>(e, vi, J) || TryUnary<EdgePedBiasPrior>(e, vi, J) || TryMulti<EdgePedTie1>(e, vi, J) ||
        TryMulti<EdgePedTie2>(e, vi, J) || TryMulti<EdgePedFloorContact<SE3>>(e, vi, J) || TryMulti<EdgePedFloorContact<VertexPose>>(e, vi, J) ||
        TryMulti<EdgePedFloorPx<SE3>>(e, vi, J) || TryMulti<EdgePedFloorPx<VertexPose>>(e, vi, J) ||
        TryMulti<EdgePedLevelPx<SE3>>(e, vi, J) || TryMulti<EdgePedLevelPx<VertexPose>>(e, vi, J) ||
        TryBinary<EdgePedPointOnLevel>(e, vi, J) || TryBinary<EdgePedPlatformHeight<SE3>>(e, vi, J) || TryBinary<EdgePedPlatformHeight<VertexPose>>(e, vi, J))
        return J;
    throw std::runtime_error("unhandled edge type in Analytic()");
}

static void CheckEdge(g2o::OptimizableGraph::Edge *e, const std::string &name, double tol = 1e-5)
{
    for (int vi = 0; vi < static_cast<int>(e->vertices().size()); ++vi)
    {
        const Eigen::MatrixXd A = Analytic(e, vi), N = NumJac(e, vi);
        const double err = (A - N).cwiseAbs().maxCoeff(), scale = std::max(1.0, N.cwiseAbs().maxCoeff());
        std::ostringstream ss;
        ss << name << ": vertex " << vi << " analytic vs numeric Jacobian max |diff| " << err << " (scale " << scale << ")";
        CHECK(err < tol * scale, ss.str());
    }
}

template <class PoseV>
static PoseV *MakePose(const Eigen::Matrix3d &, const Eigen::Vector3d &);
template <>
g2o::VertexSE3Expmap *MakePose<g2o::VertexSE3Expmap>(const Eigen::Matrix3d &R, const Eigen::Vector3d &t) { return MakeSE3(R, t); }
template <>
VertexPose *MakePose<VertexPose>(const Eigen::Matrix3d &R, const Eigen::Vector3d &t) { return MakeImu(R, t); }

template <class PoseV>
static void JacobianTests(const char *tag)
{
    const Eigen::Matrix3d R = RandRot();
    const Eigen::Vector3d t = V3(0.5);
    PoseV *pose = MakePose<PoseV>(R, t);
    const Eigen::Vector3d Xw = R.transpose() * (Eigen::Vector3d(0.4, -0.3, 6.0) - t); // in front of the camera
    VertexPedState state;
    Vec6 x;
    x << Xw, V3(1.0);
    state.setEstimate(x);
    g2o::VertexSBAPointXYZ foot;
    foot.setEstimate(Xw + Eigen::Vector3d(0.2, -0.1, -0.9));

    {
            EdgePedReproj<VertexPedState, PoseV> e(415.7, 415.7, 360.0, 240.0);
            e.setVertex(0, &state);
        e.setVertex(1, pose);
        e.setMeasurement(Eigen::Vector2d(371.0, 232.0));
        CheckEdge(&e, std::string("root reprojection [") + tag + "]");
    }
    {
        EdgePedReproj<g2o::VertexSBAPointXYZ, PoseV> e(415.7, 415.7, 360.0, 240.0);
        e.setVertex(0, &foot);
        e.setVertex(1, pose);
        e.setMeasurement(Eigen::Vector2d(355.0, 290.0));
        CheckEdge(&e, std::string("footprint reprojection [") + tag + "]");
    }
    {
        EdgePedLogRange<PoseV> e;
        e.setVertex(0, &state);
        e.setVertex(1, pose);
        e.setMeasurement(std::log(415.7) + std::log(0.91) + std::log(1.70) - std::log(150.0));
        CheckEdge(&e, std::string("nose-heel log range [") + tag + "]");
    }
    {
        // erratum (P9-C3b): the error is log of the optical-axis depth, not of the ray length
        EdgePedLogRange<PoseV> e;
        e.setVertex(0, &state);
        e.setVertex(1, pose);
        e.setMeasurement(0.3);
        e.computeError();
        const Eigen::Vector3d Xc3 = R * Xw + t;
        CHECK(std::abs(e.error()(0) - (std::log(Xc3(2)) - 0.3)) < 1e-12, std::string("log range [") + tag + "]: error = log z_c - measurement (depth, not ray length)");
        CHECK(std::abs(std::log(Xc3(2)) - std::log(Xc3.norm())) > 1e-3, std::string("log range [") + tag + "]: the test geometry distinguishes z from the ray length");
        // with the per-track bias vertex
        ped::VertexPedBias bias;
        bias.setEstimate(0.04);
        EdgePedLogRange<PoseV> eb(true);
        eb.setVertex(0, &state);
        eb.setVertex(1, pose);
        eb.setVertex(2, &bias);
        eb.setMeasurement(0.3);
        eb.computeError();
        CHECK(std::abs(eb.error()(0) - (std::log(Xc3(2)) - (0.3 + 0.04))) < 1e-12, std::string("log range with bias [") + tag + "]: error = log z_c - (meas + b)");
        CheckEdge(&eb, std::string("nose-heel log depth with bias [") + tag + "]");
        EdgePedBiasPrior pr;
        pr.setVertex(0, &bias);
        pr.setMeasurement(0.0);
        CheckEdge(&pr, std::string("bias prior [") + tag + "]");
    }
    // vertex ownership: the stack objects above are not owned by a graph
    delete pose;
}


// ---- P9-C3 floor contact ------------------------------------------------------------------------------------------
// camera ~1.5 m above the floor z = zf, looking forward-down; foot ray through (u, v); the person's root above the ground point.
template <class PoseV>
static void FloorTests(const char *tag)
{
    const double fx = 415.7, cx = 360.0, cy = 240.0, zf = 0.2;
    // world: z up. Camera at (0.3, -0.2, 1.7), optical axis forward (+x world) pitched 12 deg down, y_cam down.
    const Eigen::Matrix3d Rwc0 = (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished(); // cam x = -world y, cam y = -world z, cam z = world x
    const Eigen::Matrix3d Rwc = Eigen::AngleAxisd(0.21, Eigen::Vector3d::UnitY()).toRotationMatrix() * Rwc0; // rotation about world +y by +0.21: the forward axis (+x) tilts down
    const Eigen::Vector3d C(0.3, -0.2, 1.7);
    const Eigen::Matrix3d Rcw = Rwc.transpose();
    const Eigen::Vector3d tcw = -Rcw * C;
    PoseV *pose = MakePose<PoseV>(Rcw, tcw);
    const double u = 372.0, v = 330.0;
    VertexPedState state;
    Vec6 x;
    x << 3.1, -0.15, 1.15, 0.2, 0.1, 0.0;
    state.setEstimate(x);

    // A: fixed plane
    EdgePedFloorContact<PoseV> A(fx, fx, cx, cy, u, v, 0, zf, 0.036, /*poseGradient=*/true);
    A.setVertex(0, &state);
    A.setVertex(1, pose);
    A.setMeasurement(Eigen::Vector2d(u, v));
    {
        const auto g = A.Geometry();
        CHECK(std::abs(g.G(2) - (zf + 0.036)) < 1e-10, std::string("floor contact A [") + tag + "]: ground point on the plane z = zf + delta");
        // the ground point projects back to the foot pixel
        const Eigen::Vector3d Xc = Rcw * g.G + tcw;
        CHECK(std::abs(fx * Xc(0) / Xc(2) + cx - u) < 1e-8 && std::abs(fx * Xc(1) / Xc(2) + cy - v) < 1e-8,
              std::string("floor contact A [") + tag + "]: ground point lies on the foot ray");
        CHECK(g.s > 0 && g.den < 0, std::string("floor contact A [") + tag + "]: ray points down, depth positive");
        A.computeError();
        CHECK(std::abs(A.error()(0) - (g.G(0) - 3.1)) < 1e-12 && std::abs(A.error()(1) - (g.G(1) + 0.15)) < 1e-12,
              std::string("floor contact A [") + tag + "]: residual = (G - p)_xy");
    }
    CheckEdge(&A, std::string("floor contact A (pose gradient) [") + tag + "]");
    {
        // "pedestrian only": the pose block is exactly zero, the state block is unchanged
        EdgePedFloorContact<PoseV> Ap(fx, fx, cx, cy, u, v, 0, zf, 0.036, /*poseGradient=*/false);
        Ap.setVertex(0, &state);
        Ap.setVertex(1, pose);
        const Eigen::MatrixXd J1 = Analytic(&Ap, 1), J0 = Analytic(&Ap, 0), J0a = Analytic(&A, 0);
        CHECK(J1.cwiseAbs().maxCoeff() == 0.0, std::string("floor contact A pedestrian-only [") + tag + "]: pose Jacobian block is exactly zero");
        CHECK((J0 - J0a).cwiseAbs().maxCoeff() == 0.0, std::string("floor contact A pedestrian-only [") + tag + "]: state block unchanged");
        CHECK(NumJac(&Ap, 1).cwiseAbs().maxCoeff() > 1e-3, std::string("floor contact A pedestrian-only [") + tag + "]: the residual does depend on the pose (stop-gradient is deliberate)");
    }
    // B: k = 4 live points; plane height = mean z
    std::vector<g2o::VertexSBAPointXYZ *> pts;
    const double dz[4] = {0.03, -0.02, 0.05, -0.01};
    for (int i = 0; i < 4; ++i)
    {
        auto *pv = new g2o::VertexSBAPointXYZ();
        pv->setEstimate(Eigen::Vector3d(2.5 + 0.3 * i, -0.4 + 0.2 * i, zf + dz[i]));
        pts.push_back(pv);
    }
    EdgePedFloorContact<PoseV> B(fx, fx, cx, cy, u, v, 4, 99.0, 0.036, /*poseGradient=*/true);
    B.setVertex(0, &state);
    B.setVertex(1, pose);
    for (int i = 0; i < 4; ++i)
        B.setVertex(2 + i, pts[i]);
    B.setMeasurement(Eigen::Vector2d(u, v));
    CHECK(std::abs(B.PlaneHeight() - (zf + (0.03 - 0.02 + 0.05 - 0.01) / 4 + 0.036)) < 1e-12, std::string("floor contact B [") + tag + "]: plane height = mean z of the k points + delta");
    CheckEdge(&B, std::string("floor contact B [") + tag + "]");
    // B reduces to A when the camera and the static points are fixed and every neighbour lies on the floor plane
    for (int i = 0; i < 4; ++i)
        pts[i]->setEstimate(Eigen::Vector3d(2.5 + 0.3 * i, -0.4 + 0.2 * i, zf));
    A.computeError();
    B.computeError();
    CHECK((A.error() - B.error()).cwiseAbs().maxCoeff() < 1e-12, std::string("floor contact B == A [") + tag + "]: same residual when all neighbours are on the plane");
    CHECK((Analytic(&A, 0) - Analytic(&B, 0)).cwiseAbs().maxCoeff() < 1e-12 && (Analytic(&A, 1) - Analytic(&B, 1)).cwiseAbs().maxCoeff() < 1e-12,
          std::string("floor contact B == A [") + tag + "]: same state and pose Jacobians (camera/points fixed = the extra blocks are not used)");
    // lifting the neighbours by dz moves the ground point away from the camera by dz/tan(depression) in the viewing direction (sign check of dG/dzp)
    for (int i = 0; i < 4; ++i)
        pts[i]->setEstimate(Eigen::Vector3d(2.5 + 0.3 * i, -0.4 + 0.2 * i, zf + 0.1));
    {
        const auto gB = B.Geometry(), gA = A.Geometry();
        const Eigen::Vector3d dh = (gB.G - gA.G).template head<2>().homogeneous();
        CHECK(gB.G(2) > gA.G(2) + 0.09 && (gB.G.template head<2>() - C.head<2>()).norm() < (gA.G.template head<2>() - C.head<2>()).norm(),
              std::string("floor contact B [") + tag + "]: a higher plane puts the ground point higher and nearer to the camera");
        (void)dh;
    }
    for (auto *pv : pts)
        delete pv;
    delete pose;
}

static void RegionalFloorTests()
{
    std::mt19937 g(11);
    std::normal_distribution<double> n01(0.0, 1.0);
    std::uniform_real_distribution<double> uni(-3.0, 3.0);
    std::vector<Eigen::Vector3d> pts;
    for (int i = 0; i < 600; ++i) pts.emplace_back(uni(g), uni(g), 0.12 + 0.015 * n01(g));       // floor
    for (int i = 0; i < 150; ++i) pts.emplace_back(uni(g) * 0.3, uni(g) * 0.3, 0.75 + 0.01 * n01(g)); // table top
    for (int i = 0; i < 200; ++i) pts.emplace_back(uni(g), 3.0, 0.2 + 1.8 * std::abs(uni(g)) / 3.0);   // wall
    for (int i = 0; i < 300; ++i) pts.emplace_back(uni(g), uni(g), 2.4 + 0.02 * n01(g));          // ceiling (above the camera: excluded)
    const auto f = PedestrianBA::RegionalFloor(pts, Eigen::Vector2d(0, 0), 1.6, 2.0, 0.05, 0.3, 30, 0.25);
    CHECK(f.ok && std::abs(f.zf - 0.12) < 0.03, "regional floor: the floor mode (not table, wall or ceiling) is found");
    const auto f2 = PedestrianBA::RegionalFloor(pts, Eigen::Vector2d(0, 0), 1.6, 0.01, 0.05, 0.3, 30, 0.25);
    CHECK(!f2.ok, "regional floor: too few points in the region -> not ok");
    const auto f3 = PedestrianBA::RegionalFloor(pts, Eigen::Vector2d(0, 0), 0.05, 2.0, 0.05, 0.3, 30, 0.25);
    CHECK(!f3.ok, "regional floor: camera below every point -> not ok");
    const auto f4 = PedestrianBA::RegionalFloor(pts, Eigen::Vector2d(0, 0), 1.6, 2.0, 0.05, 0.3, 30, 0.25);
    CHECK(f4.zf == f.zf && f4.nMode == f.nMode, "regional floor: deterministic");
    {   // a scene where the table top has more points than the floor: the highest mode is the table, the lowest significant mode is the floor
        std::vector<Eigen::Vector3d> q2;
        for (int i = 0; i < 150; ++i) q2.emplace_back(uni(g) * 0.5, uni(g) * 0.5, 0.75 + 0.01 * n01(g));
        for (int i = 0; i < 90; ++i) q2.emplace_back(uni(g), uni(g), 0.05 + 0.01 * n01(g));
        const auto hi = PedestrianBA::RegionalFloor(q2, Eigen::Vector2d(0, 0), 1.6, 3.0, 0.05, 0.3, 30, 1.0);
        const auto lo = PedestrianBA::RegionalFloor(q2, Eigen::Vector2d(0, 0), 1.6, 3.0, 0.05, 0.3, 30, 0.25);
        CHECK(hi.ok && std::abs(hi.zf - 0.75) < 0.03, "regional floor: q = 1 takes the highest mode (the table)");
        CHECK(lo.ok && std::abs(lo.zf - 0.05) < 0.03, "regional floor: q = 0.25 takes the lowest significant mode (the floor)");
    }
    auto shuffled = pts;
    std::shuffle(shuffled.begin(), shuffled.end(), g);
    const auto f5 = PedestrianBA::RegionalFloor(shuffled, Eigen::Vector2d(0, 0), 1.6, 2.0, 0.05, 0.3, 30, 0.25);
    CHECK(f5.zf == f.zf, "regional floor: independent of the point order");
}

static void FloorIoTests()
{
    // reader: F rows
    char dirt[] = "/tmp/p9c3_reader_XXXXXX";
    char *dir = mkdtemp(dirt);
    CHECK(dir != nullptr, "temporary directory");
    PedestrianParams::Get() = PedestrianParams();
    PedestrianParams::Get().enabled = true;
    const std::string d(dir);
    auto write = [&](const std::string &name, const std::string &body) {
        std::FILE *f = std::fopen((d + "/" + name).c_str(), "w");
        std::fputs(body.c_str(), f);
        std::fclose(f);
    };
    write("0000000000001000000.ped.txt", "# stamp 1.0 image 0\nP 3 100.0 200.0 50.0 120.0\nP 7 300.0 210.0 0.0 90.0\nF 7 301.5 400.25 0.8\n");
    PedestrianFrameReader rd(d);
    const auto fr = rd.Read(1.0);
    CHECK(fr->obs.size() == 2 && !fr->obs[0].hasFoot && fr->obs[1].hasFoot && fr->obs[1].footU == 301.5 && fr->obs[1].footV == 400.25 && fr->obs[1].footScore == 0.8,
          "reader: an F row attaches the lowest-foot pixel to the P row of the same track");
    write("0000000000002000000.ped.txt", "P 3 100.0 200.0 50.0 120.0\nF 9 301.5 400.25 0.8\n");
    bool threw = false;
    try { rd.Read(2.0); } catch (const std::runtime_error &) { threw = true; }
    CHECK(threw, "reader: an F row without a P row of the same track throws");
    write("0000000000003000000.ped.txt", "P 3 100.0 200.0 50.0 120.0\nF 3 1 2 0.8\nF 3 1 2 0.8\n");
    threw = false;
    try { rd.Read(3.0); } catch (const std::runtime_error &) { threw = true; }
    CHECK(threw, "reader: two F rows for one track throw");
    PedestrianParams::Get() = PedestrianParams();
    // settings
    const std::string base =
        "%YAML:1.0\n---\nPedestrians.Enabled: 1\nPedestrians.RootPxSigma: 1.5\nPedestrians.LogDepthSigma: 0.031\nPedestrians.SkeletonOverStature: 0.9027\n"
        "Pedestrians.StatureFixedM: 1.7\nPedestrians.Qv: 0.1386\nPedestrians.FootprintPxSigma: 2.0\nPedestrians.T1Sigma: 0.02\nPedestrians.T2Sigma: 0.1\n"
        "Pedestrians.CRoot: 0.55\nPedestrians.CLeg: 0.55\nPedestrians.MinTrackKeyframes: 3\nPedestrians.MinStanceBoxPx: 80\nPedestrians.MinFootObservations: 2\n"
        "Pedestrians.MinLpx: 0.0\nPedestrians.HuberScale: 1.0\nPedestrians.MaskHeightM: 1.773\nPedestrians.GateCenter: -0.0043\nPedestrians.GateSigma: 0.0417\n"
        "Pedestrians.GateK: 0.0\nPedestrians.MotionPrior: isotropic\nPedestrians.InnovGate: 0\nPedestrians.Dir: /x\n";
    const std::string fl = "Pedestrians.FloorSigmaRadialM: 0.4\nPedestrians.FloorSigmaTangentialM: 0.2\nPedestrians.FloorRegionRadiusM: 3.0\nPedestrians.FloorMinPoints: 30\nPedestrians.FloorLowModeFraction: 0.25\nPedestrians.FloorContactOffsetM: 0.036\nPedestrians.BiasSigma: 0.042\nPedestrians.BiasMinContacts: 0\n"
                           "Pedestrians.FloorBinM: 0.05\nPedestrians.FloorBelowCameraM: 0.3\nPedestrians.FloorPlausM: 0.3\n";
    auto cfg = [&](const std::string &extra) {
        cv::FileStorage fs(base + extra, cv::FileStorage::READ | cv::FileStorage::MEMORY);
        return PedestrianParams::Configure(fs);
    };
    threw = false;
    try { cfg(""); } catch (const std::runtime_error &) { threw = true; }
    CHECK(threw, "settings: Pedestrians.FloorContact is required when the flag is on (no default)");
    CHECK(cfg("Pedestrians.FloorContact: none\n") && PedestrianParams::Get().floorMode == PedestrianParams::FloorMode::None, "settings: FloorContact none reads no floor key");
    threw = false;
    try { cfg("Pedestrians.FloorContact: regional\n"); } catch (const std::runtime_error &) { threw = true; }
    CHECK(threw, "settings: FloorContact regional with missing sigma keys throws");
    CHECK(cfg("Pedestrians.FloorContact: regional\n" + fl) && PedestrianParams::Get().floorMode == PedestrianParams::FloorMode::Regional, "settings: regional");
    threw = false;
    try { cfg("Pedestrians.FloorContact: joint\n" + fl); } catch (const std::runtime_error &) { threw = true; }
    CHECK(threw, "settings: joint without FloorNeighbours / radius / band throws");
    CHECK(cfg("Pedestrians.FloorContact: joint\n" + fl + "Pedestrians.FloorNeighbours: 5\nPedestrians.FloorNeighbourRadiusM: 1.0\nPedestrians.FloorBandM: 0.15\n") &&
              PedestrianParams::Get().floorMode == PedestrianParams::FloorMode::Joint && PedestrianParams::Get().floorNeighbours == 5,
          "settings: joint");
    threw = false;
    try { cfg("Pedestrians.FloorContact: bogus\n"); } catch (const std::runtime_error &) { threw = true; }
    CHECK(threw, "settings: an unknown FloorContact value throws");
    PedestrianParams::Get() = PedestrianParams();
}

// ---- P9-C4: anisotropic WNA prior, innovation gate, settings ---------------------------------------------------------
static void C4Tests()
{
    // 1. anisotropic WNA: information == Q^-1 with Q = diag(qh, qh, qz) kron [[dt^3/3, dt^2/2],[dt^2/2, dt]] (state order p(3), v(3))
    {
        const double dt = 0.37, qh = 0.1383, qz = 0.0096;
        VertexPedState a, b;
        Vec6 xa, xb;
        xa << V3(2), V3(1);
        xb << V3(2), V3(1);
        a.setEstimate(xa);
        b.setEstimate(xb);
        EdgePedWna e;
        e.setVertex(0, &a);
        e.setVertex(1, &b);
        e.setMeasurement(Vec6::Zero());
        e.SetDt(dt, Eigen::Vector3d(qh, qh, qz));
        CheckEdge(&e, "anisotropic white-noise-acceleration prior");
        const Eigen::Vector3d q(qh, qh, qz);
        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
        for (int k = 0; k < 3; ++k)
        {
            Q(k, k) = q(k) * dt * dt * dt / 3;
            Q(k, 3 + k) = Q(3 + k, k) = q(k) * dt * dt / 2;
            Q(3 + k, 3 + k) = q(k) * dt;
        }
        const Eigen::Matrix<double, 6, 6> Qinv = Q.inverse();
        CHECK((e.information() - Qinv).cwiseAbs().maxCoeff() / Qinv.cwiseAbs().maxCoeff() < 1e-12, "aniso WNA: information == Q^-1 with Q = diag(q_h, q_h, q_z) (x) [[dt^3/3, dt^2/2],[dt^2/2, dt]]");
        CHECK((e.information() * Q - Eigen::Matrix<double, 6, 6>::Identity()).cwiseAbs().maxCoeff() < 1e-10, "aniso WNA: information * Q == I");
        // the z axis is stiffer by qh/qz exactly; x and y are identical
        CHECK(std::abs(e.information()(2, 2) / e.information()(0, 0) - qh / qz) < 1e-9 && e.information()(0, 0) == e.information()(1, 1), "aniso WNA: z block = (q_h / q_z) x the horizontal block; x == y");
        // isotropic path bit-identical to the pre-C4 closed form
        EdgePedWna iso;
        iso.SetDt(dt, 0.1386);
        const double q0 = 0.1386;
        Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Zero();
        const double A = 12.0 / (q0 * dt * dt * dt), B = -6.0 / (q0 * dt * dt), C = 4.0 / (q0 * dt);
        I.topLeftCorner<3, 3>() = A * Eigen::Matrix3d::Identity();
        I.topRightCorner<3, 3>() = B * Eigen::Matrix3d::Identity();
        I.bottomLeftCorner<3, 3>() = B * Eigen::Matrix3d::Identity();
        I.bottomRightCorner<3, 3>() = C * Eigen::Matrix3d::Identity();
        CHECK((iso.information() - I).cwiseAbs().maxCoeff() == 0.0, "isotropic WNA information is bit-identical to the pre-C4 closed form");
    }
    // 2. settings
    {
        const std::string base =
            "%YAML:1.0\n---\nPedestrians.Enabled: 1\nPedestrians.RootPxSigma: 1.5\nPedestrians.LogDepthSigma: 0.031\nPedestrians.SkeletonOverStature: 0.9027\n"
            "Pedestrians.StatureFixedM: 1.7\nPedestrians.Qv: 0.1386\nPedestrians.FootprintPxSigma: 2.0\nPedestrians.T1Sigma: 0.02\nPedestrians.T2Sigma: 0.1\n"
            "Pedestrians.CRoot: 0.55\nPedestrians.CLeg: 0.55\nPedestrians.MinTrackKeyframes: 3\nPedestrians.MinStanceBoxPx: 80\nPedestrians.MinFootObservations: 2\n"
            "Pedestrians.MinLpx: 0.0\nPedestrians.HuberScale: 1.0\nPedestrians.MaskHeightM: 1.773\nPedestrians.GateCenter: -0.0043\nPedestrians.GateSigma: 0.0417\n"
            "Pedestrians.GateK: 0.0\nPedestrians.FloorContact: none\nPedestrians.Dir: /x\n";
        const std::string gate = "Pedestrians.InnovGate: 1\nPedestrians.InnovChi2Dof2: 9.21034\nPedestrians.InnovChi2Dof3: 11.34487\nPedestrians.InnovFreeKeyframes: 2\n"
                                 "Pedestrians.InnovReinitRejections: 3\nPedestrians.InnovP0PosXY: 0.2\nPedestrians.InnovP0PosZ: 0.06\nPedestrians.InnovP0VelXY: 0.19\nPedestrians.InnovP0VelZ: 0.08\n"
                                 "Pedestrians.InnovScaleK3: 0.5\nPedestrians.InnovScaleK4: 0.8\nPedestrians.InnovScaleK5: 1.5\nPedestrians.InnovScaleK6: 2.0\n";
        auto cfg = [&](const std::string &extra) {
            cv::FileStorage fs(base + extra, cv::FileStorage::READ | cv::FileStorage::MEMORY);
            return PedestrianParams::Configure(fs);
        };
        auto throws = [&](const std::string &extra) {
            try { cfg(extra); } catch (const std::runtime_error &) { return true; }
            return false;
        };
        CHECK(throws(""), "C4 settings: Pedestrians.MotionPrior is required (no default)");
        CHECK(throws("Pedestrians.MotionPrior: isotropic\n"), "C4 settings: Pedestrians.InnovGate is required (no default)");
        CHECK(cfg("Pedestrians.MotionPrior: isotropic\nPedestrians.InnovGate: 0\n") && PedestrianParams::Get().motionPrior == PedestrianParams::MotionPrior::Isotropic && !PedestrianParams::Get().innovGate,
              "C4 settings: isotropic, gate off reads no other key");
        CHECK(throws("Pedestrians.MotionPrior: anisotropic\nPedestrians.InnovGate: 0\n"), "C4 settings: anisotropic without QvH / QvZ throws");
        CHECK(cfg("Pedestrians.MotionPrior: anisotropic\nPedestrians.QvH: 0.1383\nPedestrians.QvZ: 0.0096\nPedestrians.InnovGate: 0\n") && PedestrianParams::Get().qvH == 0.1383 && PedestrianParams::Get().qvZ == 0.0096,
              "C4 settings: anisotropic");
        CHECK(throws("Pedestrians.MotionPrior: bogus\nPedestrians.InnovGate: 0\n"), "C4 settings: unknown MotionPrior throws");
        CHECK(throws("Pedestrians.MotionPrior: isotropic\nPedestrians.InnovGate: 1\n"), "C4 settings: gate on without its keys throws");
        CHECK(cfg("Pedestrians.MotionPrior: isotropic\n" + gate) && PedestrianParams::Get().innovGate && !PedestrianParams::Get().innovShadow && PedestrianParams::Get().innovReinitRejections == 3 &&
                  PedestrianParams::Get().innovScale[0] == 0.5 && PedestrianParams::Get().innovScale[3] == 2.0,
              "C4 settings: gate on, with the s_k scales");
        {
            std::string shadow = gate;
            shadow.replace(shadow.find("Pedestrians.InnovGate: 1"), 24, "Pedestrians.InnovGate: 2");
            CHECK(cfg("Pedestrians.MotionPrior: isotropic\n" + shadow) && PedestrianParams::Get().innovGate && PedestrianParams::Get().innovShadow, "C4 settings: InnovGate 2 = shadow (log only)");
            std::string noscale = gate.substr(0, gate.find("Pedestrians.InnovScaleK3"));
            CHECK(throws("Pedestrians.MotionPrior: isotropic\n" + noscale), "C4 settings: gate on without InnovScaleK* throws");
            CHECK(throws("Pedestrians.MotionPrior: isotropic\nPedestrians.InnovGate: 3\n"), "C4 settings: InnovGate 3 throws");
        }
        PedestrianParams::Get() = PedestrianParams();
    }
    // 3. innovation gate: analytic cases of the production test (PedestrianGate.h)
    {
        PedestrianParams P;
        P.rootPxSigma = 1.5;
        P.logDepthSigma = 0.031;
        P.qv = 0.1386;
        P.motionPrior = PedestrianParams::MotionPrior::Isotropic;
        P.innovChi2Dof2 = 9.21034;
        P.innovChi2Dof3 = 11.34487;
        P.innovFreeKeyframes = 2;
        P.innovReinitRejections = 3;
        P.innovP0PosXY = 0.2;
        P.innovP0PosZ = 0.06;
        P.innovP0VelXY = 0.19;
        P.innovP0VelZ = 0.08;
        const double fx = 415.7, fy = 415.7, cx = 360, cy = 240;
        const Eigen::Matrix3d Rcw = Eigen::Matrix3d::Identity();
        const Eigen::Vector3d tcw(0, 0, 0);
        const Eigen::Vector3d p(1.0, 0.2, 6.0), v(0.3, -0.1, 0.0);
        const double dt = 0.2;
        const Eigen::Vector3d ph = p + v * dt;
        const double u0 = fx * ph(0) / ph(2) + cx, v0 = fy * ph(1) / ph(2) + cy, lz0 = std::log(ph(2));
        GateTest t0 = InnovationTest(P, p, v, dt, Rcw, tcw, fx, fy, cx, cy, u0, v0, true, lz0);
        CHECK(t0.ok && t0.dof == 3 && t0.d2 < 1e-18 && !t0.reject, "gate: a measurement equal to the prediction has d^2 = 0 and is kept (dof 3)");
        GateTest t2 = InnovationTest(P, p, v, dt, Rcw, tcw, fx, fy, cx, cy, u0, v0, false, 123.0);
        CHECK(t2.ok && t2.dof == 2 && t2.d2 < 1e-18, "gate: without a range measurement dof = 2 and the log-range value is not read");
        // d^2 of a pure pixel offset equals the analytic value: y^T S^-1 y with S = J Sigma_pp J^T + R
        const double du = 7.0;
        GateTest t1 = InnovationTest(P, p, v, dt, Rcw, tcw, fx, fy, cx, cy, u0 + du, v0, true, lz0);
        Eigen::Matrix3d Jc = Eigen::Matrix3d::Zero();
        Jc(0, 0) = fx / ph(2); Jc(0, 2) = -fx * ph(0) / (ph(2) * ph(2)); Jc(1, 1) = fy / ph(2); Jc(1, 2) = -fy * ph(1) / (ph(2) * ph(2)); Jc(2, 2) = 1.0 / ph(2);
        Eigen::Matrix3d Spp = Eigen::Matrix3d::Zero();   // F P0 F^T + Q, position block, per axis
        for (int a = 0; a < 3; ++a)
        {
            const double sp = a < 2 ? P.innovP0PosXY : P.innovP0PosZ, sv = a < 2 ? P.innovP0VelXY : P.innovP0VelZ;
            Spp(a, a) = sp * sp + dt * dt * sv * sv + P.qv * dt * dt * dt / 3.0;
        }
        Eigen::Matrix3d S = Jc * Spp * Jc.transpose();
        S.diagonal() += Eigen::Vector3d(1.5 * 1.5, 1.5 * 1.5, 0.031 * 0.031);
        const Eigen::Vector3d y(du, 0, 0);
        const double d2 = y.dot(S.inverse() * y);
        CHECK(std::abs(t1.d2 - d2) < 1e-9 * d2, "gate: d^2 of a pixel offset equals y^T (J (F P0 F^T + Q) J^T + R)^-1 y");
        // the threshold: scale the offset until d^2 crosses chi2(0.99, 3); the rejection flips exactly there
        const double s_cross = std::sqrt(P.innovChi2Dof3 / d2) * du;
        CHECK(!InnovationTest(P, p, v, dt, Rcw, tcw, fx, fy, cx, cy, u0 + 0.999 * s_cross, v0, true, lz0).reject && InnovationTest(P, p, v, dt, Rcw, tcw, fx, fy, cx, cy, u0 + 1.001 * s_cross, v0, true, lz0).reject,
              "gate: rejection flips at d^2 = chi2(0.99, 3)");
        // s_k scales the covariance: the same offset is rejected at scale 1 and kept at a scale that moves the threshold above its d^2
        {
            const double dd = InnovationTest(P, p, v, dt, Rcw, tcw, fx, fy, cx, cy, u0 + 1.001 * s_cross, v0, true, lz0).d2;
            CHECK(InnovationTest(P, p, v, dt, Rcw, tcw, fx, fy, cx, cy, u0 + 1.001 * s_cross, v0, true, lz0, 1.0).reject && !InnovationTest(P, p, v, dt, Rcw, tcw, fx, fy, cx, cy, u0 + 1.001 * s_cross, v0, true, lz0, 1.01 * dd / P.innovChi2Dof3).reject &&
                      InnovationTest(P, p, v, dt, Rcw, tcw, fx, fy, cx, cy, u0 + 1.001 * s_cross, v0, true, lz0, 0.5).reject,
                  "gate: the scale s_k moves the rejection threshold (d^2 itself is unscaled)");
        }
        // anisotropic Q is the active prior: a vertical offset is judged with q_z, not q_h
        PedestrianParams Pa = P;
        Pa.motionPrior = PedestrianParams::MotionPrior::Anisotropic;
        Pa.qvH = 0.1383;
        Pa.qvZ = 0.0096;
        const double dv_px = 5.0;
        CHECK(InnovationTest(Pa, p, v, 2.0, Rcw, tcw, fx, fy, cx, cy, u0, v0 + dv_px, false, 0).d2 != InnovationTest(P, p, v, 2.0, Rcw, tcw, fx, fy, cx, cy, u0, v0 + dv_px, false, 0).d2,
              "gate: the active (anisotropic) Q enters the covariance");
        CHECK(!InnovationTest(P, p, v, dt, Rcw, tcw, fx, fy, cx, cy, u0, v0, true, lz0).reject && !InnovationTest(P, Eigen::Vector3d(0, 0, -1), v, dt, Rcw, tcw, fx, fy, cx, cy, 0, 0, true, 0).ok,
              "gate: a prediction behind the image plane is not testable (kept, counted)");
        // chain: free keyframes, rejection, re-initialisation after 3 consecutive rejections
        GateChain ch;
        GateTest rej;
        rej.ok = true; rej.dof = 3; rej.d2 = 99; rej.reject = true;
        GateTest acc = rej; acc.d2 = 1; acc.reject = false;
        PedGateRec r;
        r = GateStep(P, ch, true, rej, 0.2, 0);
        CHECK(r.status == PedGateRec::ACCEPTED && !r.tested && ch.idx == 1, "gate chain: the first keyframe is not gated (even with an outlier innovation)");
        r = GateStep(P, ch, true, rej, 0.2, 0);
        CHECK(r.status == PedGateRec::ACCEPTED && !r.tested && ch.idx == 2, "gate chain: the second keyframe is not gated");
        r = GateStep(P, ch, true, rej, 0.2, 0);
        CHECK(r.status == PedGateRec::REJECTED && r.tested && ch.idx == 2 && ch.consecRej == 1, "gate chain: the third keyframe is gated and rejected");
        r = GateStep(P, ch, true, acc, 0.4, 0);
        CHECK(r.status == PedGateRec::ACCEPTED && ch.idx == 3 && ch.consecRej == 0, "gate chain: an accepted measurement resets the rejection run");
        GateStep(P, ch, true, rej, 0.2, 0);
        r = GateStep(P, ch, true, rej, 0.2, 0);
        CHECK(r.status == PedGateRec::REJECTED && ch.consecRej == 2, "gate chain: two consecutive rejections do not re-initialise");
        r = GateStep(P, ch, true, rej, 0.2, 0);
        CHECK(r.status == PedGateRec::REINIT && ch.idx == 1 && ch.consecRej == 0, "gate chain: the third consecutive rejection re-initialises (new chain, idx = 1)");
        r = GateStep(P, ch, true, rej, 0.2, 0);
        CHECK(r.status == PedGateRec::ACCEPTED && !r.tested && ch.idx == 2, "gate chain: the new chain's second measurement is again free");
        GateTest notOk;
        r = GateStep(P, ch, false, notOk, 0.2, 0);
        CHECK(r.status == PedGateRec::ACCEPTED && !r.tested, "gate chain: no written-back predecessor -> accepted untested");
    }
    PedestrianParams::Get() = PedestrianParams();
}

// ---- P9-C3c: midpoint floor contact (image-space residual), gates, settings -------------------------------------------
template <class PoseV>
static void FloorPxTests(const char *tag)
{
    const double fx = 415.7, cx = 360.0, cy = 240.0, zf = 0.2, dv = 0.053;
    const Eigen::Matrix3d Rwc0 = (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();
    const Eigen::Matrix3d Rwc = Eigen::AngleAxisd(0.21, Eigen::Vector3d::UnitY()).toRotationMatrix() * Rwc0;
    const Eigen::Vector3d C(0.3, -0.2, 1.7);
    const Eigen::Matrix3d Rcw = Rwc.transpose();
    const Eigen::Vector3d tcw = -Rcw * C;
    PoseV *pose = MakePose<PoseV>(Rcw, tcw);
    const double u = 372.0, v = 330.0;
    VertexPedState state;
    Vec6 x;
    x << 3.1, -0.15, 1.15, 0.2, 0.1, 0.0;
    state.setEstimate(x);
    EdgePedFloorPx<PoseV> A(fx, fx, cx, cy, u, v, 0, zf, dv, /*poseGradient=*/true);
    A.setVertex(0, &state);
    A.setVertex(1, pose);
    A.computeError();
    const Eigen::Vector3d Xc = Rcw * Eigen::Vector3d(3.1, -0.15, zf + dv) + tcw;
    CHECK(std::abs(A.error()(0) - (u - (fx * Xc(0) / Xc(2) + cx))) < 1e-12 && std::abs(A.error()(1) - (v - (fx * Xc(1) / Xc(2) + cy))) < 1e-12,
          std::string("floor px A' [") + tag + "]: residual = contact pixel - pi(R (p_x, p_y, z_f + delta_v) + t)");
    CheckEdge(&A, std::string("floor px A' (pose gradient) [") + tag + "]");
    CHECK(Analytic(&A, 0).col(2).cwiseAbs().maxCoeff() == 0.0 && Analytic(&A, 0).template rightCols<3>().cwiseAbs().maxCoeff() == 0.0, std::string("floor px [") + tag + "]: the root height and the velocity do not enter the factor");
    EdgePedFloorPx<PoseV> Ap(fx, fx, cx, cy, u, v, 0, zf, dv, /*poseGradient=*/false);
    Ap.setVertex(0, &state);
    Ap.setVertex(1, pose);
    CHECK(Analytic(&Ap, 1).cwiseAbs().maxCoeff() == 0.0 && (Analytic(&Ap, 0) - Analytic(&A, 0)).cwiseAbs().maxCoeff() == 0.0, std::string("floor px A' pedestrian-only [") + tag + "]: pose block exactly zero, state block unchanged");
    CHECK(NumJac(&Ap, 1).cwiseAbs().maxCoeff() > 1e-3, std::string("floor px A' pedestrian-only [") + tag + "]: the residual does depend on the pose");
    std::vector<g2o::VertexSBAPointXYZ *> pts;
    const double dz[5] = {0.03, -0.02, 0.05, -0.01, 0.0};
    for (int i = 0; i < 5; ++i)
    {
        auto *pv = new g2o::VertexSBAPointXYZ();
        pv->setEstimate(Eigen::Vector3d(2.5 + 0.3 * i, -0.4 + 0.2 * i, zf + dz[i]));
        pts.push_back(pv);
    }
    EdgePedFloorPx<PoseV> B(fx, fx, cx, cy, u, v, 5, 99.0, dv, /*poseGradient=*/true);
    B.setVertex(0, &state);
    B.setVertex(1, pose);
    for (int i = 0; i < 5; ++i)
        B.setVertex(2 + i, pts[i]);
    CHECK(std::abs(B.FloorHeight() - (zf + 0.05 / 5)) < 1e-12, std::string("floor px B' [") + tag + "]: floor height = mean z of the k points");
    CheckEdge(&B, std::string("floor px B' [") + tag + "]");
    for (int i = 0; i < 5; ++i)
        pts[i]->setEstimate(Eigen::Vector3d(2.5 + 0.3 * i, -0.4 + 0.2 * i, zf));
    A.computeError();
    B.computeError();
    CHECK((A.error() - B.error()).cwiseAbs().maxCoeff() < 1e-12 && (Analytic(&A, 0) - Analytic(&B, 0)).cwiseAbs().maxCoeff() < 1e-12 && (Analytic(&A, 1) - Analytic(&B, 1)).cwiseAbs().maxCoeff() < 1e-12,
          std::string("floor px B' == A' [") + tag + "]: same residual and state / pose Jacobians when every neighbour lies on the floor plane");
    for (auto *pv : pts)
        delete pv;
    delete pose;
}

static void FloorPxGateTests()
{
    PedestrianParams P;
    P.floorRaisedMaxM = 0.10;
    P.floorMinElevDeg = 8.0;
    P.floorNeighbours = 5;
    P.floorPlaneSigmaMaxM = 0.02;
    CHECK(ped::FloorStageA(P, 0.05, 0.0, 12.0) == 0, "floor px gates: a contact 5 cm above the global floor mode at 12 deg passes");
    CHECK(ped::FloorStageA(P, 0.101, 0.0, 12.0) == 2 && ped::FloorStageA(P, 0.099, 0.0, 12.0) == 0, "floor px gates: raised-surface rejection flips at 0.10 m above the global lowest floor mode");
    CHECK(ped::FloorStageA(P, 0.0, 0.0, 7.99) == 3 && ped::FloorStageA(P, 0.0, 0.0, 8.0) == 0, "floor px gates: the elevation gate flips at FloorMinElevDeg");
    CHECK(ped::FloorStageA(P, 0.0, 0.0, -3.0) == 3, "floor px gates: a ray above the horizon is rejected");
    CHECK(ped::FloorStageA(P, 0.5, 0.0, 2.0) == 2, "floor px gates: the raised-surface stage is decided first");
    CHECK(ped::FloorStageB(P, 4, 0.001) == 4 && ped::FloorStageB(P, 5, 0.001) == 0, "floor px B' gates: at least 5 qualifying points");
    CHECK(ped::FloorStageB(P, 20, 0.0199) == 0 && ped::FloorStageB(P, 20, 0.02) == 5, "floor px B' gates: plane scatter must be below 2 cm");
}

static void FloorPxSettingsTests()
{
    const std::string base =
        "%YAML:1.0\n---\nPedestrians.Enabled: 1\nPedestrians.RootPxSigma: 1.5\nPedestrians.LogDepthSigma: 0.031\nPedestrians.SkeletonOverStature: 0.855387\n"
        "Pedestrians.StatureFixedM: 1.7\nPedestrians.Qv: 0.1386\nPedestrians.FootprintPxSigma: 2.0\nPedestrians.T1Sigma: 0.02\nPedestrians.T2Sigma: 0.1\n"
        "Pedestrians.CRoot: 0.55\nPedestrians.CLeg: 0.55\nPedestrians.MinTrackKeyframes: 3\nPedestrians.MinStanceBoxPx: 80\nPedestrians.MinFootObservations: 2\n"
        "Pedestrians.MinLpx: 0.0\nPedestrians.HuberScale: 1.0\nPedestrians.MaskHeightM: 1.773\nPedestrians.GateCenter: -0.0043\nPedestrians.GateSigma: 0.0417\n"
        "Pedestrians.GateK: 0.0\nPedestrians.MotionPrior: isotropic\nPedestrians.InnovGate: 0\nPedestrians.Dir: /x\n";
    const std::string common = "Pedestrians.FloorRegionRadiusM: 6.0\nPedestrians.FloorMinPoints: 30\nPedestrians.FloorLowModeFraction: 0.25\nPedestrians.BiasSigma: 0.0422\nPedestrians.BiasMinContacts: 0\n"
                               "Pedestrians.FloorBinM: 0.05\nPedestrians.FloorBelowCameraM: 0.3\nPedestrians.FloorPlausM: 0.3\n";
    const std::string px = "Pedestrians.FloorMidDeltaM: 0.053\nPedestrians.FloorSigmaPxU: 6.0\nPedestrians.FloorSigmaPxV: 3.0\nPedestrians.FloorZfBiasM: 0.02\nPedestrians.FloorRaisedMaxM: 0.10\n"
                           "Pedestrians.FloorMinElevDeg: 8.0\nPedestrians.FloorPlaneSigmaMaxM: 0.02\n";
    const std::string nb = "Pedestrians.FloorNeighbours: 5\nPedestrians.FloorNeighbourRadiusM: 1.0\nPedestrians.FloorBandM: 0.05\n";
    auto cfg = [&](const std::string &extra) {
        cv::FileStorage fs(base + extra, cv::FileStorage::READ | cv::FileStorage::MEMORY);
        return PedestrianParams::Configure(fs);
    };
    auto throws = [&](const std::string &extra) {
        try { cfg(extra); } catch (const std::runtime_error &) { return true; }
        return false;
    };
    CHECK(cfg("Pedestrians.FloorContact: regional_px\n" + common + px) && PedestrianParams::Get().floorMode == PedestrianParams::FloorMode::RegionalPx && PedestrianParams::Get().floorMidDeltaM == 0.053 &&
              PedestrianParams::Get().floorZfBiasM == 0.02,
          "C3c settings: regional_px reads the midpoint keys and no range-space key");
    CHECK(throws("Pedestrians.FloorContact: regional_px\n" + common), "C3c settings: regional_px without the midpoint keys throws");
    CHECK(throws("Pedestrians.FloorContact: joint_px\n" + common + px), "C3c settings: joint_px without FloorNeighbours / radius / band throws");
    CHECK(cfg("Pedestrians.FloorContact: joint_px\n" + common + px + nb) && PedestrianParams::Get().floorMode == PedestrianParams::FloorMode::JointPx && PedestrianParams::Get().floorNeighbours == 5, "C3c settings: joint_px");
    CHECK(cfg("Pedestrians.FloorContact: shadow\n" + common + px + nb) && PedestrianParams::Get().floorMode == PedestrianParams::FloorMode::Shadow, "C3c settings: shadow");
    CHECK(cfg("Pedestrians.FloorContact: none\n") && PedestrianParams::Get().floorMode == PedestrianParams::FloorMode::None, "C3c settings: none reads no floor key");
    PedestrianParams::Get() = PedestrianParams();
}

// ---- P9-C5: floor-level factors F1 (contact), F2 (point on level), F3 (platform height) -------------------------------
template <class PoseV>
static void LevelTests(const char *tag)
{
    const double fx = 415.7, fy = 413.2, cx = 360.0, cy = 240.0, zl = 0.2, delta = 0.053;
    const Eigen::Matrix3d Rwc0 = (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();
    const Eigen::Matrix3d Rwc = Eigen::AngleAxisd(0.21, Eigen::Vector3d::UnitY()).toRotationMatrix() * Rwc0;
    const Eigen::Vector3d C(0.3, -0.2, 1.7);
    const Eigen::Matrix3d Rcw = Rwc.transpose();
    PoseV *pose = MakePose<PoseV>(Rcw, -Rcw * C);
    VertexPedState state;
    Vec6 x;
    x << 3.1, -0.15, 1.15, 0.2, 0.1, 0.0;
    state.setEstimate(x);
    VertexFloorLevel lvl;
    lvl.setEstimate(zl);

    // F1: the residual vanishes when the pixel is the projection of (p_x, p_y, z_l + delta); Jacobians agree with central differences
    const Eigen::Vector3d X = Rcw * Eigen::Vector3d(3.1, -0.15, zl + delta) + (-Rcw * C);
    const double u = fx * X(0) / X(2) + cx, v = fy * X(1) / X(2) + cy;
    EdgePedLevelPx<PoseV> f1(fx, fy, cx, cy, u + 4.0, v - 3.0, delta);
    f1.setVertex(0, &state);
    f1.setVertex(1, pose);
    f1.setVertex(2, &lvl);
    f1.computeError();
    CHECK(std::abs(f1.error()(0) - 4.0) < 1e-9 && std::abs(f1.error()(1) + 3.0) < 1e-9, std::string("F1 level contact [") + tag + "]: e = contact pixel - pi(F), F = (p_x, p_y, z_l + delta_v)");
    CheckEdge(&f1, std::string("F1 level contact [") + tag + "]");
    CHECK(Analytic(&f1, 2).cwiseAbs().maxCoeff() > 0.1, std::string("F1 level contact [") + tag + "]: the level block is non-trivial");
    lvl.setEstimate(zl + 0.3);
    CheckEdge(&f1, std::string("F1 level contact, other level [") + tag + "]");

    // F2: e = z_point - z_l
    g2o::VertexSBAPointXYZ pt;
    pt.setEstimate(Eigen::Vector3d(1.0, 2.0, 0.27));
    lvl.setEstimate(zl);
    EdgePedPointOnLevel f2;
    f2.setVertex(0, &pt);
    f2.setVertex(1, &lvl);
    f2.setMeasurement(0.0);
    f2.computeError();
    CHECK(std::abs(f2.error()(0) - 0.07) < 1e-12, std::string("F2 point on level [") + tag + "]: e = z_point - z_l");
    CheckEdge(&f2, std::string("F2 point on level [") + tag + "]");

    // F3: e = C_z - z_l - h_cam
    EdgePedPlatformHeight<PoseV> f3;
    f3.setVertex(0, pose);
    f3.setVertex(1, &lvl);
    f3.setMeasurement(1.4895);
    f3.computeError();
    CHECK(std::abs(f3.error()(0) - (1.7 - zl - 1.4895)) < 1e-12, std::string("F3 platform height [") + tag + "]: e = C_z - z_l - h_cam");
    CheckEdge(&f3, std::string("F3 platform height [") + tag + "]");
    delete pose;
}

int main()
{
    LevelTests<g2o::VertexSE3Expmap>("SE3");
    LevelTests<VertexPose>("VertexPose");
    FloorPxTests<g2o::VertexSE3Expmap>("SE3");
    FloorPxTests<VertexPose>("VertexPose");
    FloorPxGateTests();
    FloorPxSettingsTests();
    C4Tests();
    FloorTests<g2o::VertexSE3Expmap>("SE3");
    FloorTests<VertexPose>("VertexPose");
    RegionalFloorTests();
    FloorIoTests();
    // ------------------------------------------------------------------------------------------------ 1. Jacobians
    JacobianTests<g2o::VertexSE3Expmap>("SE3");
    JacobianTests<VertexPose>("VertexPose");
    {
        VertexPedState a, b;
        Vec6 xa, xb;
        xa << V3(2), V3(1);
        xb << V3(2), V3(1);
        a.setEstimate(xa);
        b.setEstimate(xb);
        EdgePedWna e;
        e.setVertex(0, &a);
        e.setVertex(1, &b);
        e.setMeasurement(Vec6::Zero());
        e.SetDt(0.37, 0.14);
        CheckEdge(&e, "white-noise-acceleration prior");
        // information is the inverse of Q = q kron([[dt^3/3, dt^2/2],[dt^2/2, dt]], I3)
        const double dt = 0.37, q = 0.14;
        Eigen::Matrix<double, 6, 6> Q;
        Q << dt * dt * dt / 3 * Eigen::Matrix3d::Identity(), dt * dt / 2 * Eigen::Matrix3d::Identity(),
            dt * dt / 2 * Eigen::Matrix3d::Identity(), dt * Eigen::Matrix3d::Identity();
        Q *= q;
        CHECK((e.information() * Q - Eigen::Matrix<double, 6, 6>::Identity()).cwiseAbs().maxCoeff() < 1e-10, "WNA information == Q^-1 (closed form)");
    }
    {
        VertexPedState s;
        Vec6 x;
        x << 3.0, 2.0, 1.1, 0.0, 0.0, 0.0;
        s.setEstimate(x);
        g2o::VertexSBAPointXYZ f;
        f.setEstimate(Eigen::Vector3d(2.6, 1.7, 0.2));
        EdgePedTie1 t1(0.55, 1.72);
        t1.setVertex(0, &s);
        t1.setVertex(1, &f);
        t1.setMeasurement(0.0);
        CheckEdge(&t1, "tie T1");
        EdgePedTie2 t2(0.55, 1.72); // horizontal offset 0.5 m < c_leg H0 = 0.946 m: inactive (hinge) -> exactly zero Jacobian
        t2.setVertex(0, &s);
        t2.setVertex(1, &f);
        t2.setMeasurement(0.0);
        CheckEdge(&t2, "tie T2, inactive branch");
        f.setEstimate(Eigen::Vector3d(1.5, 0.9, 0.2)); // horizontal offset 1.7 m: active
        CheckEdge(&t2, "tie T2, active branch");
        t2.computeError();
        CHECK(t2.error()(0) > 0.5, "tie T2 active: positive hinge error");
    }

    // ------------------------------------------------------------------------------------------------ 2. absorption
    // Camera block of the information matrix after eliminating everything else, with and without a person.
    //   cameras K=5 (SE3), landmarks M=40 observed by all cameras (EdgePedReproj<PointXYZ> as the static-feature factor),
    //   person: one state per camera, root reprojection + log range (camera-relative factors only; the stature is a constant).
    {
        const int K = 5, M = 40;
        const double fx = 415.7, cx = 360.0, cy = 240.0;
        std::vector<g2o::VertexSE3Expmap *> cams;
        for (int k = 0; k < K; ++k)
            cams.push_back(MakeSE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
        for (int k = 0; k < K; ++k)
        {
            Eigen::Matrix3d R = Eigen::AngleAxisd(0.05 * k, Eigen::Vector3d::UnitY()).toRotationMatrix();
            cams[k]->setEstimate(g2o::SE3Quat(R, Eigen::Vector3d(-0.3 * k, 0.02 * k, 0.0)));
        }
        std::vector<g2o::VertexSBAPointXYZ *> pts;
        for (int m = 0; m < M; ++m)
        {
            auto *p = new g2o::VertexSBAPointXYZ();
            p->setEstimate(Eigen::Vector3d(U(-3, 3), U(-2, 2), U(4, 12)));
            pts.push_back(p);
        }
        std::vector<VertexPedState *> ps;
        for (int k = 0; k < K; ++k)
        {
            auto *s = new VertexPedState();
            Vec6 x;
            x << Eigen::Vector3d(1.0 + 0.1 * k, 0.2, 6.0), Eigen::Vector3d::Zero();
            s->setEstimate(x);
            ps.push_back(s);
        }
        // dense assembly: variable offsets
        const int nC = 6 * K, nP = 3 * M, nS = 6 * K, N = nC + nP + nS;
        auto offC = [&](int k) { return 6 * k; };
        auto offP = [&](int m) { return nC + 3 * m; };
        auto offS = [&](int k) { return nC + nP + 6 * k; };
        auto addEdge = [&](Eigen::MatrixXd &Hm, g2o::OptimizableGraph::Edge *e, const std::vector<int> &offs) {
            e->computeError();
            const int m = e->dimension();
            Eigen::MatrixXd Om = Eigen::Map<const Eigen::MatrixXd>(e->informationData(), m, m);
            std::vector<Eigen::MatrixXd> J;
            for (size_t i = 0; i < offs.size(); ++i)
                J.push_back(Analytic(e, static_cast<int>(i)));
            for (size_t i = 0; i < offs.size(); ++i)
                for (size_t j = 0; j < offs.size(); ++j)
                    Hm.block(offs[i], offs[j], J[i].cols(), J[j].cols()) += J[i].transpose() * Om * J[j];
        };
        Eigen::MatrixXd H0 = Eigen::MatrixXd::Zero(N, N), H1 = Eigen::MatrixXd::Zero(N, N);
        for (int k = 0; k < K; ++k)
            for (int m = 0; m < M; ++m)
            {
                EdgePedReproj<g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap> e(fx, fx, cx, cy);
                e.setVertex(0, pts[m]);
                e.setVertex(1, cams[k]);
                const Eigen::Vector3d Xc = cams[k]->estimate().map(pts[m]->estimate());
                e.setMeasurement(Eigen::Vector2d(fx * Xc(0) / Xc(2) + cx, fx * Xc(1) / Xc(2) + cy));
                e.setInformation(Eigen::Matrix2d::Identity() / (1.5 * 1.5));
                addEdge(H0, &e, {offP(m), offC(k)});
                addEdge(H1, &e, {offP(m), offC(k)});
            }
        // gauge: fix camera 0 (drop its rows/columns below by eliminating nothing from it; the comparison is on the full camera block)
        Eigen::MatrixXd Hc = H1;
        for (int k = 0; k < K; ++k)
        {
            EdgePedReproj<VertexPedState, g2o::VertexSE3Expmap> er(fx, fx, cx, cy);
            er.setVertex(0, ps[k]);
            er.setVertex(1, cams[k]);
            const Eigen::Vector3d Xc = cams[k]->estimate().map(ps[k]->p());
            er.setMeasurement(Eigen::Vector2d(fx * Xc(0) / Xc(2) + cx + 0.4, fx * Xc(1) / Xc(2) + cy - 0.3)); // non-zero residual
            er.setInformation(Eigen::Matrix2d::Identity() / (1.5 * 1.5));
            addEdge(H1, &er, {offS(k), offC(k)});
            EdgePedLogRange<g2o::VertexSE3Expmap> el;
            el.setVertex(0, ps[k]);
            el.setVertex(1, cams[k]);
            el.setMeasurement(std::log(Xc.norm()) + 0.02);
            el.setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (0.031 * 0.031));
            addEdge(H1, &el, {offS(k), offC(k)});
        }

        auto cameraBlock = [&](const Eigen::MatrixXd &Hm, bool withPerson) {
            const int nKeep = nC;
            const int nElim = N - nC;
            Eigen::MatrixXd Hee = Hm.bottomRightCorner(nElim, nElim);
            const Eigen::MatrixXd Hce = Hm.topRightCorner(nKeep, nElim);
            // tiny ridge on the person variables only (their velocity components have no factor at all)
            for (int i = nP; i < nElim; ++i)
                Hee(i, i) += withPerson ? 1e-12 : 0.0;
            if (!withPerson)
            {
                // landmark-only reference: eliminate only the landmark block
                Eigen::MatrixXd Hpp = Hm.block(nC, nC, nP, nP);
                return Eigen::MatrixXd(Hm.topLeftCorner(nKeep, nKeep) - Hm.block(0, nC, nKeep, nP) * Hpp.ldlt().solve(Hm.block(nC, 0, nP, nKeep)));
            }
            return Eigen::MatrixXd(Hm.topLeftCorner(nKeep, nKeep) - Hce * Hee.ldlt().solve(Hce.transpose()));
        };
        const Eigen::MatrixXd I0 = cameraBlock(H0, false), I1 = cameraBlock(H1, true);
        const double rel = (I1 - I0).cwiseAbs().maxCoeff() / I0.cwiseAbs().maxCoeff();
        std::ostringstream ss;
        ss << "absorption: camera information block changes by " << rel << " (relative max-norm) when a camera-relative-only person is added";
        CHECK(rel < 1e-9, ss.str());

        // positive control: add the motion prior between consecutive states; the camera block must now change
        Eigen::MatrixXd H2 = H1;
        for (int k = 0; k + 1 < K; ++k)
        {
            EdgePedWna w;
            w.setVertex(0, ps[k]);
            w.setVertex(1, ps[k + 1]);
            w.setMeasurement(Vec6::Zero());
            w.SetDt(0.5, 0.14);
            addEdge(H2, &w, {offS(k), offS(k + 1)});
        }
        const Eigen::MatrixXd I2 = cameraBlock(H2, true);
        const double rel2 = (I2 - I0).cwiseAbs().maxCoeff() / I0.cwiseAbs().maxCoeff();
        std::ostringstream s2;
        s2 << "positive control: with the motion prior the camera block changes by " << rel2 << " (must be > 1e-6)";
        CHECK(rel2 > 1e-6, s2.str());
    }

    // ------------------------------------------------------------------------------------------------ 3. flag off
    {
        PedestrianParams::Get() = PedestrianParams(); // Enabled = false
        g2o::SparseOptimizer opt;
        bool threw = false;
        try { PedestrianBA::Add(opt, {}, nullptr, true, true, 0); } catch (const std::runtime_error &) { threw = true; }
        CHECK(threw, "flag off: PedestrianBA::Add refuses to run");
        threw = false;
        try { PedestrianFrameReader r("/nonexistent"); } catch (const std::runtime_error &) { threw = true; }
        CHECK(threw, "flag off: PedestrianFrameReader cannot be constructed (no pedestrian file can be opened)");
        long v, e;
        PedestrianBA::Counters(v, e);
        CHECK(v == 0 && e == 0, "flag off: no person vertex or edge was ever created");
        cv::FileStorage fs("%YAML:1.0\n---\nPedestrians.Enabled: 0\n", cv::FileStorage::READ | cv::FileStorage::MEMORY);
        CHECK(!PedestrianParams::Configure(fs), "flag off: Configure() with Pedestrians.Enabled: 0 reads no other key");
        cv::FileStorage fs2("%YAML:1.0\n---\nPedestrians.Enabled: 1\n", cv::FileStorage::READ | cv::FileStorage::MEMORY);
        threw = false;
        try { PedestrianParams::Configure(fs2); } catch (const std::runtime_error &) { threw = true; }
        CHECK(threw, "flag on with a missing key: Configure() throws (no defaults)");
        PedestrianParams::Get() = PedestrianParams();
    }

    std::cout << (failures ? "FAILED" : "ALL PASSED") << " (" << failures << " failures)" << std::endl;
    return failures ? 1 : 0;
}
