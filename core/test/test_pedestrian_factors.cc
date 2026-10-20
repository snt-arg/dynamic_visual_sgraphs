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
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>

#include "PedestrianBA.h"
#include "PedestrianFactors.h"
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
        TryMulti<EdgePedLogRange<SE3>>(e, vi, J) || TryMulti<EdgePedLogRange<VertexPose>>(e, vi, J) || TryMulti<EdgePedTie1>(e, vi, J) ||
        TryMulti<EdgePedTie2>(e, vi, J))
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
    // vertex ownership: the stack objects above are not owned by a graph
    delete pose;
}

int main()
{
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
        try { PedestrianBA::Add(opt, {}, nullptr, true, true); } catch (const std::runtime_error &) { threw = true; }
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
