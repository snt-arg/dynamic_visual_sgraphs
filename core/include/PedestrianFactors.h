// P9-C: g2o vertices and edges for pedestrians in ORB-SLAM3's local BA. Header only; the unit test
// (core/test/test_pedestrian_factors.cc) checks every analytic Jacobian against central differences.
//
// Frames and conventions
//   * Xc = Rcw Xw + tcw. The camera pose vertex is either g2o::VertexSE3Expmap (LocalBundleAdjustment, update = exp([w;u]) * T)
//     or ORB_SLAM3::VertexPose (LocalInertialBA, camera index 0, update = ImuCamPose::Update). `PoseTraits<V>` gives
//     (Rcw, tcw) and dXc/d(pose update) for both.
//   * Person root state: VertexPedState = (p, v), world frame. Footprint: g2o::VertexSBAPointXYZ. The stature is NOT a state: it is the constant H0
//     (P9-C2 5d: a free per-track stature is not identifiable from monocular measurements and the motion prior shrinks it; see step5d_report.md).
//   * Pinhole camera, no distortion (checked by PedestrianBA at graph build).
//
// Provenance of the math (reuse of object_motion_estimator, branch p9c-eval, include/object_motion_estimator/):
//   EdgePedRootReproj    <- EdgeMonoObject (object_reprojection_factor.hpp:43,57,90); same residual obs - pi(Xc), SE3deriv
//   EdgePedLogRange      <- EdgeObjectRangeBearing (object_range_bearing_factor.hpp:68-150), re-expressed as a log-range
//                           measurement on the nose->heel length (P7-A A3) and on a world-frame state instead of H_jk
//   EdgePedWna           : new edge (white-noise-acceleration kernel, P7-A p7a_lib.py:213 dwna_q2 / addendum_A8.md:5-6)
//   EdgePedFootRoot T1,T2: new edges (P7-A p7a_lib.py:293,298; criterion_G.md:80-81)
#ifndef PEDESTRIAN_FACTORS_H
#define PEDESTRIAN_FACTORS_H

#include <cmath>
#include <istream>
#include <stdexcept>
#include <ostream>

#include <Eigen/Dense>

#include "G2oTypes.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/types/types_sba.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

namespace ORB_SLAM3
{
    namespace ped
    {
        typedef Eigen::Matrix<double, 6, 1> Vec6;

        inline Eigen::Matrix3d Skew(const Eigen::Vector3d &w)
        {
            Eigen::Matrix3d S;
            S << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
            return S;
        }

        // ---------------------------------------------------------------- camera pose traits
        template <class V>
        struct PoseTraits;

        template <>
        struct PoseTraits<g2o::VertexSE3Expmap>
        {
            static void Get(const g2o::VertexSE3Expmap *v, Eigen::Matrix3d &Rcw, Eigen::Vector3d &tcw)
            {
                Rcw = v->estimate().rotation().toRotationMatrix();
                tcw = v->estimate().translation();
            }
            // Xc' = exp([w;u]) * Xc = Xc + w x Xc + u  =>  dXc/d[w;u] = [-skew(Xc) | I]
            static Eigen::Matrix<double, 3, 6> dXc(const g2o::VertexSE3Expmap *, const Eigen::Vector3d &Xc)
            {
                Eigen::Matrix<double, 3, 6> J;
                J.leftCols<3>() = -Skew(Xc);
                J.rightCols<3>().setIdentity();
                return J;
            }
        };

        template <>
        struct PoseTraits<VertexPose>
        {
            static void Get(const VertexPose *v, Eigen::Matrix3d &Rcw, Eigen::Vector3d &tcw)
            {
                Rcw = v->estimate().Rcw[0];
                tcw = v->estimate().tcw[0];
            }
            // As EdgeMono::linearizeOplus: dXc/d[ur;ut] = -Rcb * SE3deriv(Xb), Xb = Rbc Xc + tbc (cf. G2oTypes.cc EdgeMono)
            static Eigen::Matrix<double, 3, 6> dXc(const VertexPose *v, const Eigen::Vector3d &Xc)
            {
                const Eigen::Matrix3d &Rcb = v->estimate().Rcb[0];
                const Eigen::Matrix3d &Rbc = v->estimate().Rbc[0];
                const Eigen::Vector3d &tbc = v->estimate().tbc[0];
                const Eigen::Vector3d Xb = Rbc * Xc + tbc;
                Eigen::Matrix<double, 3, 6> SE3deriv;
                const double x = Xb(0), y = Xb(1), z = Xb(2);
                SE3deriv << 0.0, z, -y, 1.0, 0.0, 0.0,
                    -z, 0.0, x, 0.0, 1.0, 0.0,
                    y, -x, 0.0, 0.0, 0.0, 1.0;
                return -Rcb * SE3deriv;
            }
        };

        // ---------------------------------------------------------------- vertices
        // Root position and velocity (world frame), additive update.
        class VertexPedState : public g2o::BaseVertex<6, Vec6>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            VertexPedState() {}
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }
            void setToOriginImpl() override { _estimate.setZero(); }
            void oplusImpl(const double *u) override { _estimate += Eigen::Map<const Vec6>(u); }
            Eigen::Vector3d p() const { return _estimate.head<3>(); }
            Eigen::Vector3d v() const { return _estimate.tail<3>(); }
        };

        // Per-track log-scale bias b_j (P9-C3b): one scalar per track, persistent between BA calls like the stature of P9-C2 5b/5c was. Additive update.
        class VertexPedBias : public g2o::BaseVertex<1, double>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            VertexPedBias() {}
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }
            void setToOriginImpl() override { _estimate = 0.0; }
            void oplusImpl(const double *u) override { _estimate += u[0]; }
        };

        // N(0, sigma_b^2) prior on b: e = b (information 1 / sigma_b^2 set by the caller).
        class EdgePedBiasPrior : public g2o::BaseUnaryEdge<1, double, VertexPedBias>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgePedBiasPrior() {}
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }
            void computeError() override { this->_error(0) = static_cast<const VertexPedBias *>(this->_vertices[0])->estimate() - _measurement; }
            void linearizeOplus() override { this->_jacobianOplusXi(0, 0) = 1.0; }
        };

        inline Eigen::Vector3d PosOf(const VertexPedState *v) { return v->p(); }
        inline Eigen::Vector3d PosOf(const g2o::VertexSBAPointXYZ *v) { return v->estimate(); }

        // ---------------------------------------------------------------- pixel reprojection of a world point
        // PV = VertexPedState (root, sigma = rootPxSigma) or g2o::VertexSBAPointXYZ (footprint, sigma = footPxSigma).
        // Vertices: (0) point/state, (1) camera pose. e = obs - pi(Xc). Same sign convention as ORB_SLAM3::EdgeMono.
        template <class PV, class PoseV>
        class EdgePedReproj : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, PV, PoseV>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgePedReproj(double fx_, double fy_, double cx_, double cy_) : fx(fx_), fy(fy_), cx(cx_), cy(cy_) {}
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            Eigen::Vector3d Xc(Eigen::Matrix3d *Rcw_out = nullptr) const
            {
                const PV *P = static_cast<const PV *>(this->_vertices[0]);
                const PoseV *C = static_cast<const PoseV *>(this->_vertices[1]);
                Eigen::Matrix3d Rcw;
                Eigen::Vector3d tcw;
                PoseTraits<PoseV>::Get(C, Rcw, tcw);
                if (Rcw_out)
                    *Rcw_out = Rcw;
                return Rcw * PosOf(P) + tcw;
            }
            bool isDepthPositive() const { return Xc()(2) > 1e-3; }

            void computeError() override
            {
                const Eigen::Vector3d X = Xc();
                const double z = X(2) > 1e-6 ? X(2) : 1e-6;
                this->_error << this->_measurement(0) - (fx * X(0) / z + cx), this->_measurement(1) - (fy * X(1) / z + cy);
            }

            void linearizeOplus() override
            {
                Eigen::Matrix3d Rcw;
                const Eigen::Vector3d X = Xc(&Rcw);
                const double z = X(2) > 1e-6 ? X(2) : 1e-6;
                Eigen::Matrix<double, 2, 3> dpi; // d pi / d Xc
                dpi << fx / z, 0.0, -fx * X(0) / (z * z), 0.0, fy / z, -fy * X(1) / (z * z);
                const Eigen::Matrix<double, 2, 3> Jp = -dpi * Rcw;
                this->_jacobianOplusXi.setZero();
                this->_jacobianOplusXi.template leftCols<3>() = Jp;
                this->_jacobianOplusXj = -dpi * PoseTraits<PoseV>::dXc(static_cast<const PoseV *>(this->_vertices[1]), X);
            }

            double fx, fy, cx, cy;
        };

        // ---------------------------------------------------------------- nose->heel log-depth (P9-C3b erratum: depth z, not ray length)
        // Vertices: (0) VertexPedState, (1) camera pose [, (2) VertexPedBias, Variants A / B only].
        //   e = log z_c - (log f + log cL + log H0 [+ b] - log l_px),   z_c = (Rcw p + tcw)_z   the optical-axis depth of the root
        // The apparent length of a vertical body extent L at depth z is l_px = f L / z -- NOT f L / |Xc| (P9-C3 check 0a: r/z = 1.06 median, 1.22 at p90; with the ray
        // length the per-track scale spread is 8.9 % and depends on the image column, with z it is 4.2 % and does not). cL is the z-model value (constants_z.json).
        // _measurement = log f + log cL + log H0 - log l_px.   With the bias vertex b (log-scale offset of the track, b = log(H_track / H0)) the model is z_c = e^b f cL H0 / l_px.
        template <class PoseV>
        class EdgePedLogRange : public g2o::BaseMultiEdge<1, double>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            explicit EdgePedLogRange(bool withBias = false) : biased(withBias) { this->resize(withBias ? 3 : 2); }
            Eigen::MatrixXd Jac(int i) const { return this->_jacobianOplus[i]; } // read access for the unit test
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            Eigen::Vector3d Xc(Eigen::Matrix3d *Rcw_out = nullptr) const
            {
                const VertexPedState *S = static_cast<const VertexPedState *>(this->_vertices[0]);
                const PoseV *C = static_cast<const PoseV *>(this->_vertices[1]);
                Eigen::Matrix3d Rcw;
                Eigen::Vector3d tcw;
                PoseTraits<PoseV>::Get(C, Rcw, tcw);
                if (Rcw_out)
                    *Rcw_out = Rcw;
                return Rcw * S->p() + tcw;
            }
            double Bias() const;

            void computeError() override
            {
                this->_error(0) = std::log(Xc()(2)) - (_measurement + Bias());
            }

            void linearizeOplus() override
            {
                Eigen::Matrix3d Rcw;
                const Eigen::Vector3d X = Xc(&Rcw);
                Eigen::Matrix<double, 1, 3> dlog; // d log z / d Xc
                dlog << 0.0, 0.0, 1.0 / X(2);
                this->_jacobianOplus[0].setZero();
                this->_jacobianOplus[0].template leftCols<3>() = dlog * Rcw;
                this->_jacobianOplus[1] = dlog * PoseTraits<PoseV>::dXc(static_cast<const PoseV *>(this->_vertices[1]), X);
                if (biased)
                    this->_jacobianOplus[2](0, 0) = -1.0;
            }
            bool biased;
        };

        template <class PoseV>
        double EdgePedLogRange<PoseV>::Bias() const
        {
            return biased ? static_cast<const VertexPedBias *>(this->_vertices[2])->estimate() : 0.0;
        }

        // ---------------------------------------------------------------- white-noise-acceleration prior between two states of one track
        // e = [p_j - p_i - v_i dt ; v_j - v_i];  Q = q kron([[dt^3/3, dt^2/2],[dt^2/2, dt]], I3)  (P7-A addendum_A8.md:5-6, without the
        // P3 deviation term: the spec lists q_v as the only noise of this factor). Information Q^-1 is set by SetDt().
        class EdgePedWna : public g2o::BaseBinaryEdge<6, Vec6, VertexPedState, VertexPedState>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgePedWna() { dt = 0; }
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            // Q^-1 = (1/q) [[12/dt^3, -6/dt^2],[-6/dt^2, 4/dt]] kron I3   (closed form of the 2x2 inverse; det = q^2 dt^4/12)
            void SetDt(double dt_, double q) { SetDt(dt_, Eigen::Vector3d(q, q, q)); }

            // P9-C4: per-axis q = (q_x, q_y, q_z) in the world frame: Q = diag(q) kron [[dt^3/3, dt^2/2],[dt^2/2, dt]]; each axis a carries its own 2x2 inverse
            // (1/q_a) [[12/dt^3, -6/dt^2],[-6/dt^2, 4/dt]] between (p_a, v_a). With q_x = q_y = q_z this is the isotropic information bit for bit.
            void SetDt(double dt_, const Eigen::Vector3d &q)
            {
                dt = dt_;
                Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Zero();
                for (int a = 0; a < 3; ++a)
                {
                    I(a, a) = 12.0 / (q(a) * dt * dt * dt);
                    I(a, 3 + a) = I(3 + a, a) = -6.0 / (q(a) * dt * dt);
                    I(3 + a, 3 + a) = 4.0 / (q(a) * dt);
                }
                this->setInformation(I);
            }

            void computeError() override
            {
                const Vec6 &xi = static_cast<const VertexPedState *>(this->_vertices[0])->estimate();
                const Vec6 &xj = static_cast<const VertexPedState *>(this->_vertices[1])->estimate();
                this->_error.head<3>() = xj.head<3>() - xi.head<3>() - dt * xi.tail<3>();
                this->_error.tail<3>() = xj.tail<3>() - xi.tail<3>();
            }

            void linearizeOplus() override
            {
                Eigen::Matrix<double, 6, 6> Ji = Eigen::Matrix<double, 6, 6>::Zero();
                Ji.topLeftCorner<3, 3>() = -Eigen::Matrix3d::Identity();
                Ji.topRightCorner<3, 3>() = -dt * Eigen::Matrix3d::Identity();
                Ji.bottomRightCorner<3, 3>() = -Eigen::Matrix3d::Identity();
                this->_jacobianOplusXi = Ji;
                this->_jacobianOplusXj.setIdentity();
            }

            double dt;
        };

        // ---------------------------------------------------------------- root-over-footprint ties
        // Vertices: (0) VertexPedState, (1) footprint g2o::VertexSBAPointXYZ; the stature is the constant H0. UP = world z (gravity-aligned after IMU init).
        //   T1: e = UP.(t - X_f) - cRoot H0                              sigma 0.02   (criterion_G.md:80)
        //   T2: e = max(0, |(I-UP UP^T)(t - X_f)| - cLeg H0)  (hinge)   sigma 0.10   (criterion_G.md:81)
        class EdgePedTie1 : public g2o::BaseMultiEdge<1, double>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgePedTie1(double cRoot_, double H0_) : cRoot(cRoot_), H0(H0_) { this->resize(2); }
            Eigen::MatrixXd Jac(int i) const { return this->_jacobianOplus[i]; } // read access for the unit test
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }
            void computeError() override
            {
                const Eigen::Vector3d t = static_cast<const VertexPedState *>(this->_vertices[0])->p();
                const Eigen::Vector3d X = static_cast<const g2o::VertexSBAPointXYZ *>(this->_vertices[1])->estimate();
                this->_error(0) = (t(2) - X(2)) - cRoot * H0;
            }
            void linearizeOplus() override
            {
                this->_jacobianOplus[0].setZero();
                this->_jacobianOplus[0](0, 2) = 1.0;
                this->_jacobianOplus[1].setZero();
                this->_jacobianOplus[1](0, 2) = -1.0;
            }
            double cRoot, H0;
        };

        class EdgePedTie2 : public g2o::BaseMultiEdge<1, double>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgePedTie2(double cLeg_, double H0_) : cLeg(cLeg_), H0(H0_) { this->resize(2); }
            Eigen::MatrixXd Jac(int i) const { return this->_jacobianOplus[i]; } // read access for the unit test
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            double Excess(Eigen::Vector2d &u) const
            {
                const Eigen::Vector3d t = static_cast<const VertexPedState *>(this->_vertices[0])->p();
                const Eigen::Vector3d X = static_cast<const g2o::VertexSBAPointXYZ *>(this->_vertices[1])->estimate();
                const Eigen::Vector2d h = (t - X).head<2>();
                const double n = h.norm();
                u = n > 0 ? Eigen::Vector2d(h / n) : Eigen::Vector2d(Eigen::Vector2d::Zero());
                return n - cLeg * H0;
            }
            void computeError() override
            {
                Eigen::Vector2d u;
                const double ex = Excess(u);
                this->_error(0) = ex > 0 ? ex : 0.0;
            }
            void linearizeOplus() override
            {
                Eigen::Vector2d u;
                const double ex = Excess(u);
                this->_jacobianOplus[0].setZero();
                this->_jacobianOplus[1].setZero();
                if (ex > 0)
                {
                    this->_jacobianOplus[0].template block<1, 2>(0, 0) = u.transpose();
                    this->_jacobianOplus[1].template block<1, 2>(0, 0) = -u.transpose();
                }
            }
            double cLeg, H0;
        };

        // ---------------------------------------------------------------- floor contact (P9-C3)
        // Lowest foot keypoint (heel or toe) of a person = a ray b = ((u-cx)/fx, (v-cy)/fy, 1) in the camera frame. A walking person always has a foot on
        // the floor, so the ray meets the gravity-aligned floor plane z = zp (world frame, z up) at a point G that is stature-independent:
        //   n = Rcw e_z (world up in camera coordinates), den = n.b (< 0 for a downward ray), s = (zp + n.tcw)/den, Xc_G = s b, G = Rcw^T (Xc_G - tcw)
        // The edge penalises the horizontal offset between G and the root state:  e = (G - p)_xy  (metres, 2-D). This is the contact-vertex formulation
        // (pixel reprojection of a contact point + contact point on the plane + horizontal tie to the root) with the contact vertex eliminated: the
        // pixel and plane constraints are exact, and every noise (keypoint pixel, swing, heel/toe offset) sits in one metric residual that the
        // tuning data calibrate directly (c3_check0c.py).
        //   zp = (floor height) + delta, the floor height being: k == 0 (Variant A, regional) a constant z_f estimated from the static map points at build time and held fixed;
        //       k >= 1 (Variant B, joint) the mean height of k static map points that are live vertices of the same BA (the gravity-aligned plane through
        //       them, least-squares height); the points and the camera pose move.
        // Vertices: (0) VertexPedState, (1) camera pose, (2..k+1) g2o::VertexSBAPointXYZ.  `poseGradient` = false (Variant A, "pedestrian only"): the
        // residual follows the current camera pose but the pose Jacobian block is zero, so the factor cannot move the camera.
        // Jacobian derivation (both pose types): with M = dXc/d(pose update) of a fixed world point (PoseTraits::dXc),
        //   dG/dxi = Rcw^T ( b n^T M / den - M ),   dG/dzp = Rcw^T b / den.
        template <class PoseV>
        class EdgePedFloorContact : public g2o::BaseMultiEdge<2, Eigen::Vector2d>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            // k = 0: fixed plane height zf. k >= 1: the plane height is the mean z of the k point vertices (set via setVertex(2 + i, ...)).
            EdgePedFloorContact(double fx, double fy, double cx, double cy, double u, double v, int k, double zf, double deltaM, bool poseGradient)
                : b((u - cx) / fx, (v - cy) / fy, 1.0), nPts(k), zFixed(zf), delta(deltaM), grad(poseGradient)
            {
                this->resize(2 + k);
                this->_measurement << u, v;
            }
            Eigen::MatrixXd Jac(int i) const { return this->_jacobianOplus[i]; } // read access for the unit test
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            // Height of the plane the ray is intersected with: the floor (z_f, or the mean of the k points) plus `delta`, the calibrated height of the
            // lowest foot keypoint above the sole (heel / toe keypoint offset + swing; median of c3_check0c e_v on the tuning sequences).
            double PlaneHeight() const
            {
                return FloorHeight() + delta;
            }
            double FloorHeight() const
            {
                if (nPts == 0)
                    return zFixed;
                double z = 0;
                for (int i = 0; i < nPts; ++i)
                    z += static_cast<const g2o::VertexSBAPointXYZ *>(this->_vertices[2 + i])->estimate()(2);
                return z / nPts;
            }

            // Ground point, the ray parameter s (depth along the unit-z camera ray), and den; throws on a ray parallel to the floor.
            struct Geo
            {
                Eigen::Matrix3d Rcw;
                Eigen::Vector3d tcw, n, XcG, G;
                double s, den, zp;
            };
            Geo Geometry() const
            {
                Geo g;
                const PoseV *C = static_cast<const PoseV *>(this->_vertices[1]);
                PoseTraits<PoseV>::Get(C, g.Rcw, g.tcw);
                g.n = g.Rcw.col(2);
                g.den = g.n.dot(b);
                if (std::abs(g.den) < 1e-9)
                    throw std::runtime_error("[Pedestrians] floor contact: the foot ray is parallel to the floor plane");
                g.zp = PlaneHeight();
                g.s = (g.zp + g.n.dot(g.tcw)) / g.den;
                g.XcG = g.s * b;
                g.G = g.Rcw.transpose() * (g.XcG - g.tcw);
                return g;
            }

            void computeError() override
            {
                const Geo g = Geometry();
                const Eigen::Vector3d p = static_cast<const VertexPedState *>(this->_vertices[0])->p();
                this->_error = (g.G - p).template head<2>();
            }

            void linearizeOplus() override
            {
                const Geo g = Geometry();
                this->_jacobianOplus[0].setZero();
                this->_jacobianOplus[0](0, 0) = -1.0;
                this->_jacobianOplus[0](1, 1) = -1.0;
                if (grad)
                {
                    const Eigen::Matrix<double, 3, 6> M = PoseTraits<PoseV>::dXc(static_cast<const PoseV *>(this->_vertices[1]), g.XcG);
                    const Eigen::Matrix<double, 3, 6> dG = g.Rcw.transpose() * (b * (g.n.transpose() * M) / g.den - M);
                    this->_jacobianOplus[1] = dG.topRows<2>();
                }
                else
                    this->_jacobianOplus[1].setZero();
                if (nPts > 0)
                {
                    const Eigen::Vector3d dGdz = g.Rcw.transpose() * b / g.den;
                    for (int i = 0; i < nPts; ++i)
                    {
                        this->_jacobianOplus[2 + i].setZero();
                        this->_jacobianOplus[2 + i](0, 2) = dGdz(0) / nPts;
                        this->_jacobianOplus[2 + i](1, 2) = dGdz(1) / nPts;
                    }
                }
            }

            Eigen::Vector3d b;
            int nPts;
            double zFixed, delta;
            bool grad;
        };


        // ---------------------------------------------------------------- P9-C3c midpoint floor contact, image-space residual
        // Vertices: (0) VertexPedState, (1) camera pose, (2 ..) k static-point vertices (B': the floor height is the mean z of the k points; k = 0 (A'): the fixed z_f).
        //   F = (p_x, p_y, z_f + delta_v),   e = (u, v)_contact - pi(Rcw F + tcw)      (the same sign convention as EdgePedReproj)
        // The contact pixel is the midpoint of the two feet's lowest keypoints; delta_v is the calibrated height of that point above the floor (0.053 m).
        // Only p_x, p_y of the state enter: the root height is not constrained by this factor. poseGradient = false (A'): the pose block is exactly zero
        // (the factor is pedestrian-only; the residual still follows the current pose).
        template <class PoseV>
        class EdgePedFloorPx : public g2o::BaseMultiEdge<2, Eigen::Vector2d>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgePedFloorPx(double fx_, double fy_, double cx_, double cy_, double u, double v, int k, double zf, double deltaM, bool poseGradient)
                : fx(fx_), fy(fy_), cx(cx_), cy(cy_), nPts(k), zFixed(zf), delta(deltaM), grad(poseGradient)
            {
                this->resize(2 + k);
                this->_measurement << u, v;
            }
            Eigen::MatrixXd Jac(int i) const { return this->_jacobianOplus[i]; } // read access for the unit test
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            double FloorHeight() const
            {
                if (nPts == 0)
                    return zFixed;
                double z = 0;
                for (int i = 0; i < nPts; ++i)
                    z += static_cast<const g2o::VertexSBAPointXYZ *>(this->_vertices[2 + i])->estimate()(2);
                return z / nPts;
            }
            Eigen::Vector3d Xc(Eigen::Matrix3d *Rcw_out = nullptr) const
            {
                const PoseV *C = static_cast<const PoseV *>(this->_vertices[1]);
                Eigen::Matrix3d Rcw;
                Eigen::Vector3d tcw;
                PoseTraits<PoseV>::Get(C, Rcw, tcw);
                if (Rcw_out)
                    *Rcw_out = Rcw;
                const Eigen::Vector3d p = static_cast<const VertexPedState *>(this->_vertices[0])->p();
                return Rcw * Eigen::Vector3d(p(0), p(1), FloorHeight() + delta) + tcw;
            }
            void computeError() override
            {
                const Eigen::Vector3d X = Xc();
                const double z = X(2) > 1e-6 ? X(2) : 1e-6;
                this->_error << this->_measurement(0) - (fx * X(0) / z + cx), this->_measurement(1) - (fy * X(1) / z + cy);
            }
            void linearizeOplus() override
            {
                Eigen::Matrix3d Rcw;
                const Eigen::Vector3d X = Xc(&Rcw);
                const double z = X(2) > 1e-6 ? X(2) : 1e-6;
                Eigen::Matrix<double, 2, 3> dpi;
                dpi << fx / z, 0.0, -fx * X(0) / (z * z), 0.0, fy / z, -fy * X(1) / (z * z);
                this->_jacobianOplus[0].setZero();
                this->_jacobianOplus[0].template leftCols<2>() = -dpi * Rcw.template leftCols<2>();
                if (grad)
                    this->_jacobianOplus[1] = -dpi * PoseTraits<PoseV>::dXc(static_cast<const PoseV *>(this->_vertices[1]), X);
                else
                    this->_jacobianOplus[1].setZero();
                for (int i = 0; i < nPts; ++i)
                {
                    this->_jacobianOplus[2 + i].setZero();
                    this->_jacobianOplus[2 + i].col(2) = -dpi * Rcw.col(2) / nPts;
                }
            }
            double fx, fy, cx, cy;
            int nPts;
            double zFixed, delta;
            bool grad;
        };


        // ---------------------------------------------------------------- P9-C5 floor levels as landmark states
        // A level is a horizontal plane z = z_l in the gravity-aligned frame of one map (the S-Graphs floor layer). Additive update.
        class VertexFloorLevel : public g2o::BaseVertex<1, double>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            VertexFloorLevel() {}
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }
            void setToOriginImpl() override { _estimate = 0.0; }
            void oplusImpl(const double *u) override { _estimate += u[0]; }
        };

        // F1: contact factor. Vertices: (0) VertexPedState, (1) camera pose, (2) VertexFloorLevel.
        //   F = (p_x, p_y, z_l + delta_v),  e = (u, v)_contact - pi(Rcw F + tcw)   (sign convention of EdgePedReproj / EdgePedFloorPx)
        // Jacobians: dF/dp_xy = I, dF/dz_l = e_z, so  de/dp_xy = -dpi Rcw[:, 0:2],  de/dz_l = -dpi Rcw[:, 2],  de/dxi = -dpi M(Xc) (PoseTraits::dXc).
        template <class PoseV>
        class EdgePedLevelPx : public g2o::BaseMultiEdge<2, Eigen::Vector2d>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgePedLevelPx(double fx_, double fy_, double cx_, double cy_, double u, double v, double deltaM)
                : fx(fx_), fy(fy_), cx(cx_), cy(cy_), delta(deltaM)
            {
                this->resize(3);
                this->_measurement << u, v;
            }
            Eigen::MatrixXd Jac(int i) const { return this->_jacobianOplus[i]; } // read access for the unit test
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            Eigen::Vector3d Xc(Eigen::Matrix3d *Rcw_out = nullptr) const
            {
                Eigen::Matrix3d Rcw;
                Eigen::Vector3d tcw;
                PoseTraits<PoseV>::Get(static_cast<const PoseV *>(this->_vertices[1]), Rcw, tcw);
                if (Rcw_out)
                    *Rcw_out = Rcw;
                const Eigen::Vector3d p = static_cast<const VertexPedState *>(this->_vertices[0])->p();
                const double zl = static_cast<const VertexFloorLevel *>(this->_vertices[2])->estimate();
                return Rcw * Eigen::Vector3d(p(0), p(1), zl + delta) + tcw;
            }
            void computeError() override
            {
                const Eigen::Vector3d X = Xc();
                const double z = X(2) > 1e-6 ? X(2) : 1e-6;
                this->_error << this->_measurement(0) - (fx * X(0) / z + cx), this->_measurement(1) - (fy * X(1) / z + cy);
            }
            void linearizeOplus() override
            {
                Eigen::Matrix3d Rcw;
                const Eigen::Vector3d X = Xc(&Rcw);
                const double z = X(2) > 1e-6 ? X(2) : 1e-6;
                Eigen::Matrix<double, 2, 3> dpi;
                dpi << fx / z, 0.0, -fx * X(0) / (z * z), 0.0, fy / z, -fy * X(1) / (z * z);
                this->_jacobianOplus[0].setZero();
                this->_jacobianOplus[0].template leftCols<2>() = -dpi * Rcw.template leftCols<2>();
                this->_jacobianOplus[1] = -dpi * PoseTraits<PoseV>::dXc(static_cast<const PoseV *>(this->_vertices[1]), X);
                this->_jacobianOplus[2] = -dpi * Rcw.col(2);
            }
            double fx, fy, cx, cy, delta;
        };

        // F2: static map point on a level. Vertices: (0) g2o::VertexSBAPointXYZ (live or fixed), (1) VertexFloorLevel.  e = z_point - z_l.
        class EdgePedPointOnLevel : public g2o::BaseBinaryEdge<1, double, g2o::VertexSBAPointXYZ, VertexFloorLevel>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgePedPointOnLevel() {}
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }
            void computeError() override
            {
                this->_error(0) = static_cast<const g2o::VertexSBAPointXYZ *>(this->_vertices[0])->estimate()(2) -
                                  static_cast<const VertexFloorLevel *>(this->_vertices[1])->estimate();
            }
            void linearizeOplus() override
            {
                this->_jacobianOplusXi.setZero();
                this->_jacobianOplusXi(0, 2) = 1.0;
                this->_jacobianOplusXj(0, 0) = -1.0;
            }
        };

        // F3: platform-height prior. Vertices: (0) camera pose, (1) VertexFloorLevel of the level under the camera.  e = z_cam - z_l - h_cam, z_cam the
        // camera-centre height C_z, C = -Rcw^T tcw. A fixed world point at C has Xc = 0, and dXc = -Rcw dC for any pose update (the rotation terms
        // multiply Xc = 0), so dC/dxi = -Rcw^T M(0), M = PoseTraits::dXc(., 0); the Jacobian row is the third row of that.
        template <class PoseV>
        class EdgePedPlatformHeight : public g2o::BaseBinaryEdge<1, double, PoseV, VertexFloorLevel>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgePedPlatformHeight() {}
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }
            // _measurement = h_cam
            static double CamZ(const PoseV *v)
            {
                Eigen::Matrix3d Rcw;
                Eigen::Vector3d tcw;
                PoseTraits<PoseV>::Get(v, Rcw, tcw);
                return (-Rcw.transpose() * tcw)(2);
            }
            void computeError() override
            {
                this->_error(0) = CamZ(static_cast<const PoseV *>(this->_vertices[0])) -
                                  static_cast<const VertexFloorLevel *>(this->_vertices[1])->estimate() - this->_measurement;
            }
            void linearizeOplus() override
            {
                Eigen::Matrix3d Rcw;
                Eigen::Vector3d tcw;
                const PoseV *C = static_cast<const PoseV *>(this->_vertices[0]);
                PoseTraits<PoseV>::Get(C, Rcw, tcw);
                const Eigen::Matrix<double, 3, 6> M0 = PoseTraits<PoseV>::dXc(C, Eigen::Vector3d::Zero());
                this->_jacobianOplusXi = (-Rcw.transpose() * M0).row(2);
                this->_jacobianOplusXj(0, 0) = -1.0;
            }
        };

    } // namespace ped
} // namespace ORB_SLAM3

#endif
