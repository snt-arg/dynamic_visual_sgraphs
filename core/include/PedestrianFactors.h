// P9-C: g2o vertices and edges for pedestrians in ORB-SLAM3's local BA. Header only; the unit test
// (core/test/test_pedestrian_factors.cc) checks every analytic Jacobian against central differences.
//
// Frames and conventions
//   * Xc = Rcw Xw + tcw. The camera pose vertex is either g2o::VertexSE3Expmap (LocalBundleAdjustment, update = exp([w;u]) * T)
//     or ORB_SLAM3::VertexPose (LocalInertialBA, camera index 0, update = ImuCamPose::Update). `PoseTraits<V>` gives
//     (Rcw, tcw) and dXc/d(pose update) for both.
//   * Person root state: VertexPedState = (p, v), world frame. Stature: VertexStature. Footprint: g2o::VertexSBAPointXYZ.
//   * Pinhole camera, no distortion (checked by PedestrianBA at graph build).
//
// Provenance of the math (reuse of object_motion_estimator, branch p9c-eval, include/object_motion_estimator/):
//   EdgePedRootReproj    <- EdgeMonoObject (object_reprojection_factor.hpp:43,57,90); same residual obs - pi(Xc), SE3deriv
//   EdgePedLogRange      <- EdgeObjectRangeBearing (object_range_bearing_factor.hpp:68-150), re-expressed as a log-range
//                           measurement on the nose->heel length (P7-A A3) and on a world-frame state instead of H_jk
//   VertexStature/Prior  <- VertexPersonHeight, EdgePersonHeightPrior (object_range_bearing_factor.hpp:37,49)
//   EdgePedWna           : new edge (white-noise-acceleration kernel, P7-A p7a_lib.py:213 dwna_q2 / addendum_A8.md:5-6)
//   EdgePedFootRoot T1,T2: new edges (P7-A p7a_lib.py:293,298; criterion_G.md:80-81)
#ifndef PEDESTRIAN_FACTORS_H
#define PEDESTRIAN_FACTORS_H

#include <cmath>
#include <istream>
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

        // Stature of one track [m]; persistent across BAs (Map::mPedStature).
        class VertexStature : public g2o::BaseVertex<1, double>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            VertexStature() {}
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }
            void setToOriginImpl() override { _estimate = 1.70; }
            void oplusImpl(const double *u) override { _estimate += u[0]; }
        };

        inline Eigen::Vector3d PosOf(const VertexPedState *v) { return v->p(); }
        inline Eigen::Vector3d PosOf(const g2o::VertexSBAPointXYZ *v) { return v->estimate(); }

        // ---------------------------------------------------------------- stature prior (unary)
        // e = H - mean; information 1/sigma^2 (set by the caller). Not robustified: it is a prior.
        class EdgeStaturePrior : public g2o::BaseUnaryEdge<1, double, VertexStature>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgeStaturePrior() {}
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }
            void computeError() override { _error(0) = static_cast<const VertexStature *>(_vertices[0])->estimate() - _measurement; }
            void linearizeOplus() override { _jacobianOplusXi(0, 0) = 1.0; }
        };

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

        // ---------------------------------------------------------------- nose->heel log-range
        // Vertices: (0) VertexPedState, (1) camera pose, (2) VertexStature.
        //   e = log|Xc| - (log f + log(cL) + log H - log l_px)       (P7-A A3: range along the ray from the nose->heel length,
        //   sigma = 0.031 in log range; cL = L_skel / stature, P1-B_report.md:96-100)
        // _measurement = log f + log cL - log l_px.
        template <class PoseV>
        class EdgePedLogRange : public g2o::BaseMultiEdge<1, double>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            EdgePedLogRange() { this->resize(3); }
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

            void computeError() override
            {
                const double H = static_cast<const VertexStature *>(this->_vertices[2])->estimate();
                this->_error(0) = std::log(Xc().norm()) - (_measurement + std::log(H));
            }

            void linearizeOplus() override
            {
                Eigen::Matrix3d Rcw;
                const Eigen::Vector3d X = Xc(&Rcw);
                const double n2 = X.squaredNorm();
                const Eigen::Matrix<double, 1, 3> dlog = X.transpose() / n2; // d log|Xc| / d Xc
                this->_jacobianOplus[0].setZero();
                this->_jacobianOplus[0].template leftCols<3>() = dlog * Rcw;
                this->_jacobianOplus[1] = dlog * PoseTraits<PoseV>::dXc(static_cast<const PoseV *>(this->_vertices[1]), X);
                this->_jacobianOplus[2](0, 0) = -1.0 / static_cast<const VertexStature *>(this->_vertices[2])->estimate();
            }
        };

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
            void SetDt(double dt_, double q)
            {
                dt = dt_;
                Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Zero();
                const double a = 12.0 / (q * dt * dt * dt), b = -6.0 / (q * dt * dt), c = 4.0 / (q * dt);
                I.topLeftCorner<3, 3>() = a * Eigen::Matrix3d::Identity();
                I.topRightCorner<3, 3>() = b * Eigen::Matrix3d::Identity();
                I.bottomLeftCorner<3, 3>() = b * Eigen::Matrix3d::Identity();
                I.bottomRightCorner<3, 3>() = c * Eigen::Matrix3d::Identity();
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
        // Vertices: (0) VertexPedState, (1) footprint g2o::VertexSBAPointXYZ, (2) VertexStature. UP = world z (gravity-aligned after IMU init).
        //   T1: e = UP.(t - X_f) - cRoot H                              sigma 0.02   (criterion_G.md:80)
        //   T2: e = max(0, |(I-UP UP^T)(t - X_f)| - cLeg H)  (hinge)   sigma 0.10   (criterion_G.md:81)
        class EdgePedTie1 : public g2o::BaseMultiEdge<1, double>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            explicit EdgePedTie1(double cRoot_) : cRoot(cRoot_) { this->resize(3); }
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }
            void computeError() override
            {
                const Eigen::Vector3d t = static_cast<const VertexPedState *>(this->_vertices[0])->p();
                const Eigen::Vector3d X = static_cast<const g2o::VertexSBAPointXYZ *>(this->_vertices[1])->estimate();
                const double H = static_cast<const VertexStature *>(this->_vertices[2])->estimate();
                this->_error(0) = (t(2) - X(2)) - cRoot * H;
            }
            void linearizeOplus() override
            {
                this->_jacobianOplus[0].setZero();
                this->_jacobianOplus[0](0, 2) = 1.0;
                this->_jacobianOplus[1].setZero();
                this->_jacobianOplus[1](0, 2) = -1.0;
                this->_jacobianOplus[2](0, 0) = -cRoot;
            }
            double cRoot;
        };

        class EdgePedTie2 : public g2o::BaseMultiEdge<1, double>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            explicit EdgePedTie2(double cLeg_) : cLeg(cLeg_) { this->resize(3); }
            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            double Excess(Eigen::Vector2d &u) const
            {
                const Eigen::Vector3d t = static_cast<const VertexPedState *>(this->_vertices[0])->p();
                const Eigen::Vector3d X = static_cast<const g2o::VertexSBAPointXYZ *>(this->_vertices[1])->estimate();
                const double H = static_cast<const VertexStature *>(this->_vertices[2])->estimate();
                const Eigen::Vector2d h = (t - X).head<2>();
                const double n = h.norm();
                u = n > 0 ? Eigen::Vector2d(h / n) : Eigen::Vector2d(Eigen::Vector2d::Zero());
                return n - cLeg * H;
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
                this->_jacobianOplus[2].setZero();
                if (ex > 0)
                {
                    this->_jacobianOplus[0].template block<1, 2>(0, 0) = u.transpose();
                    this->_jacobianOplus[1].template block<1, 2>(0, 0) = -u.transpose();
                    this->_jacobianOplus[2](0, 0) = -cLeg;
                }
            }
            double cLeg;
        };

    } // namespace ped
} // namespace ORB_SLAM3

#endif
