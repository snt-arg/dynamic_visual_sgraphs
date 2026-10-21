// P9-C4: prediction-based innovation gate for new pedestrian keyframe measurements (Pedestrians.InnovGate: 1; metric maps only).
//
// The production logic, shared by PedestrianBA.cc and the synthetic checks (core/test/synthetic_gate.cc).
//   prediction  p^ = p + v dt from the track's last written-back state (the preceding accepted keyframe), dt = time to the new keyframe
//   covariance  Sigma = F P0 F^T + Q(dt), F = [[I, dt I],[0, I]], P0 = diag(sP_xy^2, sP_xy^2, sP_z^2, sV_xy^2, sV_xy^2, sV_z^2) (constant, calibrated on
//               tuning data), Q(dt) = diag(q) kron [[dt^3/3, dt^2/2],[dt^2/2, dt]] of the ACTIVE motion prior (isotropic q_v, or (q_h, q_h, q_z))
//   innovation  y = z - h(p^), h = (u, v[, log z]) of the keyframe camera, covariance S = J Sigma_pp J^T + R, R = diag(rootPxSigma^2, rootPxSigma^2,
//               logDepthSigma^2) (the frozen measurement noise); the log-depth row only when the observation carries a range measurement
//   test        reject if y^T S^-1 y > chi2(0.99, dof) (dof 3 with the range row, 2 without)
//   chain       the first `innovFreeKeyframes` accepted measurements of a chain are not gated; `innovReinitRejections` consecutive rejections
//               re-initialise the track (the measurement that completes the run starts a new chain)
#ifndef PEDESTRIAN_GATE_H
#define PEDESTRIAN_GATE_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "Pedestrians.h"

namespace ORB_SLAM3
{
    namespace ped
    {
        // active per-axis motion-prior PSD
        inline Eigen::Vector3d ActiveQ(const PedestrianParams &P)
        {
            return P.motionPrior == PedestrianParams::MotionPrior::Anisotropic ? Eigen::Vector3d(P.qvH, P.qvH, P.qvZ) : Eigen::Vector3d(P.qv, P.qv, P.qv);
        }

        struct GateTest
        {
            bool ok = false;     // false: not testable (predicted point at or behind the image plane)
            int dof = 0;
            double d2 = 0;
            bool reject = false;
        };

        // p, v: world-frame state of the predecessor; dt: time to the new keyframe; Rcw, tcw: pose of the new keyframe; (u, v): measured root pixel;
        // hasRange: the observation carries a range measurement, logZ = log(f cL H / l_px)
        inline GateTest InnovationTest(const PedestrianParams &P, const Eigen::Vector3d &p, const Eigen::Vector3d &v, double dt, const Eigen::Matrix3d &Rcw,
                                       const Eigen::Vector3d &tcw, double fx, double fy, double cx, double cy, double u, double vpx, bool hasRange, double logZ)
        {
            GateTest g;
            const Eigen::Vector3d ph = p + v * dt;
            const Eigen::Vector3d Xc = Rcw * ph + tcw;
            if (!(Xc(2) > 0.1))
                return g;
            Eigen::Matrix<double, 6, 6> P0 = Eigen::Matrix<double, 6, 6>::Zero();
            P0.diagonal() << P.innovP0PosXY * P.innovP0PosXY, P.innovP0PosXY * P.innovP0PosXY, P.innovP0PosZ * P.innovP0PosZ, P.innovP0VelXY * P.innovP0VelXY,
                P.innovP0VelXY * P.innovP0VelXY, P.innovP0VelZ * P.innovP0VelZ;
            Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
            F.topRightCorner<3, 3>() = dt * Eigen::Matrix3d::Identity();
            Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
            const Eigen::Vector3d q = ActiveQ(P);
            for (int a = 0; a < 3; ++a)
            {
                Q(a, a) = q(a) * dt * dt * dt / 3.0;
                Q(a, 3 + a) = Q(3 + a, a) = q(a) * dt * dt / 2.0;
                Q(3 + a, 3 + a) = q(a) * dt;
            }
            const Eigen::Matrix<double, 6, 6> Sig = F * P0 * F.transpose() + Q;
            const Eigen::Matrix3d Spp = Sig.topLeftCorner<3, 3>();
            const int m = hasRange ? 3 : 2;
            Eigen::Matrix<double, 3, 3> Jc = Eigen::Matrix3d::Zero(); // d h / d Xc
            Jc(0, 0) = fx / Xc(2);
            Jc(0, 2) = -fx * Xc(0) / (Xc(2) * Xc(2));
            Jc(1, 1) = fy / Xc(2);
            Jc(1, 2) = -fy * Xc(1) / (Xc(2) * Xc(2));
            Jc(2, 2) = 1.0 / Xc(2);
            const Eigen::Matrix3d J = Jc * Rcw;
            Eigen::Vector3d y;
            y << u - (fx * Xc(0) / Xc(2) + cx), vpx - (fy * Xc(1) / Xc(2) + cy), logZ - std::log(Xc(2));
            Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
            R.diagonal() << P.rootPxSigma * P.rootPxSigma, P.rootPxSigma * P.rootPxSigma, P.logDepthSigma * P.logDepthSigma;
            const Eigen::Matrix3d S = J * Spp * J.transpose() + R;
            const Eigen::MatrixXd Sm = S.topLeftCorner(m, m);
            const Eigen::VectorXd ym = y.head(m);
            g.ok = true;
            g.dof = m;
            g.d2 = ym.dot(Sm.ldlt().solve(ym));
            g.reject = g.d2 > (hasRange ? P.innovChi2Dof3 : P.innovChi2Dof2);
            return g;
        }

        // chain bookkeeping of one track; one call per NEW measurement in time order. `test` = the Mahalanobis test, valid only if `tested`.
        struct GateChain
        {
            int idx = 0;       // accepted measurements in the current chain
            int consecRej = 0; // consecutive rejections
        };

        inline PedGateRec GateStep(const PedestrianParams &P, GateChain &ch, bool havePred, const GateTest &t, double dt, double predStamp)
        {
            PedGateRec r;
            r.dt = dt;
            r.predStamp = predStamp;
            const bool free = ch.idx < P.innovFreeKeyframes || !havePred || !t.ok;
            if (free)
            {
                r.status = PedGateRec::ACCEPTED;
                ++ch.idx;
                ch.consecRej = 0;
            }
            else
            {
                r.tested = true;
                r.dof = t.dof;
                r.d2 = t.d2;
                if (!t.reject)
                {
                    r.status = PedGateRec::ACCEPTED;
                    ++ch.idx;
                    ch.consecRej = 0;
                }
                else if (++ch.consecRej >= P.innovReinitRejections)
                {
                    r.status = PedGateRec::REINIT;
                    ch.idx = 1;
                    ch.consecRej = 0;
                }
                else
                    r.status = PedGateRec::REJECTED;
            }
            r.chainIdx = ch.idx;
            r.consecRej = ch.consecRej;
            return r;
        }
    } // namespace ped
} // namespace ORB_SLAM3

#endif
