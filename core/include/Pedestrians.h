// P9-C: pedestrian measurements and states for ORB-SLAM3's local BA.
//
// Everything here is inert unless `Pedestrians.Enabled: 1` is in the settings file (PedestrianParams::Enabled()).
// With the flag off no pedestrian input is read, no vertex or edge is created (core/src/PedestrianBA.cc is never entered).
#ifndef PEDESTRIANS_H
#define PEDESTRIANS_H

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

namespace ORB_SLAM3
{

    // One pedestrian measured in one image (file schema: eval/phase9/P9-C/perception/PERCEPTION_SCHEMA.md).
    struct PedObs
    {
        int trackId = -1;           // tracker id (dynamic_keypoint_tracker)
        double rootU = 0, rootV = 0; // mask root (cu, vb - 0.55 h) [px]
        double lpx = 0;             // |nose - mean(heels)| in the image [px] (P1-B l_px); <= 0 -> no range measurement
        double maskH = 0;           // mask pixel height [px]; only used to initialise a state's depth when lpx <= 0
        // P9-C3: lowest foot keypoint (heel or toe, RTMW, score >= 0.3) of this person in this image [px]; `F` row of the per-frame file
        bool hasFoot = false;
        double footU = 0, footV = 0, footScore = 0;
    };

    // Stance claim of one track at one image, delivered LATE: the P4-S1 agree+timing claim at image t is final only 6 images later
    // (stance_foot_poc stance/fusion.py:72 TIMING_WINDOW_FRAMES = 6, :92 window, :110 near_in_time looks 6 images ahead).
    struct PedClaim
    {
        int trackId = -1;
        std::array<bool, 2> claim = {false, false};         // left, right foot
        std::array<double, 2> toeU = {0, 0}, toeV = {0, 0}; // toe-midpoint pixel of that foot at the claimed image
        double boxH = 0;                                     // person box height [px] (visibility gate for stance, P4-S1)
    };

    struct PedLateClaims
    {
        double stamp = 0;               // the CLAIMED image (the claims were finalised 6 images after it)
        std::vector<PedClaim> claims;
    };

    struct PedFrame
    {
        double stamp = 0;
        std::vector<PedObs> obs;    // sorted by trackId (the loader enforces it)
        std::vector<PedLateClaims> lateClaims; // claims finalised at this image, for earlier images (System::SetPedestrianFrame applies them)
    };

    // Settings (all required when Pedestrians.Enabled; no defaults). Sources are listed in eval/phase9/P9-C/factors.md.
    struct PedestrianParams
    {
        bool enabled = false;
        double rootPxSigma = 0;      // root keypoint reprojection [px]
        double logDepthSigma = 0;    // nose->heel log-depth factor
        double statureFixedM = 0;    // fixed stature H0 [m] (1.70: the convention of cL = L_skel / 1.70); not a state (P9-C2 5d)
        double skelOverStature = 0;  // L_skel / 1.70: nose->heel-mid length as a fraction of the fixed stature (P1-B_report.md:96-100)
        double qv = 0;               // white-noise-acceleration PSD [m^2/s^3]
        double footPxSigma = 0;      // stance footprint reprojection [px]
        double t1Sigma = 0, t2Sigma = 0; // root-over-footprint ties [m]
        double cRoot = 0, cLeg = 0;  // root height / leg length as fractions of stature
        int minTrackKeyframes = 0;   // measurements of a track seen in fewer graph keyframes are skipped
        double minStanceBoxPx = 0;   // stance only when the person box is at least this tall (P4-S1 visibility gate)
        int minFootObservations = 0; // a footprint vertex needs this many observations
        double minLpx = 0;           // observations with a nose->heel length below this [px] are not used (0: any valid length); tuning gate
        // range-consistency gate (P9-C2 5b.2): the nose->heel range and the mask-height range are independent cues; an observation whose log
        // difference d = log r_lpx - log r_mask is further than gateK * gateSigma from gateCenter loses its range factor (root reprojection stays).
        double maskHeightM = 0, gateCenter = 0, gateSigma = 0, gateK = 0; // gateK <= 0: gate off
        // P9-C3 floor contact (Pedestrians.FloorContact: none | regional | joint). All keys below are read only when the mode is not `none`.
        // P9-C3c: `shadow` = the A'/B' geometry and gates are evaluated and logged (ped_floor_px.log), no vertex / edge is added (calibration of the gates);
        // `regional_px` = A' + b, `joint_px` = B' + b: midpoint-of-feet contact, image-space residual pi(p_x, p_y, z_f + delta_v) - contact pixel.
        // P9-C5: `levels` = floor levels as landmark states (F1 contact + F2 points on the level [+ F3 platform height]) with b; `levels_shadow` = the levels are estimated from the
        // contacts of the current states and logged, no vertex / edge is added (level check, calibration).
        enum class FloorMode { None, Regional, Joint, Shadow, RegionalPx, JointPx, Levels, LevelsShadow };
        FloorMode floorMode = FloorMode::None;
        double floorSigmaRadialM = 0, floorSigmaTangentialM = 0; // metric residual (G - root)_xy along / across the horizontal viewing direction (c3_check0c calibration)
        double floorRegionRadiusM = 0;  // Variant A: radius R of the region whose static map points give z_f
        double floorContactOffsetM = 0; // delta: height of the lowest foot keypoint above the floor (calibrated, c3_check0c); added to the floor plane height
        double floorLowModeFraction = 0;  // Variant A estimator: 1 = highest histogram mode; < 1 = lowest local maximum with at least this fraction of the highest count
        int floorMinPoints = 0;         // Variant A: fewer map points below the camera in the region -> no z_f, no factor
        double floorBinM = 0;           // Variant A: height-histogram bin
        double floorBelowCameraM = 0;   // Variant A: only points lower than the camera by at least this margin
        int floorNeighbours = 0;        // Variant B: k static points that define the local plane
        double floorNeighbourRadiusM = 0; // Variant B: horizontal search radius around the ray/z_f intersection
        double floorBandM = 0;          // Variant B: neighbours must lie within +- this of the regional z_f (selection only; z_f is not a constraint in B)
        // P9-C3c (modes shadow / regional_px / joint_px)
        double floorMidDeltaM = 0;       // delta_v: height of the midpoint contact above the floor [m] (0.053, Marco 21 Oct)
        double floorSigmaPxU = 0, floorSigmaPxV = 0; // contact pixel noise [px] (horizontal 6.0, vertical 3.0; 5b rule)
        double floorZfBiasM = 0;         // median error of the regional z_f against the GT floor, subtracted from z_f (calibrated on the C4 base runs)
        double floorRaisedMaxM = 0;      // a contact whose regional z_f is more than this above the map's global lowest floor mode is dropped (raised surface)
        double floorMinElevDeg = 0;      // a contact whose foot ray is less than this below the horizontal is dropped (calibrated)
        double floorPlaneSigmaMaxM = 0;  // B': the qualifying static points must have a height scatter below this
        double floorPlausM = 0;         // plausibility: contact height within +- this of the local floor estimate (log + check script)
        // P9-C5 (read only when FloorContact is levels | levels_shadow; calibrated on tuning data, constants_c5.md; the design constants are in PedestrianLevels.h)
        double levelSigmaA = 0, levelSigmaPt = 0;
        double levelTransRate = 0;   // P9-L: transitional flag threshold on the robust |dh/dt| of the contact height [m/s] (replaces LevelSMax)
        double footGateTau = 0;      // P9-L: foot-validity gate, a contact whose foot pixel is more than tau * maskH above the mask bottom is rejected; 0 = off
        bool platformHeight = false;                   // Pedestrians.PlatformHeight: off | on (F3)
        double platformHeightM = 0, platformSigmaM = 0; // h_cam, sigma_h (read only when on)
        // P9-C3b per-track log-scale bias b_j (read only when FloorContact != none): prior N(0, biasSigma^2); biasMinContacts > 0: b_j exists only for tracks with
        // at least that many foot observations on free states in the BA window (otherwise b = 0 for that call); 0: always.
        double biasSigma = 0;
        int biasMinContacts = 0;
        // P9-C4 motion prior (Pedestrians.MotionPrior: isotropic | anisotropic). isotropic: Q = qv (x) [[dt^3/3, dt^2/2],[dt^2/2, dt]] on all three axes (P9-C..C3);
        // anisotropic: Q = diag(qvH, qvH, qvZ) (x) [[...]] in the gravity-aligned world frame (metric maps only; the prior is only ever added there).
        enum class MotionPrior { Isotropic, Anisotropic };
        MotionPrior motionPrior = MotionPrior::Isotropic;
        double qvH = 0, qvZ = 0;     // read only when anisotropic
        // P9-C4 prediction-based innovation gate (Pedestrians.InnovGate: 0 | 1; metric maps only). All keys below are read only when the gate is on.
        bool innovGate = false;
        bool innovShadow = false;                      // Pedestrians.InnovGate: 2 = log-only: tests are computed and logged at decision time, every measurement is accepted
        double innovScale[4] = {1, 1, 1, 1};           // P9-C4 II: covariance scale s_k by chain age k of the tested measurement (k = 3, 4, 5, >= 6): reject iff d^2 > s_k chi2(0.99, dof)
        double innovChi2Dof2 = 0, innovChi2Dof3 = 0;   // rejection thresholds chi2(0.99, 2) and chi2(0.99, 3)
        int innovFreeKeyframes = 0;                    // the first this-many accepted measurements of a track are not gated
        int innovReinitRejections = 0;                 // this many consecutive rejections re-initialise the track
        double innovP0PosXY = 0, innovP0PosZ = 0, innovP0VelXY = 0, innovP0VelZ = 0; // state standard deviations of P0 = diag(...)^2 (world frame; m, m/s)
        double huberScale = 0;       // multiplies every Huber threshold of the pedestrian measurement factors (1 = the chi2-95 % values of Optimizer.cc:1994-1995)
        std::string dir;             // per-frame input directory (read by the runner, not by the library)

        static PedestrianParams &Get();
        static bool Enabled() { return Get().enabled; }
        // Reads the Pedestrians.* keys; throws std::runtime_error naming the first missing key. Returns enabled.
        static bool Configure(cv::FileStorage &fs);
    };

    // P9-C4 innovation gate: the decision taken once, when a (keyframe, track) measurement first reaches a metric BA call. Persisted on the keyframe so a
    // later window does not re-decide it (the track's last written-back state moves with every solve).
    struct PedGateRec
    {
        enum Status { ACCEPTED = 1, REJECTED = 2, REINIT = 3 };
        int status = ACCEPTED;
        int chainIdx = 0;     // accepted measurements in the track's current chain after this decision (a REINIT starts the chain at 1)
        int consecRej = 0;    // consecutive rejections after this decision
        bool tested = false;  // false: accepted without a test (free keyframes, or no written-back predecessor in the window)
        int dof = 0;
        double d2 = 0;        // Mahalanobis distance^2 of the innovation (tested only)
        double dt = 0;        // time to the predecessor state
        double predStamp = 0; // stamp of the keyframe whose written-back state was propagated
    };

    // Persistent per-keyframe state of one track (world frame; the map is metric once the IMU is initialised).
    struct PedState
    {
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
    };

    // Reads the per-frame files written by eval/phase9/P9-C/perception/p9c_assemble.py. Used by the runner only,
    // and only constructed when the flag is on.
    class PedestrianFrameReader
    {
    public:
        explicit PedestrianFrameReader(const std::string &dir); // <dir>/<stamp_ns:019d>.ped.txt
        // Returns the frame whose stamp equals `stamp` to 1 ms; throws if the file is missing (every image has one, possibly empty).
        std::shared_ptr<PedFrame> Read(double stamp) const;
        std::size_t FilesOpened() const { return mnOpened; }

    private:
        std::string mDir;
        mutable std::size_t mnOpened = 0;
    };

} // namespace ORB_SLAM3

#endif
