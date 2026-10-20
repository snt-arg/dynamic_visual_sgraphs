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
        double huberScale = 0;       // multiplies every Huber threshold of the pedestrian measurement factors (1 = the chi2-95 % values of Optimizer.cc:1994-1995)
        std::string dir;             // per-frame input directory (read by the runner, not by the library)

        static PedestrianParams &Get();
        static bool Enabled() { return Get().enabled; }
        // Reads the Pedestrians.* keys; throws std::runtime_error naming the first missing key. Returns enabled.
        static bool Configure(cv::FileStorage &fs);
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
