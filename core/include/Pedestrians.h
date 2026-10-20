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
        std::array<bool, 2> claim = {false, false};      // stance claim per foot (L, R), gated by the producer on scores
        std::array<double, 2> toeU = {0, 0}, toeV = {0, 0}; // toe-midpoint pixel of the claimed foot
        double boxH = 0;            // person box height [px] (visibility gate for stance, P4-S1)
    };

    struct PedFrame
    {
        double stamp = 0;
        std::vector<PedObs> obs;    // sorted by trackId (the loader enforces it)
    };

    // Settings (all required when Pedestrians.Enabled; no defaults). Sources are listed in eval/phase9/P9-C/factors.md.
    struct PedestrianParams
    {
        bool enabled = false;
        double rootPxSigma = 0;      // root keypoint reprojection [px]
        double logDepthSigma = 0;    // nose->heel log-depth factor
        double statureMean = 0, statureSigma = 0; // [m]
        double skelOverStature = 0;  // L_skel / stature: nose->heel-mid length as a fraction of stature (P1-B_report.md:96-100)
        double qv = 0;               // white-noise-acceleration PSD [m^2/s^3]
        double footPxSigma = 0;      // stance footprint reprojection [px]
        double t1Sigma = 0, t2Sigma = 0; // root-over-footprint ties [m]
        double cRoot = 0, cLeg = 0;  // root height / leg length as fractions of stature
        int minTrackKeyframes = 0;   // measurements of a track seen in fewer graph keyframes are skipped
        double minStanceBoxPx = 0;   // stance only when the person box is at least this tall (P4-S1 visibility gate)
        int minFootObservations = 0; // a footprint vertex needs this many observations
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
