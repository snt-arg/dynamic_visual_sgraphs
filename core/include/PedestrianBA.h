// P9-C: the one helper that adds pedestrian vertices and edges to a local BA graph. Called from
// Optimizer::LocalBundleAdjustment (poses = g2o::VertexSE3Expmap) and Optimizer::LocalInertialBA (poses = VertexPose).
#ifndef PEDESTRIAN_BA_H
#define PEDESTRIAN_BA_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "KeyFrame.h"
#include "Map.h"
#include "PedestrianFactors.h"
#include "Pedestrians.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"

namespace ORB_SLAM3
{
    struct PedGraphKF
    {
        KeyFrame *kf;
        bool fixed; // pose vertex is fixed in the graph; its person states are then fixed too (as ORB-SLAM3 fixes keyframes)
    };

    struct PedGraph
    {
        struct StateHandle
        {
            int track;
            KeyFrame *kf;
            ped::VertexPedState *v;
            bool fixed;
        };
        std::vector<StateHandle> states;                       // deterministic order: track id, then keyframe time
        unsigned long levelEpoch = 0;                          // P9-L: epoch of the map's level set when Add read it (WriteBack skips the level update if it changed)
        enum Type { ROOT = 0, RANGE, WNA, FOOT, T1, T2, FLOOR, BIAS, LVPT, PLAT, NTYPES }; // LVPT = F2 point on level, PLAT = F3 platform height (P9-C5)
        struct FloorHandle // one floor-contact edge (P9-C3): written back with the state it belongs to
        {
            int track;
            KeyFrame *kf;
            g2o::OptimizableGraph::Edge *e;                            // ped::EdgePedFloorContact<VertexPose> if `imu`, else <g2o::VertexSE3Expmap>
            bool imu;
            double zRegional;                                          // the regional z_f at build time (Variant A: the plane height; B: selection reference)
        };
        std::vector<FloorHandle> floors;
        struct PxHandle // one midpoint floor-contact edge (P9-C3c): the residual at the solution is written to the px log
        {
            unsigned long kf;
            int track;
            g2o::OptimizableGraph::Edge *e;
        };
        std::vector<PxHandle> pxFloors;
        int nPxNoZf = 0, nPxRaised = 0, nPxElev = 0, nPxNoNeighbours = 0, nPxPlane = 0;
        struct BiasHandle
        {
            int track;
            ped::VertexPedBias *v;
        };
        struct LevelHandle // P9-C5: a level vertex of this call, written back to the map's LevelSet
        {
            int id;
            ped::VertexFloorLevel *v;
        };
        std::vector<LevelHandle> levels;
        std::vector<BiasHandle> biases; // P9-C3b: one per track with a bias vertex (written back to Map::SetPedBias)
        int nFloor = 0, nFloorNoObs = 0, nFloorNoZf = 0, nFloorNoNeighbours = 0, nFloorGrazing = 0;
        std::vector<g2o::OptimizableGraph::Edge *> edges;      // every edge added (for tests / logging)
        std::vector<g2o::OptimizableGraph::Edge *> byType[NTYPES];
        // counters (also printed once per call when non-zero)
        int nTracks = 0, nSkippedTracks = 0, nRoot = 0, nRange = 0, nWna = 0, nFoot = 0, nFootVertices = 0, nTie = 0, nDroppedDepth = 0;
    };

    class PedestrianBA
    {
    public:
        // `kfs` is the full graph: free and fixed keyframes whose pose vertices already exist in `opt` with id = KeyFrame::mnId.
        // `metric` = the map is metric and gravity-aligned (IMU initialised): range, motion and stance factors are added;
        // otherwise only the scale-free root reprojection is (pre-IMU-init maps have arbitrary scale; Marco, 20 Oct).
        // `mpVertexBase`: the map-point vertex of MapPoint mp has id mp->mnId + mpVertexBase in `opt` (LocalInertialBA: maxKFid * 5 + 1); only used by
        // Pedestrians.FloorContact: joint (Variant B), which also un-marginalises the chosen point vertices (a plane factor couples k landmarks).
        static PedGraph Add(g2o::SparseOptimizer &opt, const std::vector<PedGraphKF> &kfs, Map *pMap, bool metric, bool inertialPoses,
                            unsigned long mpVertexBase);

        // P9-C5: per-contact level log (modes levels | levels_shadow), text, hexfloat. One row per (keyframe, track), the last write:
        //   map_id kf_id stamp track hc level_id level_z transitional n_levels in_call
        // then `L` rows, the last state of every level seen: L map_id level_id z n_contacts n_f2_points_max n_merges_map
        static void DumpLevelsLog(const std::string &path);

        // Variant A: robust mode of the heights of the points `pts` (map frame, z up) inside the horizontal radius R of `centre` and lower than the
        // camera height `zCamera` by at least `belowCamera`: 3-bin-smoothed histogram (bin width `bin`), highest count (lowest bin on ties), median of
        // the points of the winning 3 bins. `lowModeFraction` q < 1: the winning window is instead the LOWEST local maximum of the smoothed histogram whose
        // count is >= q * (highest count) -- floors are the lowest large horizontal surface, table tops and beds are higher. q = 1: the highest count.
        // ok = false when fewer than `minPoints` points are in the region.
        struct FloorEstimate
        {
            bool ok = false;
            double zf = 0;
            int nRegion = 0, nMode = 0;
        };
        static FloorEstimate RegionalFloor(const std::vector<Eigen::Vector3d> &pts, const Eigen::Vector2d &centre, double zCamera, double R, double bin,
                                           double belowCamera, int minPoints, double lowModeFraction);

        // Last written floor-contact record of every (keyframe, track): text, hexfloat. Columns: kf_id stamp track zp(plane height) z_regional Gx Gy Gz
        // px py pz s nPts  (G = foot-ray / plane intersection, p = optimised root, s = ray depth).
        static void DumpFloorLog(const std::string &path);

        // P9-C3c: every midpoint contact the geometry and gates looked at (modes shadow / regional_px / joint_px), one row per (keyframe, track), text:
        //   kf_id stamp track stage zf_raw zf_glob elev_deg u v nRegion nQual sigQual res_u res_v
        // stage 0 = passed every gate (an edge exists unless the mode is `shadow`), 1 = no regional z_f, 2 = raised surface (zf_raw - zf_glob > FloorRaisedMaxM),
        // 3 = foot ray less than FloorMinElevDeg below the horizontal, 4 = fewer than k qualifying static points, 5 = plane scatter >= FloorPlaneSigmaMaxM
        // (stages 4, 5 only evaluated when the mode is joint_px or shadow); res_* = contact-pixel residual of the edge at the solution (0 if no edge).
        static void DumpFloorPxLog(const std::string &path);

        // P9-C4: every innovation-gate decision (written only when Pedestrians.InnovGate: 1): kf_id stamp track status tested dof d2 dt pred_stamp chain_idx consec_rej.
        static void DumpGateLog(const std::string &path);

        // Writes the optimised person states of free keyframes back. Call only after the BA succeeded.
        static void WriteBack(const PedGraph &g, Map *pMap);

        // Process-wide totals of person vertices / edges ever created (the flag-off check: both must stay 0).
        static void Counters(long &vertices, long &edges);

        // Last written estimate of every (keyframe, track) ever optimised, with the keyframe pose at that write (survives culling and
        // map resets; the final map export only holds keyframes that are still alive). Hexfloat text, ordered by (keyframe id, track).
        // Columns: kf_id stamp track px py pz vx vy vz tx ty tz qx qy qz qw (Twc)
        static void DumpLog(const std::string &path);

        // Per-factor-type accounting (P9-C2 step 2). Call after the solve: counts, per type, the edges created in this call and the
        // ones inside their robust threshold; keeps one record per call and process-wide totals.
        static void Account(const PedGraph &g, const char *kind, double stamp);
        static void DumpCounters(const std::string &path); // CSV: one row per BA call + a `total` row
    };
} // namespace ORB_SLAM3

#endif
