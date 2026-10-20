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
        std::vector<std::pair<int, ped::VertexStature *>> statures;
        enum Type { ROOT = 0, RANGE, STATURE, WNA, FOOT, T1, T2, NTYPES };
        std::vector<g2o::OptimizableGraph::Edge *> edges;      // every edge added (for tests / logging)
        std::vector<g2o::OptimizableGraph::Edge *> byType[NTYPES];
        // counters (also printed once per call when non-zero)
        int nTracks = 0, nSkippedTracks = 0, nRoot = 0, nRange = 0, nWna = 0, nFoot = 0, nFootVertices = 0, nTie = 0, nDroppedDepth = 0;
    };

    class PedestrianBA
    {
    public:
        // `kfs` is the full graph: free and fixed keyframes whose pose vertices already exist in `opt` with id = KeyFrame::mnId.
        // `metric` = the map is metric and gravity-aligned (IMU initialised): range, stature, motion and stance factors are added;
        // otherwise only the scale-free root reprojection is (pre-IMU-init maps have arbitrary scale; Marco, 20 Oct).
        static PedGraph Add(g2o::SparseOptimizer &opt, const std::vector<PedGraphKF> &kfs, Map *pMap, bool metric, bool inertialPoses);

        // Writes the optimised person states of free keyframes and the statures back. Call only after the BA succeeded.
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
