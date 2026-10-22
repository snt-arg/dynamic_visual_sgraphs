#include "PedestrianBA.h"
#include "PedestrianLevels.h"
#include "PedestrianFloorGate.h"
#include "PedestrianGate.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"

namespace ORB_SLAM3
{
    namespace
    {
        std::atomic<long> gVertices(0), gEdges(0);
        struct LogRec
        {
            double stamp;
            Eigen::Matrix<double, 6, 1> x;
            Eigen::Matrix<double, 7, 1> twc;
        };
        std::map<std::pair<unsigned long, int>, LogRec> gLog; // ordered: deterministic dump
        struct FloorRec
        {
            double stamp, zp, zRegional, s, b;
            Eigen::Vector3d G, p;
            int nPts;
        };
        std::map<std::pair<unsigned long, int>, FloorRec> gFloorLog;
        struct PxRec // P9-C3c: one midpoint contact seen by the geometry / gates
        {
            double stamp = 0, zfRaw = 0, zfGlob = 0, elev = 0, u = 0, v = 0, sigQual = 0, resU = 0, resV = 0;
            int stage = 0, nRegion = 0, nQual = 0;
        };
        std::map<std::pair<unsigned long, int>, PxRec> gPxLog;
        struct LevelRow // P9-C5
        {
            double stamp = 0, hc = 0, levelZ = 0;
            int level = -1, transitional = 0, nLevels = 0;
        };
        std::map<std::tuple<unsigned long, unsigned long, int>, LevelRow> gLevelLog; // (map id, kf id, track)
        struct LevelStat
        {
            double z = 0;
            int nContacts = 0, nPointsMax = 0, merges = 0;
        };
        std::map<std::pair<unsigned long, int>, LevelStat> gLevelStat; // (map id, level id)
        std::map<std::tuple<unsigned long, unsigned long, int>, double> gBiasRow; // P9-C5: (map id, kf id, track) -> the track's log-scale bias b at the last write
        struct PtRow // P9-C5 (levels_shadow only): a static map point within the RANSAC radius of a non-transitional contact, its latest position and nearest contact
        {
            double x, y, z, d;
            unsigned long kf;
            int track;
        };
        std::map<std::pair<unsigned long, unsigned long>, PtRow> gPtLog; // (map id, map point id)
        long gFloorSkipped[4] = {0, 0, 0, 0}; // no foot observation (not counted), no regional z_f, no neighbours, grazing ray
        struct GateRow
        {
            double stamp;
            PedGateRec rec;
        };
        std::map<std::pair<unsigned long, int>, GateRow> gGateLog; // P9-C4: the first (and only) decision per (keyframe, track)
        long gGateNoPred = 0;

        // P9-C4 (determinism diagnosis; OFF unless the environment variable P9C_DET_LOG names a file): one line per BA call with FNV-1a hashes of the inputs the
        // call consumed (the perception rows of the keyframes, the keyframe poses, the person vertices and edges it created) and, after the solve, of the written-back states.
        // P9C_DET_LOG_DETAIL=1 additionally writes every vertex and edge.
        struct DetLog
        {
            std::FILE *f = nullptr;
            bool detail = false;
            long n = 0;
            DetLog()
            {
                const char *p = std::getenv("P9C_DET_LOG");
                if (p && *p)
                    f = std::fopen(p, "w");
                const char *d = std::getenv("P9C_DET_LOG_DETAIL");
                detail = d && *d == '1';
            }
        };
        DetLog &DL()
        {
            static DetLog d;
            return d;
        }
        struct Fnv
        {
            std::uint64_t h = 1469598103934665603ULL;
            void bytes(const void *p, std::size_t n)
            {
                const unsigned char *c = static_cast<const unsigned char *>(p);
                for (std::size_t i = 0; i < n; ++i)
                {
                    h ^= c[i];
                    h *= 1099511628211ULL;
                }
            }
            template <class T> void val(const T &v) { bytes(&v, sizeof(T)); }
        };
        long gDetCall = 0;
        std::mutex gLogMutex;

        // Huber thresholds: the same chi2 95% values the local BAs use (Optimizer.cc:1994-1995 thHuber1D / thHuberMono).
        const double kHuber1D = std::sqrt(3.841);
        const double kHuber2D = std::sqrt(5.991);
        // A person state closer than this to the image plane is not a usable measurement (root reprojection has a 1/z Jacobian).
        const double kMinDepthM = 0.5;

        struct Entry
        {
            KeyFrame *kf;
            bool fixed;
            const PedObs *obs;
            ped::VertexPedState *v;
            PedClaim cl;     // late stance claim of this track at this keyframe (valid iff hasCl)
            bool hasCl = false;
            bool rangeOk = true; // false: the two range cues disagree (5b.2); the nose->heel range factor is dropped, root reprojection stays
            bool reinit = false; // P9-C4: the innovation gate re-initialised the track at this measurement (no WNA edge to the preceding entry)
        };
        void Tag(PedGraph &G, PedGraph::Type ty, g2o::OptimizableGraph::Edge *e)
        {
            G.edges.push_back(e);
            G.byType[ty].push_back(e);
        }

        struct Cam
        {
            double fx, fy, cx, cy;
        };

        Cam CameraOf(KeyFrame *kf)
        {
            if (kf->mpCamera->GetType() != GeometricCamera::CAM_PINHOLE)
                throw std::runtime_error("[Pedestrians] only the pinhole camera model is supported");
            if (cv::countNonZero(kf->mDistCoef) != 0)
                throw std::runtime_error("[Pedestrians] person pixels are distortion-free measurements; the camera has distortion");
            return {kf->fx, kf->fy, kf->cx, kf->cy};
        }

        template <class PoseV>
        PedGraph AddImpl(g2o::SparseOptimizer &opt, const std::vector<PedGraphKF> &kfsIn, Map *pMap, bool metric, unsigned long mpVertexBase)
        {
            const PedestrianParams &P = PedestrianParams::Get();
            PedGraph G;
            const bool floorOn = metric && P.floorMode != PedestrianParams::FloorMode::None;
            const bool levelsShadow = P.floorMode == PedestrianParams::FloorMode::LevelsShadow;
            const bool levelsMode = P.floorMode == PedestrianParams::FloorMode::Levels || levelsShadow;
            const bool pxMode = P.floorMode == PedestrianParams::FloorMode::Shadow || P.floorMode == PedestrianParams::FloorMode::RegionalPx || P.floorMode == PedestrianParams::FloorMode::JointPx;
            const bool shadowMode = P.floorMode == PedestrianParams::FloorMode::Shadow;
            const bool jointPx = P.floorMode == PedestrianParams::FloorMode::JointPx;

            // P9-C3: static map points of the map (non-bad, ordered by id -> deterministic); for Variant B also their BA vertices
            struct StaticPt
            {
                unsigned long id;
                Eigen::Vector3d x;
                g2o::VertexSBAPointXYZ *v;
            };
            std::vector<StaticPt> statics;
            std::vector<Eigen::Vector3d> staticXyz;
            if (floorOn)
            {
                if (!pMap)
                    throw std::runtime_error("[Pedestrians] floor contact needs the map");
                for (MapPoint *mp : pMap->GetAllMapPoints())
                {
                    if (!mp || mp->isBad())
                        continue;
                    g2o::VertexSBAPointXYZ *vv = nullptr;
                    if (P.floorMode == PedestrianParams::FloorMode::Joint || jointPx || levelsMode)
                        vv = dynamic_cast<g2o::VertexSBAPointXYZ *>(opt.vertex(static_cast<int>(mp->mnId + mpVertexBase)));
                    statics.push_back({mp->mnId, mp->GetWorldPos().cast<double>(), vv});
                }
                std::sort(statics.begin(), statics.end(), [](const StaticPt &a, const StaticPt &b) { return a.id < b.id; });
                staticXyz.reserve(statics.size());
                for (const StaticPt &sp : statics)
                    staticXyz.push_back(sp.x);
            }

            // 1. per-track keyframe lists, ordered by time (then keyframe id) -> deterministic
            std::vector<PedGraphKF> kfs = kfsIn;
            std::sort(kfs.begin(), kfs.end(), [](const PedGraphKF &a, const PedGraphKF &b) {
                if (a.kf->mTimeStamp != b.kf->mTimeStamp)
                    return a.kf->mTimeStamp < b.kf->mTimeStamp;
                return a.kf->mnId < b.kf->mnId;
            });
            std::map<int, std::vector<Entry>> tracks;
            for (const PedGraphKF &g : kfs)
            {
                if (!g.kf->mpPedFrame || g.kf->isBad())
                    continue;
                for (const PedObs &o : g.kf->mpPedFrame->obs)
                {
                    // An observation without a nose->heel length has no depth cue: its state would be free along the pixel ray
                    // (root reprojection is flat along it, the 1/z Jacobians diverge as the state approaches the camera; seen in the
                    // first smoke run). It is not a measurement of this system and is skipped.
                    if (o.lpx <= 0 || o.lpx < PedestrianParams::Get().minLpx)
                        continue;
                    {
                        Entry en{g.kf, g.fixed, &o, nullptr, PedClaim(), false, true};
                        en.hasCl = g.kf->GetPedClaim(o.trackId, en.cl);
                        if (PedestrianParams::Get().gateK > 0 && o.maskH > 0)
                        {
                            const PedestrianParams &Q = PedestrianParams::Get();
                            const double d = std::log(Q.skelOverStature * Q.statureFixedM / o.lpx) - std::log(Q.maskHeightM / o.maskH); // f cancels
                            en.rangeOk = std::abs(d - Q.gateCenter) <= Q.gateK * Q.gateSigma;
                        }
                        tracks[o.trackId].push_back(en);
                    }
                }
            }

            // P9-C4 prediction-based innovation gate (metric maps only): one decision per new (keyframe, track) measurement, in time order, against the
            // last written-back state of the track's preceding accepted keyframe; REJECTED measurements leave the graph (no state, no factor), a REINIT starts a new chain.
            if (metric && P.innovGate)
                for (auto &tr : tracks)
                {
                    const int track = tr.first;
                    std::vector<Entry> kept;
                    ped::GateChain ch;
                    KeyFrame *lastAcc = nullptr;
                    for (Entry &e : tr.second)
                    {
                        PedGateRec rec;
                        if (e.kf->GetPedGate(track, rec))
                        {
                            ch.idx = rec.chainIdx;
                            ch.consecRej = rec.consecRej;
                        }
                        else
                        {
                            PedState ps;
                            bool havePred = lastAcc && lastAcc->GetPedState(track, ps);
                            ped::GateTest t;
                            double dt = 0;
                            if (havePred)
                            {
                                dt = e.kf->mTimeStamp - lastAcc->mTimeStamp;
                                if (!(dt > 1e-6))
                                    havePred = false;
                                else
                                {
                                    const Cam c = CameraOf(e.kf);
                                    const Sophus::SE3f Tcw = e.kf->GetPose();
                                    const int kAge = ch.idx + 1; // chain age of this measurement (the first gated one is 3 with 2 free keyframes)
                                    const double sk = P.innovScale[std::min(std::max(kAge, 3), 6) - 3];
                                    t = ped::InnovationTest(P, ps.p, ps.v, dt, Tcw.rotationMatrix().cast<double>(), Tcw.translation().cast<double>(), c.fx, c.fy, c.cx, c.cy,
                                                            e.obs->rootU, e.obs->rootV, e.rangeOk,
                                                            std::log(c.fx) + std::log(P.skelOverStature) + std::log(P.statureFixedM) - std::log(e.obs->lpx), sk);
                                }
                            }
                            if (P.innovShadow)
                            {
                                // log-only (InnovGate: 2): the test is computed at decision time against the predecessor's current written-back state and logged
                                // (d2, dof, dt; `tested` = the measurement would have been gated: chain age >= free + 1), every measurement is accepted
                                rec = PedGateRec();
                                rec.dt = dt;
                                rec.predStamp = lastAcc ? lastAcc->mTimeStamp : 0.0;
                                rec.status = PedGateRec::ACCEPTED;
                                if (havePred && t.ok)
                                {
                                    rec.dof = t.dof;
                                    rec.d2 = t.d2;
                                    rec.tested = ch.idx >= P.innovFreeKeyframes;
                                }
                                ++ch.idx;
                                ch.consecRej = 0;
                                rec.chainIdx = ch.idx;
                                rec.consecRej = 0;
                            }
                            else
                                rec = ped::GateStep(P, ch, havePred, t, dt, lastAcc ? lastAcc->mTimeStamp : 0.0);
                            e.kf->SetPedGate(track, rec);
                            std::lock_guard<std::mutex> lk(gLogMutex);
                            gGateLog[{e.kf->mnId, track}] = {e.kf->mTimeStamp, rec};
                            if (!rec.tested && rec.chainIdx > P.innovFreeKeyframes)
                                ++gGateNoPred;
                        }
                        if (rec.status == PedGateRec::REJECTED)
                            continue;
                        e.reinit = rec.status == PedGateRec::REINIT;
                        kept.push_back(e);
                        lastAcc = e.kf;
                    }
                    tr.second = kept;
                }

            int nextId = 0; // above every id in the graph (the container is unordered; the maximum does not depend on its order)
            for (const auto &kv : opt.vertices())
                nextId = std::max(nextId, kv.first + 1);

            for (auto &tr : tracks)
            {
                std::vector<Entry> &E = tr.second;
                const bool anyFree = std::any_of(E.begin(), E.end(), [](const Entry &e) { return !e.fixed; });
                const int nValid = static_cast<int>(std::count_if(E.begin(), E.end(), [](const Entry &e) { return e.rangeOk; }));
                if (nValid < P.minTrackKeyframes || !anyFree)
                {
                    ++G.nSkippedTracks;
                    continue;
                }
                ++G.nTracks;
                const int track = tr.first;

                const double Hcur = P.statureFixedM; // fixed stature (P9-C2 5d): not a state

                // P9-C3b: per-track log-scale bias b_j (Variants A / B only; "none" keeps b = 0 and the 2-vertex range edge)
                ped::VertexPedBias *vbias = nullptr;
                if (floorOn && !shadowMode && !levelsShadow)
                {
                    const int contacts = static_cast<int>(std::count_if(E.begin(), E.end(), [](const Entry &en) { return en.obs->hasFoot && !en.fixed; }));
                    if (contacts >= P.biasMinContacts)
                    {
                        double b0 = 0.0;
                        if (pMap)
                            pMap->GetPedBias(track, b0);
                        vbias = new ped::VertexPedBias();
                        vbias->setId(nextId++);
                        vbias->setEstimate(b0);
                        vbias->setFixed(false);
                        opt.addVertex(vbias);
                        auto *pr = new ped::EdgePedBiasPrior();
                        pr->setVertex(0, vbias);
                        pr->setMeasurement(0.0);
                        pr->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.biasSigma * P.biasSigma));
                        opt.addEdge(pr);
                        Tag(G, PedGraph::BIAS, pr);
                        G.biases.push_back({track, vbias});
                    }
                }

                // 2. state vertices (existing estimate, else initialised from the keyframe's pose, the root pixel and the range)
                for (Entry &e : E)
                {
                    const Cam c = CameraOf(e.kf);
                    PedState s;
                    if (!e.kf->GetPedState(track, s))
                    {
                        const Eigen::Vector3d b((e.obs->rootU - c.cx) / c.fx, (e.obs->rootV - c.cy) / c.fy, 1.0);
                        // optical-axis depth z = f cL H / l_px (P9-C3b erratum: the length is inversely proportional to z, not to the ray length);
                        // without a skeleton length the mask height gives an initial depth only (the estimate is refined by the BA)
                        const double z = e.rangeOk ? (c.fx * P.skelOverStature * Hcur / e.obs->lpx)
                                                   : (c.fx * P.maskHeightM / std::max(e.obs->maskH, 1.0));
                        const Sophus::SE3f Twc = e.kf->GetPoseInverse();
                        s.p = (Twc.rotationMatrix() * (z * b).cast<float>() + Twc.translation()).cast<double>();
                        s.v.setZero();
                    }
                    e.v = new ped::VertexPedState();
                    e.v->setId(nextId++);
                    ped::Vec6 x;
                    x << s.p, s.v;
                    e.v->setEstimate(x);
                    e.v->setFixed(e.fixed);
                    opt.addVertex(e.v);
                    G.states.push_back({track, e.kf, e.v, e.fixed});
                }

                // 3. root reprojection (all modes) and log-range (metric)
                for (const Entry &e : E)
                {
                    auto *vpose = dynamic_cast<PoseV *>(opt.vertex(e.kf->mnId));
                    if (!vpose)
                        throw std::runtime_error("[Pedestrians] keyframe pose vertex missing from the graph");
                    const Cam c = CameraOf(e.kf);
                    {
                        auto *edge = new ped::EdgePedReproj<ped::VertexPedState, PoseV>(c.fx, c.fy, c.cx, c.cy);
                        edge->setVertex(0, e.v);
                        edge->setVertex(1, vpose);
                        edge->setMeasurement(Eigen::Vector2d(e.obs->rootU, e.obs->rootV));
                        edge->setInformation(Eigen::Matrix2d::Identity() / (P.rootPxSigma * P.rootPxSigma));
                        if (edge->Xc()(2) < kMinDepthM)
                        {
                            ++G.nDroppedDepth;
                            delete edge;
                        }
                        else if (e.fixed)
                            delete edge; // fixed state and fixed pose: no free variable
                        else
                        {
                            auto *rk = new g2o::RobustKernelHuber();
                            rk->setDelta(kHuber2D * P.huberScale);
                            edge->setRobustKernel(rk);
                            opt.addEdge(edge);
                            Tag(G, PedGraph::ROOT, edge);
                            ++G.nRoot;
                        }
                    }
                    if (metric && e.rangeOk)
                    {
                        auto *edge = new ped::EdgePedLogRange<PoseV>(vbias != nullptr);
                        edge->setVertex(0, e.v);
                        edge->setVertex(1, vpose);
                        if (vbias)
                            edge->setVertex(2, vbias);
                        edge->setMeasurement(std::log(c.fx) + std::log(P.skelOverStature) + std::log(P.statureFixedM) - std::log(e.obs->lpx));
                        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.logDepthSigma * P.logDepthSigma));
                        auto *rk = new g2o::RobustKernelHuber();
                        rk->setDelta(kHuber1D * P.huberScale);
                        edge->setRobustKernel(rk);
                        opt.addEdge(edge);
                        Tag(G, PedGraph::RANGE, edge);
                        ++G.nRange;
                    }
                    if (floorOn && pxMode && e.obs->hasFoot && !e.fixed)
                    {
                        // P9-C3c midpoint contact: regional z_f (bias-corrected), raised-surface rejection, elevation gate, B': local plane through the qualifying static points
                        const Sophus::SE3f Twc = e.kf->GetPoseInverse();
                        const Eigen::Vector3d cam = Twc.translation().cast<double>();
                        const Eigen::Vector3d p0 = e.v->p();
                        PxRec rec;
                        rec.stamp = e.kf->mTimeStamp;
                        rec.u = e.obs->footU;
                        rec.v = e.obs->footV;
                        std::vector<g2o::VertexSBAPointXYZ *> nb;
                        double zfc = 0;
                        const PedestrianBA::FloorEstimate fe =
                            PedestrianBA::RegionalFloor(staticXyz, p0.template head<2>(), cam(2), P.floorRegionRadiusM, P.floorBinM, P.floorBelowCameraM, P.floorMinPoints, P.floorLowModeFraction);
                        if (!fe.ok)
                        {
                            rec.stage = 1;
                            ++G.nPxNoZf;
                        }
                        else
                        {
                            rec.zfRaw = fe.zf;
                            rec.nRegion = fe.nRegion;
                            const PedestrianBA::FloorEstimate gl =
                                PedestrianBA::RegionalFloor(staticXyz, p0.template head<2>(), cam(2), 1e6, P.floorBinM, P.floorBelowCameraM, P.floorMinPoints, P.floorLowModeFraction);
                            rec.zfGlob = gl.ok ? gl.zf : fe.zf;
                            zfc = fe.zf - P.floorZfBiasM;
                            const Eigen::Vector3d bray((e.obs->footU - c.cx) / c.fx, (e.obs->footV - c.cy) / c.fy, 1.0);
                            const Eigen::Vector3d dw = (Twc.rotationMatrix() * bray.cast<float>()).cast<double>();
                            rec.elev = std::asin(std::max(-1.0, std::min(1.0, -dw(2) / dw.norm()))) * 180.0 / M_PI;
                            rec.stage = ped::FloorStageA(P, fe.zf, rec.zfGlob, rec.elev);
                            G.nPxRaised += rec.stage == 2;
                            G.nPxElev += rec.stage == 3;
                            if (rec.stage == 0 && P.floorMode != PedestrianParams::FloorMode::RegionalPx)
                            {
                                // qualifying points: within the band of the corrected z_f and within the radius of the contact (horizontal distance to the root state)
                                std::vector<std::pair<double, size_t>> cand;
                                std::vector<double> zq;
                                for (size_t q = 0; q < statics.size(); ++q)
                                {
                                    const StaticPt &sp = statics[q];
                                    if (std::abs(sp.x(2) - zfc) > P.floorBandM)
                                        continue;
                                    const double d = (sp.x.template head<2>() - p0.template head<2>()).norm();
                                    if (d > P.floorNeighbourRadiusM)
                                        continue;
                                    zq.push_back(sp.x(2));
                                    if (sp.v && !sp.v->fixed())
                                        cand.push_back({d, q});
                                }
                                rec.nQual = static_cast<int>(zq.size());
                                if (rec.nQual > 0)
                                {
                                    double m = 0;
                                    for (double z : zq)
                                        m += z;
                                    m /= zq.size();
                                    double ss = 0;
                                    for (double z : zq)
                                        ss += (z - m) * (z - m);
                                    rec.sigQual = std::sqrt(ss / zq.size());
                                }
                                rec.stage = ped::FloorStageB(P, rec.nQual, rec.sigQual);
                                G.nPxNoNeighbours += rec.stage == 4;
                                G.nPxPlane += rec.stage == 5;
                                if (rec.stage == 0 && jointPx)
                                {
                                    std::sort(cand.begin(), cand.end(), [&](const std::pair<double, size_t> &a, const std::pair<double, size_t> &b) {
                                        return a.first != b.first ? a.first < b.first : statics[a.second].id < statics[b.second].id;
                                    });
                                    if (static_cast<int>(cand.size()) < P.floorNeighbours)
                                    {
                                        rec.stage = 4;
                                        ++G.nPxNoNeighbours;
                                    }
                                    else
                                        for (int q = 0; q < P.floorNeighbours; ++q)
                                            nb.push_back(statics[cand[q].second].v);
                                }
                            }
                        }
                        if (rec.stage == 0 && !shadowMode)
                        {
                            const int k = static_cast<int>(nb.size());
                            auto *fl = new ped::EdgePedFloorPx<PoseV>(c.fx, c.fy, c.cx, c.cy, e.obs->footU, e.obs->footV, k, zfc, P.floorMidDeltaM, /*poseGradient=*/jointPx);
                            fl->setVertex(0, e.v);
                            fl->setVertex(1, vpose);
                            for (int q = 0; q < k; ++q)
                            {
                                nb[q]->setMarginalized(false); // the plane height couples k landmarks: they cannot stay in the Schur landmark block
                                fl->setVertex(2 + q, nb[q]);
                            }
                            fl->setMeasurement(Eigen::Vector2d(e.obs->footU, e.obs->footV));
                            Eigen::Matrix2d I = Eigen::Matrix2d::Zero();
                            I(0, 0) = 1.0 / (P.floorSigmaPxU * P.floorSigmaPxU);
                            I(1, 1) = 1.0 / (P.floorSigmaPxV * P.floorSigmaPxV);
                            fl->setInformation(I);
                            auto *rk = new g2o::RobustKernelHuber();
                            rk->setDelta(kHuber2D * P.huberScale);
                            fl->setRobustKernel(rk);
                            opt.addEdge(fl);
                            Tag(G, PedGraph::FLOOR, fl);
                            ++G.nFloor;
                            G.pxFloors.push_back({e.kf->mnId, track, fl});
                        }
                        std::lock_guard<std::mutex> lk(gLogMutex);
                        gPxLog[{e.kf->mnId, track}] = rec;
                    }
                    if (floorOn && !levelsMode && !pxMode && e.obs->hasFoot && !e.fixed)
                    {
                        const Sophus::SE3f Twc = e.kf->GetPoseInverse();
                        const Eigen::Vector3d cam = Twc.translation().cast<double>();
                        const Eigen::Vector3d p0 = e.v->p();
                        const PedestrianBA::FloorEstimate fe =
                            PedestrianBA::RegionalFloor(staticXyz, p0.template head<2>(), cam(2), P.floorRegionRadiusM, P.floorBinM, P.floorBelowCameraM, P.floorMinPoints, P.floorLowModeFraction);
                        if (!fe.ok)
                        {
                            ++G.nFloorNoZf;
                            ++gFloorSkipped[1];
                        }
                        else
                        {
                            const bool joint = P.floorMode == PedestrianParams::FloorMode::Joint;
                            std::vector<g2o::VertexSBAPointXYZ *> nb;
                            // ground point of the ray on the regional plane (build-time geometry: the noise frame and the Variant B neighbourhood)
                            const Eigen::Vector3d bray((e.obs->footU - c.cx) / c.fx, (e.obs->footV - c.cy) / c.fy, 1.0);
                            const Eigen::Vector3d dw = (Twc.rotationMatrix() * bray.cast<float>()).cast<double>();
                            if (dw(2) / dw.norm() > -0.02) // ray less than ~1.1 degrees below the horizon
                            {
                                ++G.nFloorGrazing;
                                ++gFloorSkipped[3];
                            }
                            else
                            {
                                const double sg = (fe.zf - cam(2)) / dw(2);
                                const Eigen::Vector3d G0 = cam + sg * dw;
                                bool usable = true;
                                if (joint)
                                {
                                    std::vector<std::pair<double, size_t>> cand;
                                    for (size_t q = 0; q < statics.size(); ++q)
                                    {
                                        const StaticPt &sp = statics[q];
                                        if (!sp.v || sp.v->fixed() || std::abs(sp.x(2) - fe.zf) > P.floorBandM)
                                            continue;
                                        const double d = (sp.x.template head<2>() - G0.template head<2>()).norm();
                                        if (d <= P.floorNeighbourRadiusM)
                                            cand.push_back({d, q});
                                    }
                                    std::sort(cand.begin(), cand.end(), [&](const std::pair<double, size_t> &a, const std::pair<double, size_t> &b) {
                                        return a.first != b.first ? a.first < b.first : statics[a.second].id < statics[b.second].id;
                                    });
                                    if (static_cast<int>(cand.size()) < P.floorNeighbours)
                                    {
                                        usable = false;
                                        ++G.nFloorNoNeighbours;
                                        ++gFloorSkipped[2];
                                    }
                                    else
                                        for (int q = 0; q < P.floorNeighbours; ++q)
                                            nb.push_back(statics[cand[q].second].v);
                                }
                                if (usable)
                                {
                                    const int k = static_cast<int>(nb.size());
                                    auto *fl = new ped::EdgePedFloorContact<PoseV>(c.fx, c.fy, c.cx, c.cy, e.obs->footU, e.obs->footV, k, fe.zf, P.floorContactOffsetM, /*poseGradient=*/joint);
                                    fl->setVertex(0, e.v);
                                    fl->setVertex(1, vpose);
                                    for (int q = 0; q < k; ++q)
                                    {
                                        nb[q]->setMarginalized(false); // a plane factor couples k landmarks: they cannot stay in the Schur landmark block
                                        fl->setVertex(2 + q, nb[q]);
                                    }
                                    fl->setMeasurement(Eigen::Vector2d(e.obs->footU, e.obs->footV));
                                    const double phi = std::atan2(G0(1) - cam(1), G0(0) - cam(0));
                                    const Eigen::Vector2d ur(std::cos(phi), std::sin(phi)), ut(-std::sin(phi), std::cos(phi));
                                    fl->setInformation(ur * ur.transpose() / (P.floorSigmaRadialM * P.floorSigmaRadialM) +
                                                       ut * ut.transpose() / (P.floorSigmaTangentialM * P.floorSigmaTangentialM));
                                    auto *rk = new g2o::RobustKernelHuber();
                                    rk->setDelta(kHuber2D * P.huberScale);
                                    fl->setRobustKernel(rk);
                                    opt.addEdge(fl);
                                    Tag(G, PedGraph::FLOOR, fl);
                                    ++G.nFloor;
                                    G.floors.push_back({track, e.kf, fl, std::is_same<PoseV, VertexPose>::value, fe.zf});
                                }
                            }
                        }
                    }
                }

                if (!metric)
                    continue;

                // 4. white-noise-acceleration prior between consecutive states of the track
                for (size_t i = 0; i + 1 < E.size(); ++i)
                {
                    if (E[i].fixed && E[i + 1].fixed)
                        continue;
                    if (E[i + 1].reinit)
                        continue; // P9-C4: a re-initialised track is a new track: no prior to the state before the jump
                    const double dt = E[i + 1].kf->mTimeStamp - E[i].kf->mTimeStamp;
                    if (!(dt > 1e-6))
                        continue;
                    auto *edge = new ped::EdgePedWna();
                    edge->setVertex(0, E[i].v);
                    edge->setVertex(1, E[i + 1].v);
                    edge->setMeasurement(ped::Vec6::Zero());
                    edge->SetDt(dt, ped::ActiveQ(P));
                    opt.addEdge(edge);
                    Tag(G, PedGraph::WNA, edge);
                    ++G.nWna;
                }

                // 5. stance: footprint vertex per run of consecutive claimed, sufficiently tall entries of one foot
                for (int foot = 0; foot < 2; ++foot)
                {
                    size_t i = 0;
                    while (i < E.size())
                    {
                        auto ok = [&](const Entry &e) { return e.hasCl && e.cl.claim[foot] && e.cl.boxH >= P.minStanceBoxPx; };
                        if (!ok(E[i]))
                        {
                            ++i;
                            continue;
                        }
                        size_t j = i;
                        while (j < E.size() && ok(E[j]))
                            ++j;
                        if (static_cast<int>(j - i) >= P.minFootObservations)
                        {
                            // initial footprint: the first claimed toe ray meets the plane one root-height below the root state
                            const Entry &e0 = E[i];
                            const Cam c = CameraOf(e0.kf);
                            const Sophus::SE3f Twc = e0.kf->GetPoseInverse();
                            const Eigen::Vector3d b((e0.cl.toeU[foot] - c.cx) / c.fx, (e0.cl.toeV[foot] - c.cy) / c.fy, 1.0);
                            const Eigen::Vector3d d = (Twc.rotationMatrix() * b.cast<float>()).cast<double>();
                            const Eigen::Vector3d C = Twc.translation().cast<double>();
                            const Eigen::Vector3d p0 = e0.v->p();
                            const double zf = p0(2) - P.cRoot * Hcur;
                            Eigen::Vector3d X0 = p0 - Eigen::Vector3d(0, 0, P.cRoot * Hcur);
                            if (std::abs(d(2)) > 1e-6)
                            {
                                const double s = (zf - C(2)) / d(2);
                                if (s > 0)
                                    X0 = C + s * d;
                            }
                            auto *VX = new g2o::VertexSBAPointXYZ();
                            VX->setId(nextId++);
                            VX->setEstimate(X0);
                            VX->setFixed(false);
                            opt.addVertex(VX);
                            ++G.nFootVertices;
                            for (size_t k = i; k < j; ++k)
                            {
                                const Entry &e = E[k];
                                auto *vpose = dynamic_cast<PoseV *>(opt.vertex(e.kf->mnId));
                                const Cam ck = CameraOf(e.kf);
                                auto *re = new ped::EdgePedReproj<g2o::VertexSBAPointXYZ, PoseV>(ck.fx, ck.fy, ck.cx, ck.cy);
                                re->setVertex(0, VX);
                                re->setVertex(1, vpose);
                                re->setMeasurement(Eigen::Vector2d(e.cl.toeU[foot], e.cl.toeV[foot]));
                                re->setInformation(Eigen::Matrix2d::Identity() / (P.footPxSigma * P.footPxSigma));
                                auto *rk = new g2o::RobustKernelHuber();
                                rk->setDelta(kHuber2D * P.huberScale);
                                re->setRobustKernel(rk);
                                opt.addEdge(re);
                                Tag(G, PedGraph::FOOT, re);
                                ++G.nFoot;

                                auto *t1 = new ped::EdgePedTie1(P.cRoot, P.statureFixedM);
                                t1->setVertex(0, e.v);
                                t1->setVertex(1, VX);
                                t1->setMeasurement(0.0);
                                t1->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.t1Sigma * P.t1Sigma));
                                auto *rk1 = new g2o::RobustKernelHuber();
                                rk1->setDelta(kHuber1D * P.huberScale);
                                t1->setRobustKernel(rk1);
                                opt.addEdge(t1);
                                Tag(G, PedGraph::T1, t1);

                                auto *t2 = new ped::EdgePedTie2(P.cLeg, P.statureFixedM);
                                t2->setVertex(0, e.v);
                                t2->setVertex(1, VX);
                                t2->setMeasurement(0.0);
                                t2->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.t2Sigma * P.t2Sigma));
                                auto *rk2 = new g2o::RobustKernelHuber();
                                rk2->setDelta(kHuber1D * P.huberScale);
                                t2->setRobustKernel(rk2);
                                opt.addEdge(t2);
                                Tag(G, PedGraph::T2, t2);
                                G.nTie += 2;
                            }
                        }
                        i = j;
                    }
                }
            }

            // ------------------------------------------------------------------------------------------------ P9-C5 floor levels
            if (levelsMode && floorOn)
            {
                const ped::LevelParams LP = ped::DesignLevelParams(P.levelSigmaA, P.levelSigmaPt, P.levelSMax);
                // implied contact heights h_c from the current person states (tracks in id order, entries in time order: deterministic)
                std::vector<ped::Contact> cs;
                std::vector<const Entry *> ce;
                for (auto &tr : tracks)
                    for (const Entry &e : tr.second)
                    {
                        if (!e.obs->hasFoot || !e.v)
                            continue;
                        const Cam c = CameraOf(e.kf);
                        const Sophus::SE3f Twc = e.kf->GetPoseInverse();
                        const Eigen::Vector3d cam = Twc.translation().cast<double>();
                        const Eigen::Vector3d bray((e.obs->footU - c.cx) / c.fx, (e.obs->footV - c.cy) / c.fy, 1.0);
                        const Eigen::Vector3d dw = (Twc.rotationMatrix() * bray.cast<float>()).cast<double>();
                        const Eigen::Vector3d p0 = e.v->p();
                        const double dh2 = dw(0) * dw(0) + dw(1) * dw(1);
                        // the foot ray must point down by more than ~1.1 deg (design constant, the grazing limit of Variant A) and have a horizontal component
                        if (dw(2) / dw.norm() > -0.02 || dh2 < 1e-12)
                            continue;
                        // ray point nearest (horizontally) to the vertical line through the root state; its height minus delta_v
                        const double sr = (dw(0) * (p0(0) - cam(0)) + dw(1) * (p0(1) - cam(1))) / dh2;
                        if (!(sr > 0))
                            continue;
                        ped::Contact k;
                        k.track = tr.first;
                        k.kf = e.kf->mnId;
                        k.t = e.kf->mTimeStamp;
                        k.x = p0(0);
                        k.y = p0(1);
                        k.h = cam(2) + sr * dw(2) - P.floorMidDeltaM;
                        cs.push_back(k);
                        ce.push_back(&e);
                    }
                std::lock_guard<std::mutex> lvLock(pMap->PedLevelsMutex());
                ped::LevelSet &LS = pMap->PedLevels(LP);
                ped::FlagTransitional(cs, LP);
                LS.Update(cs);
                const unsigned long mapId = pMap->GetId();
                std::map<int, ped::VertexFloorLevel *> lvVert;
                std::map<int, std::vector<size_t>> lvContacts; // level id -> indices into cs (non-transitional, free state)
                for (size_t i = 0; i < cs.size(); ++i)
                    if (cs[i].level >= 0 && !ce[i]->fixed)
                        lvContacts[cs[i].level].push_back(i);
                for (const auto &lv : LS.Levels())
                {
                    LevelStat &st = gLevelStat[{mapId, lv.id}];
                    st.z = lv.z;
                    st.nContacts = lv.nContacts;
                    st.merges = LS.Merges();
                }
                for (size_t i = 0; i < cs.size(); ++i)
                {
                    LevelRow lr;
                    lr.stamp = cs[i].t;
                    lr.hc = cs[i].h;
                    lr.level = cs[i].level;
                    lr.transitional = cs[i].transitional ? 1 : 0;
                    lr.nLevels = static_cast<int>(LS.Levels().size());
                    for (const auto &lv : LS.Levels())
                        if (lv.id == cs[i].level)
                            lr.levelZ = lv.z;
                    std::lock_guard<std::mutex> lk(gLogMutex);
                    gLevelLog[{mapId, cs[i].kf, cs[i].track}] = lr;
                }
                if (levelsShadow)
                {
                    // calibration log of the points a level's RANSAC would see (sigma_pt, F2 coverage): points within ransacRadiusM of any non-transitional contact
                    const double r2 = LP.ransacRadiusM * LP.ransacRadiusM;
                    for (const StaticPt &sp : statics)
                    {
                        double bd = r2;
                        long bi = -1;
                        for (size_t i = 0; i < cs.size(); ++i)
                        {
                            if (cs[i].transitional)
                                continue;
                            const double d2 = (sp.x(0) - cs[i].x) * (sp.x(0) - cs[i].x) + (sp.x(1) - cs[i].y) * (sp.x(1) - cs[i].y);
                            if (d2 <= bd)
                            {
                                bd = d2;
                                bi = static_cast<long>(i);
                            }
                        }
                        if (bi >= 0)
                        {
                            std::lock_guard<std::mutex> lk(gLogMutex);
                            gPtLog[{mapId, sp.id}] = {sp.x(0), sp.x(1), sp.x(2), std::sqrt(bd), cs[bi].kf, cs[bi].track};
                        }
                    }
                }
                if (!levelsShadow)
                {
                    for (const auto &lc : lvContacts)
                    {
                        double z0 = 0;
                        for (const auto &lv : LS.Levels())
                            if (lv.id == lc.first)
                                z0 = lv.z;
                        auto *vl = new ped::VertexFloorLevel();
                        vl->setId(nextId++);
                        vl->setEstimate(z0);
                        vl->setFixed(false);
                        opt.addVertex(vl);
                        lvVert[lc.first] = vl;
                        G.levels.push_back({lc.first, vl});
                        // F1 contact factors
                        for (size_t i : lc.second)
                        {
                            const Entry &e = *ce[i];
                            auto *vpose = dynamic_cast<PoseV *>(opt.vertex(e.kf->mnId));
                            if (!vpose)
                                throw std::runtime_error("[Pedestrians] keyframe pose vertex missing from the graph");
                            const Cam c = CameraOf(e.kf);
                            auto *f1 = new ped::EdgePedLevelPx<PoseV>(c.fx, c.fy, c.cx, c.cy, e.obs->footU, e.obs->footV, P.floorMidDeltaM);
                            f1->setVertex(0, e.v);
                            f1->setVertex(1, vpose);
                            f1->setVertex(2, vl);
                            Eigen::Matrix2d I = Eigen::Matrix2d::Zero();
                            I(0, 0) = 1.0 / (P.floorSigmaPxU * P.floorSigmaPxU);
                            I(1, 1) = 1.0 / (P.floorSigmaPxV * P.floorSigmaPxV);
                            f1->setInformation(I);
                            auto *rk = new g2o::RobustKernelHuber();
                            rk->setDelta(kHuber2D * P.huberScale);
                            f1->setRobustKernel(rk);
                            opt.addEdge(f1);
                            Tag(G, PedGraph::FLOOR, f1);
                            ++G.nFloor;
                            G.pxFloors.push_back({e.kf->mnId, cs[i].track, f1});
                            PxRec rec;
                            rec.stamp = e.kf->mTimeStamp;
                            rec.u = e.obs->footU;
                            rec.v = e.obs->footV;
                            std::lock_guard<std::mutex> lk(gLogMutex);
                            gPxLog[{e.kf->mnId, cs[i].track}] = rec;
                        }
                        // F2: RANSAC points of the level (never creates a level)
                        std::vector<ped::PointZ> pz;
                        pz.reserve(statics.size());
                        for (size_t q = 0; q < statics.size(); ++q)
                            pz.push_back({static_cast<int>(q), statics[q].x(0), statics[q].x(1), statics[q].x(2)});
                        std::vector<ped::Contact> mine;
                        for (size_t i : lc.second)
                            mine.push_back(cs[i]);
                        const ped::PlaneFit fit = ped::RansacLevelPoints(pz, mine, z0, LP);
                        if (fit.ok)
                        {
                            std::vector<std::pair<double, int>> order;
                            for (int q : fit.inliers)
                            {
                                double dmin = 1e18;
                                for (const auto &m : mine)
                                    dmin = std::min(dmin, std::hypot(statics[q].x(0) - m.x, statics[q].x(1) - m.y));
                                order.push_back({dmin, q});
                            }
                            std::sort(order.begin(), order.end(), [&](const std::pair<double, int> &a, const std::pair<double, int> &b) {
                                return a.first != b.first ? a.first < b.first : statics[a.second].id < statics[b.second].id;
                            });
                            if (order.size() > 50)
                                order.resize(50);
                            for (const auto &o : order)
                            {
                                g2o::VertexSBAPointXYZ *pv = statics[o.second].v;
                                if (pv)
                                    pv->setMarginalized(false); // a level couples its points: they cannot stay in the Schur landmark block
                                else
                                {
                                    pv = new g2o::VertexSBAPointXYZ(); // not in this BA's point set: fixed
                                    pv->setId(nextId++);
                                    pv->setEstimate(statics[o.second].x);
                                    pv->setFixed(true);
                                    opt.addVertex(pv);
                                }
                                auto *f2 = new ped::EdgePedPointOnLevel();
                                f2->setVertex(0, pv);
                                f2->setVertex(1, vl);
                                f2->setMeasurement(0.0);
                                f2->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.levelSigmaPt * P.levelSigmaPt));
                                auto *rk = new g2o::RobustKernelHuber();
                                rk->setDelta(kHuber1D * P.huberScale);
                                f2->setRobustKernel(rk);
                                opt.addEdge(f2);
                                Tag(G, PedGraph::LVPT, f2);
                            }
                            std::lock_guard<std::mutex> lk(gLogMutex);
                            LevelStat &st = gLevelStat[{mapId, lc.first}];
                            st.nPointsMax = std::max(st.nPointsMax, static_cast<int>(order.size()));
                        }
                    }
                    // F3: platform height (levels whose height is within 3 sigma_a of "camera height - h_cam" for the keyframe; see report: the spatial
                    // "under the camera" test is replaced by this height gate because walkers are never under the camera)
                    if (P.platformHeight)
                        for (const PedGraphKF &g : kfs)
                        {
                            if (!g.kf || g.kf->isBad())
                                continue;
                            auto *vpose = dynamic_cast<PoseV *>(opt.vertex(g.kf->mnId));
                            if (!vpose)
                                continue;
                            const double zc = ped::EdgePedPlatformHeight<PoseV>::CamZ(vpose);
                            int best = -1;
                            double bd = 3.0 * P.levelSigmaA;
                            for (const auto &lv : lvVert)
                            {
                                const double d = std::abs(zc - lv.second->estimate() - P.platformHeightM);
                                if (d < bd)
                                {
                                    bd = d;
                                    best = lv.first;
                                }
                            }
                            if (best < 0 || (g.fixed && lvVert[best]->fixed()))
                                continue;
                            auto *f3 = new ped::EdgePedPlatformHeight<PoseV>();
                            f3->setVertex(0, vpose);
                            f3->setVertex(1, lvVert[best]);
                            f3->setMeasurement(P.platformHeightM);
                            f3->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.platformSigmaM * P.platformSigmaM));
                            auto *rk = new g2o::RobustKernelHuber();
                            rk->setDelta(kHuber1D * P.huberScale);
                            f3->setRobustKernel(rk);
                            opt.addEdge(f3);
                            Tag(G, PedGraph::PLAT, f3);
                        }
                }
            }
            if (DL().f)
            {
                Fnv hObs, hPose, hV, hE;
                for (const PedGraphKF &g : kfs)
                {
                    hObs.val(g.kf->mnId);
                    hObs.val(g.fixed);
                    hPose.val(g.kf->mnId);
                    const Sophus::SE3f Tcw = g.kf->GetPose();
                    hPose.bytes(Tcw.matrix().data(), 16 * sizeof(float));
                    if (g.kf->mpPedFrame)
                        for (const PedObs &o : g.kf->mpPedFrame->obs)
                        {
                            hObs.val(o.trackId); hObs.val(o.rootU); hObs.val(o.rootV); hObs.val(o.lpx); hObs.val(o.maskH);
                            hObs.val(o.hasFoot); hObs.val(o.footU); hObs.val(o.footV); hObs.val(o.footScore);
                        }
                }
                for (const auto &st : G.states)
                {
                    hV.val(st.v->id()); hV.val(st.fixed); hV.val(st.track);
                    const ped::Vec6 x = st.v->estimate();
                    hV.bytes(x.data(), 6 * sizeof(double));
                    if (DL().detail)
                        std::fprintf(DL().f, "V %d kf=%lu tr=%d fixed=%d %a %a %a %a %a %a\n", st.v->id(), st.kf->mnId, st.track, st.fixed ? 1 : 0, x(0), x(1), x(2), x(3), x(4), x(5));
                }
                for (int ty = 0; ty < PedGraph::NTYPES; ++ty)
                    for (g2o::OptimizableGraph::Edge *e : G.byType[ty])
                    {
                        hE.val(ty);
                        for (auto *hv : e->vertices())
                            hE.val(hv ? hv->id() : -1);
                        const int dim = e->dimension();
                        hE.bytes(e->informationData(), static_cast<std::size_t>(dim) * dim * sizeof(double));
                        if (DL().detail)
                        {
                            std::fprintf(DL().f, "E t=%d v=", ty);
                            for (auto *hv : e->vertices())
                                std::fprintf(DL().f, "%d,", hv ? hv->id() : -1);
                            std::fprintf(DL().f, " info0=%a\n", e->informationData()[0]);
                        }
                    }
                std::fprintf(DL().f, "BA %ld metric=%d nKF=%zu obs=%016llx pose=%016llx vtx=%016llx edges=%016llx nV=%zu nE=%zu\n", gDetCall++, metric ? 1 : 0, kfs.size(),
                             static_cast<unsigned long long>(hObs.h), static_cast<unsigned long long>(hPose.h), static_cast<unsigned long long>(hV.h),
                             static_cast<unsigned long long>(hE.h), G.states.size(), G.edges.size());
                std::fflush(DL().f);
            }
            gVertices += static_cast<long>(G.states.size()) + G.nFootVertices;
            gEdges += static_cast<long>(G.edges.size());
            return G;
        }
    } // namespace

    PedGraph PedestrianBA::Add(g2o::SparseOptimizer &opt, const std::vector<PedGraphKF> &kfs, Map *pMap, bool metric, bool inertialPoses,
                               unsigned long mpVertexBase)
    {
        if (!PedestrianParams::Enabled())
            throw std::runtime_error("[Pedestrians] PedestrianBA::Add called with Pedestrians.Enabled: 0");
        return inertialPoses ? AddImpl<VertexPose>(opt, kfs, pMap, metric, mpVertexBase) : AddImpl<g2o::VertexSE3Expmap>(opt, kfs, pMap, metric, mpVertexBase);
    }

    PedestrianBA::FloorEstimate PedestrianBA::RegionalFloor(const std::vector<Eigen::Vector3d> &pts, const Eigen::Vector2d &centre, double zCamera, double R,
                                                            double bin, double belowCamera, int minPoints, double lowModeFraction)
    {
        FloorEstimate out;
        std::vector<double> z;
        const double R2 = R * R;
        for (const Eigen::Vector3d &x : pts)
            if ((x.head<2>() - centre).squaredNorm() <= R2 && x(2) <= zCamera - belowCamera)
                z.push_back(x(2));
        out.nRegion = static_cast<int>(z.size());
        if (out.nRegion < minPoints || out.nRegion == 0)
            return out;
        const double zmin = *std::min_element(z.begin(), z.end());
        const int nb = static_cast<int>(std::floor((*std::max_element(z.begin(), z.end()) - zmin) / bin)) + 1;
        std::vector<int> cnt(nb, 0);
        for (double v : z)
            ++cnt[static_cast<int>(std::floor((v - zmin) / bin))];
        int best = -1, bestIdx = 0;
        for (int j = 0; j < nb; ++j)
        {
            const int c3 = cnt[j] + (j > 0 ? cnt[j - 1] : 0) + (j + 1 < nb ? cnt[j + 1] : 0);
            if (c3 > best) // strict: the lowest window wins ties
            {
                best = c3;
                bestIdx = j;
            }
        }
        if (lowModeFraction < 1.0)
        {
            std::vector<int> c3(nb);
            for (int j = 0; j < nb; ++j)
                c3[j] = cnt[j] + (j > 0 ? cnt[j - 1] : 0) + (j + 1 < nb ? cnt[j + 1] : 0);
            for (int j = 0; j < nb; ++j)
                if (c3[j] >= lowModeFraction * best && (j == 0 || c3[j] >= c3[j - 1]) && (j == nb - 1 || c3[j] > c3[j + 1]))
                {
                    bestIdx = j;
                    break;
                }
        }
        std::vector<double> win;
        for (double v : z)
        {
            const int j = static_cast<int>(std::floor((v - zmin) / bin));
            if (j >= bestIdx - 1 && j <= bestIdx + 1)
                win.push_back(v);
        }
        std::sort(win.begin(), win.end());
        out.zf = win[win.size() / 2];
        out.nMode = static_cast<int>(win.size());
        out.ok = true;
        return out;
    }

    namespace
    {
        struct CallRec
        {
            const char *kind;
            double stamp;
            long created[PedGraph::NTYPES], active[PedGraph::NTYPES];
        };
        std::vector<CallRec> gCalls;
        long gCreatedT[PedGraph::NTYPES] = {0}, gActiveT[PedGraph::NTYPES] = {0};
        const char *kTypeName[PedGraph::NTYPES] = {"root", "range", "wna", "foot", "t1", "t2", "floor", "bias", "lvpt", "plat"};
        // inside the robust-kernel threshold: the Huber deltas of PedestrianBA.cc (chi2 95 %: 5.991 for 2 dof, 3.841 for 1 dof);
        // the WNA prior (6 dof, no kernel) uses its own 95 % chi2 (12.592)
        const double kActiveChi2[PedGraph::NTYPES] = {5.991, 3.841, 12.592, 5.991, 3.841, 3.841, 5.991, 3.841, 3.841, 3.841};
    } // namespace

    void PedestrianBA::Account(const PedGraph &g, const char *kind, double stamp)
    {
        CallRec r;
        r.kind = kind;
        r.stamp = stamp;
        for (int ty = 0; ty < PedGraph::NTYPES; ++ty)
        {
            long act = 0;
            for (g2o::OptimizableGraph::Edge *e : g.byType[ty])
            {
                // P9-C4 I: an edge whose vertices are all fixed is not in g2o's active set, so no solve ever computed its _error; chi2() summed uninitialised memory
                // (valgrind: DumpCounters). Compute it from the current estimates.
                bool anyFree = false;
                for (auto *hv : e->vertices())
                    anyFree = anyFree || (hv && !static_cast<g2o::OptimizableGraph::Vertex *>(hv)->fixed());
                if (!anyFree)
                    e->computeError();
                if (e->chi2() <= kActiveChi2[ty])
                    ++act;
            }
            r.created[ty] = static_cast<long>(g.byType[ty].size());
            r.active[ty] = act;
            gCreatedT[ty] += r.created[ty];
            gActiveT[ty] += act;
        }
        std::lock_guard<std::mutex> lk(gLogMutex);
        gCalls.push_back(r);
    }

    void PedestrianBA::DumpCounters(const std::string &path)
    {
        std::ofstream f(path);
        f << "call,kind,stamp";
        for (int ty = 0; ty < PedGraph::NTYPES; ++ty)
            f << "," << kTypeName[ty] << "_created," << kTypeName[ty] << "_active";
        f << "\n";
        std::lock_guard<std::mutex> lk(gLogMutex);
        long n = 0;
        for (const CallRec &r : gCalls)
        {
            f << n++ << "," << r.kind << "," << std::fixed << r.stamp << std::defaultfloat;
            for (int ty = 0; ty < PedGraph::NTYPES; ++ty)
                f << "," << r.created[ty] << "," << r.active[ty];
            f << "\n";
        }
        f << "total,-,0";
        for (int ty = 0; ty < PedGraph::NTYPES; ++ty)
            f << "," << gCreatedT[ty] << "," << gActiveT[ty];
        f << "\n";
    }

    void PedestrianBA::Counters(long &vertices, long &edges)
    {
        vertices = gVertices;
        edges = gEdges;
    }

    void PedestrianBA::DumpLog(const std::string &path)
    {
        std::ofstream f(path);
        f << "# kf_id stamp track px py pz vx vy vz tx ty tz qx qy qz qw (world positions/velocities; Twc of the keyframe at the write)\n";
        char buf[512];
        std::lock_guard<std::mutex> lk(gLogMutex);
        for (const auto &kv : gLog)
        {
            const LogRec &r = kv.second;
            std::snprintf(buf, sizeof(buf), "%lu %.9f %d %a %a %a %a %a %a %a %a %a %a %a %a %a\n", kv.first.first, r.stamp, kv.first.second, r.x(0), r.x(1),
                          r.x(2), r.x(3), r.x(4), r.x(5), r.twc(0), r.twc(1), r.twc(2), r.twc(3), r.twc(4), r.twc(5), r.twc(6));
            f << buf;
        }
    }

    void PedestrianBA::DumpGateLog(const std::string &path)
    {
        std::ofstream f(path);
        f << "# P9-C4 innovation gate, one decision per (keyframe, track): status 1 accepted / 2 rejected / 3 re-initialised; tested 0 = accepted without a test; no-predecessor passes after the free keyframes: " << gGateNoPred << "\n";
        f << "# kf_id stamp track status tested dof d2 dt pred_stamp chain_idx consec_rej\n";
        char buf[256];
        std::lock_guard<std::mutex> lk(gLogMutex);
        for (const auto &kv : gGateLog)
        {
            const PedGateRec &r = kv.second.rec;
            std::snprintf(buf, sizeof(buf), "%lu %.9f %d %d %d %d %a %a %.9f %d %d\n", kv.first.first, kv.second.stamp, kv.first.second, r.status, r.tested ? 1 : 0, r.dof, r.d2, r.dt,
                          r.predStamp, r.chainIdx, r.consecRej);
            f << buf;
        }
    }

    void PedestrianBA::DumpFloorPxLog(const std::string &path)
    {
        std::ofstream f(path);
        f << "# P9-C3c midpoint contacts seen by the gates. stage 0 pass / 1 no regional z_f / 2 raised surface / 3 elevation / 4 too few qualifying points / 5 plane scatter\n";
        f << "# kf_id stamp track stage zf_raw zf_glob elev_deg u v nRegion nQual sigQual res_u res_v\n";
        char buf[384];
        std::lock_guard<std::mutex> lk(gLogMutex);
        for (const auto &kv : gPxLog)
        {
            const PxRec &r = kv.second;
            std::snprintf(buf, sizeof(buf), "%lu %.9f %d %d %a %a %a %a %a %d %d %a %a %a\n", kv.first.first, r.stamp, kv.first.second, r.stage, r.zfRaw, r.zfGlob, r.elev, r.u, r.v, r.nRegion, r.nQual,
                          r.sigQual, r.resU, r.resV);
            f << buf;
        }
    }

    void PedestrianBA::DumpLevelsLog(const std::string &path)
    {
        std::ofstream f(path);
        f << "# P9-C5 contacts. map_id kf_id stamp track hc level_id level_z transitional n_levels\n";
        f << "# L map_id level_id z n_contacts n_f2_points_max n_merges_map\n";
        f << "# P map_id point_id x y z dist_to_nearest_contact kf_of_that_contact track_of_that_contact (levels_shadow only)\n";
        f << "# B map_id kf_id track b  (the track's log-scale bias at the last write of that state; levels / levels_platform)\n";
        char buf[384];
        std::lock_guard<std::mutex> lk(gLogMutex);
        for (const auto &kv : gLevelLog)
        {
            const LevelRow &r = kv.second;
            std::snprintf(buf, sizeof(buf), "%lu %lu %.9f %d %a %d %a %d %d\n", std::get<0>(kv.first), std::get<1>(kv.first), r.stamp, std::get<2>(kv.first), r.hc, r.level, r.levelZ,
                          r.transitional, r.nLevels);
            f << buf;
        }
        for (const auto &kv : gPtLog)
        {
            std::snprintf(buf, sizeof(buf), "P %lu %lu %a %a %a %a %lu %d\n", kv.first.first, kv.first.second, kv.second.x, kv.second.y, kv.second.z, kv.second.d, kv.second.kf, kv.second.track);
            f << buf;
        }
        for (const auto &kv : gBiasRow)
        {
            std::snprintf(buf, sizeof(buf), "B %lu %lu %d %a\n", std::get<0>(kv.first), std::get<1>(kv.first), std::get<2>(kv.first), kv.second);
            f << buf;
        }
        for (const auto &kv : gLevelStat)
        {
            std::snprintf(buf, sizeof(buf), "L %lu %d %a %d %d %d\n", kv.first.first, kv.first.second, kv.second.z, kv.second.nContacts, kv.second.nPointsMax, kv.second.merges);
            f << buf;
        }
    }

    void PedestrianBA::DumpFloorLog(const std::string &path)
    {
        std::ofstream f(path);
        f << "# skipped (no regional z_f, no neighbours, grazing ray): " << gFloorSkipped[1] << " " << gFloorSkipped[2] << " " << gFloorSkipped[3] << "\n";
        f << "# kf_id stamp track zp z_regional Gx Gy Gz px py pz s nPts b (map frame; b = the track's log-scale bias at the write; G = foot-ray / plane intersection, p = optimised root, s = ray depth)\n";
        char buf[512];
        std::lock_guard<std::mutex> lk(gLogMutex);
        for (const auto &kv : gFloorLog)
        {
            const FloorRec &r = kv.second;
            std::snprintf(buf, sizeof(buf), "%lu %.9f %d %a %a %a %a %a %a %a %a %a %d %a\n", kv.first.first, r.stamp, kv.first.second, r.zp, r.zRegional, r.G(0), r.G(1),
                          r.G(2), r.p(0), r.p(1), r.p(2), r.s, r.nPts, r.b);
            f << buf;
        }
    }

    void PedestrianBA::WriteBack(const PedGraph &g, Map *pMap)
    {
        for (const auto &bh : g.biases)
        {
            pMap->SetPedBias(bh.track, bh.v->estimate());
            for (const auto &s : g.states)
                if (s.track == bh.track && !s.fixed)
                {
                    std::lock_guard<std::mutex> lk(gLogMutex);
                    gBiasRow[{pMap->GetId(), s.kf->mnId, s.track}] = bh.v->estimate();
                }
        }
        if (!g.levels.empty())
        {
            std::lock_guard<std::mutex> lvLock(pMap->PedLevelsMutex());
            const ped::LevelParams LP = ped::DesignLevelParams(PedestrianParams::Get().levelSigmaA, PedestrianParams::Get().levelSigmaPt, PedestrianParams::Get().levelSMax);
            ped::LevelSet &LS = pMap->PedLevels(LP);
            for (const auto &lh : g.levels)
            {
                LS.SetHeight(lh.id, lh.v->estimate());
                std::lock_guard<std::mutex> lk(gLogMutex);
                gLevelStat[{pMap->GetId(), lh.id}].z = lh.v->estimate();
            }
        }
        for (const auto &h : g.pxFloors)
        {
            h.e->computeError(); // residual of the contact pixel at the solution
            std::lock_guard<std::mutex> lk(gLogMutex);
            PxRec &r = gPxLog[{h.kf, h.track}];
            r.resU = h.e->errorData()[0];
            r.resV = h.e->errorData()[1];
        }
        for (const auto &h : g.floors)
        {
            FloorRec r;
            auto fill = [&](auto *e) {
                const auto geo = e->Geometry();
                r.zp = geo.zp;
                r.s = geo.s;
                r.G = geo.G;
                r.p = static_cast<const ped::VertexPedState *>(e->vertex(0))->p();
                r.nPts = e->nPts;
            };
            if (h.imu)
                fill(static_cast<ped::EdgePedFloorContact<VertexPose> *>(h.e));
            else
                fill(static_cast<ped::EdgePedFloorContact<g2o::VertexSE3Expmap> *>(h.e));
            r.stamp = h.kf->mTimeStamp;
            r.zRegional = h.zRegional;
            r.b = 0.0;
            for (const auto &bh : g.biases)
                if (bh.track == h.track)
                    r.b = bh.v->estimate();
            std::lock_guard<std::mutex> lk(gLogMutex);
            gFloorLog[{h.kf->mnId, h.track}] = r;
        }
        for (const auto &s : g.states)
        {
            if (s.fixed)
                continue;
            PedState st;
            st.p = s.v->estimate().head<3>();
            st.v = s.v->estimate().tail<3>();
            s.kf->SetPedState(s.track, st);
            const Sophus::SE3f Twc = s.kf->GetPoseInverse();
            const Eigen::Quaternionf q = Twc.unit_quaternion();
            LogRec r;
            r.stamp = s.kf->mTimeStamp;
            r.x << st.p, st.v;
            r.twc << Twc.translation().cast<double>(), q.x(), q.y(), q.z(), q.w();
            std::lock_guard<std::mutex> lk(gLogMutex);
            gLog[{s.kf->mnId, s.track}] = r;
        }
        if (DL().f)
        {
            Fnv h;
            for (const auto &s : g.states)
            {
                const ped::Vec6 x = s.v->estimate();
                h.val(s.v->id());
                h.bytes(x.data(), 6 * sizeof(double));
            }
            std::fprintf(DL().f, "WB %ld post=%016llx nStates=%zu\n", gDetCall - 1, static_cast<unsigned long long>(h.h), g.states.size());
            std::fflush(DL().f);
        }
    }
} // namespace ORB_SLAM3
