#include "PedestrianBA.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <type_traits>

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
            double stamp, zp, zRegional, s;
            Eigen::Vector3d G, p;
            int nPts;
        };
        std::map<std::pair<unsigned long, int>, FloorRec> gFloorLog;
        long gFloorSkipped[4] = {0, 0, 0, 0}; // no foot observation (not counted), no regional z_f, no neighbours, grazing ray
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
                    if (P.floorMode == PedestrianParams::FloorMode::Joint)
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

                // 2. state vertices (existing estimate, else initialised from the keyframe's pose, the root pixel and the range)
                for (Entry &e : E)
                {
                    const Cam c = CameraOf(e.kf);
                    PedState s;
                    if (!e.kf->GetPedState(track, s))
                    {
                        const Eigen::Vector3d b((e.obs->rootU - c.cx) / c.fx, (e.obs->rootV - c.cy) / c.fy, 1.0);
                        // depth along the optical axis: range r = f cL H / l_px along the unit ray, z = r / |b~|;
                        // without a skeleton length the mask height gives an initial depth only (the estimate is refined by the BA)
                        const double z = e.rangeOk ? (c.fx * P.skelOverStature * Hcur / e.obs->lpx) / b.norm()
                                                   : (c.fx * P.maskHeightM / std::max(e.obs->maskH, 1.0)) / b.norm();
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
                        auto *edge = new ped::EdgePedLogRange<PoseV>();
                        edge->setVertex(0, e.v);
                        edge->setVertex(1, vpose);
                        edge->setMeasurement(std::log(c.fx) + std::log(P.skelOverStature) + std::log(P.statureFixedM) - std::log(e.obs->lpx));
                        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.logDepthSigma * P.logDepthSigma));
                        auto *rk = new g2o::RobustKernelHuber();
                        rk->setDelta(kHuber1D * P.huberScale);
                        edge->setRobustKernel(rk);
                        opt.addEdge(edge);
                        Tag(G, PedGraph::RANGE, edge);
                        ++G.nRange;
                    }
                    if (floorOn && e.obs->hasFoot && !e.fixed)
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
                    const double dt = E[i + 1].kf->mTimeStamp - E[i].kf->mTimeStamp;
                    if (!(dt > 1e-6))
                        continue;
                    auto *edge = new ped::EdgePedWna();
                    edge->setVertex(0, E[i].v);
                    edge->setVertex(1, E[i + 1].v);
                    edge->setMeasurement(ped::Vec6::Zero());
                    edge->SetDt(dt, P.qv);
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
        const char *kTypeName[PedGraph::NTYPES] = {"root", "range", "wna", "foot", "t1", "t2", "floor"};
        // inside the robust-kernel threshold: the Huber deltas of PedestrianBA.cc (chi2 95 %: 5.991 for 2 dof, 3.841 for 1 dof);
        // the WNA prior (6 dof, no kernel) uses its own 95 % chi2 (12.592)
        const double kActiveChi2[PedGraph::NTYPES] = {5.991, 3.841, 12.592, 5.991, 3.841, 3.841, 5.991};
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
                if (e->chi2() <= kActiveChi2[ty])
                    ++act;
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

    void PedestrianBA::DumpFloorLog(const std::string &path)
    {
        std::ofstream f(path);
        f << "# skipped (no regional z_f, no neighbours, grazing ray): " << gFloorSkipped[1] << " " << gFloorSkipped[2] << " " << gFloorSkipped[3] << "\n";
        f << "# kf_id stamp track zp z_regional Gx Gy Gz px py pz s nPts (map frame; G = foot-ray / plane intersection, p = optimised root, s = ray depth)\n";
        char buf[512];
        std::lock_guard<std::mutex> lk(gLogMutex);
        for (const auto &kv : gFloorLog)
        {
            const FloorRec &r = kv.second;
            std::snprintf(buf, sizeof(buf), "%lu %.9f %d %a %a %a %a %a %a %a %a %a %d\n", kv.first.first, r.stamp, kv.first.second, r.zp, r.zRegional, r.G(0), r.G(1),
                          r.G(2), r.p(0), r.p(1), r.p(2), r.s, r.nPts);
            f << buf;
        }
    }

    void PedestrianBA::WriteBack(const PedGraph &g, Map *pMap)
    {
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
    }
} // namespace ORB_SLAM3
