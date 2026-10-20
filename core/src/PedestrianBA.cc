#include "PedestrianBA.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iostream>
#include <fstream>
#include <mutex>
#include <stdexcept>

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
        };

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
        PedGraph AddImpl(g2o::SparseOptimizer &opt, const std::vector<PedGraphKF> &kfsIn, Map *pMap, bool metric)
        {
            const PedestrianParams &P = PedestrianParams::Get();
            PedGraph G;

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
                    if (o.lpx <= 0)
                        continue;
                    tracks[o.trackId].push_back({g.kf, g.fixed, &o, nullptr});
                }
            }

            int nextId = 0; // above every id in the graph (the container is unordered; the maximum does not depend on its order)
            for (const auto &kv : opt.vertices())
                nextId = std::max(nextId, kv.first + 1);

            for (auto &tr : tracks)
            {
                std::vector<Entry> &E = tr.second;
                const bool anyFree = std::any_of(E.begin(), E.end(), [](const Entry &e) { return !e.fixed; });
                if (static_cast<int>(E.size()) < P.minTrackKeyframes || !anyFree)
                {
                    ++G.nSkippedTracks;
                    continue;
                }
                ++G.nTracks;
                const int track = tr.first;

                // stature (metric factors only)
                ped::VertexStature *VH = nullptr;
                double Hcur = P.statureMean;
                if (metric)
                {
                    double H;
                    if (pMap->GetPedStature(track, H))
                        Hcur = H;
                    VH = new ped::VertexStature();
                    VH->setId(nextId++);
                    VH->setEstimate(Hcur);
                    VH->setFixed(false);
                    opt.addVertex(VH);
                    G.statures.push_back({track, VH});
                    auto *prior = new ped::EdgeStaturePrior();
                    prior->setVertex(0, VH);
                    prior->setMeasurement(P.statureMean);
                    prior->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.statureSigma * P.statureSigma));
                    opt.addEdge(prior);
                    G.edges.push_back(prior);
                }

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
                        const double z = e.obs->lpx > 0 ? (c.fx * P.skelOverStature * Hcur / e.obs->lpx) / b.norm()
                                                        : c.fx * Hcur / std::max(e.obs->maskH, 1.0);
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
                            rk->setDelta(kHuber2D);
                            edge->setRobustKernel(rk);
                            opt.addEdge(edge);
                            G.edges.push_back(edge);
                            ++G.nRoot;
                        }
                    }
                    if (metric && e.obs->lpx > 0)
                    {
                        auto *edge = new ped::EdgePedLogRange<PoseV>();
                        edge->setVertex(0, e.v);
                        edge->setVertex(1, vpose);
                        edge->setVertex(2, VH);
                        edge->setMeasurement(std::log(c.fx) + std::log(P.skelOverStature) - std::log(e.obs->lpx));
                        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.logDepthSigma * P.logDepthSigma));
                        auto *rk = new g2o::RobustKernelHuber();
                        rk->setDelta(kHuber1D);
                        edge->setRobustKernel(rk);
                        opt.addEdge(edge);
                        G.edges.push_back(edge);
                        ++G.nRange;
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
                    G.edges.push_back(edge);
                    ++G.nWna;
                }

                // 5. stance: footprint vertex per run of consecutive claimed, sufficiently tall entries of one foot
                for (int foot = 0; foot < 2; ++foot)
                {
                    size_t i = 0;
                    while (i < E.size())
                    {
                        auto ok = [&](const Entry &e) { return e.obs->claim[foot] && e.obs->boxH >= P.minStanceBoxPx; };
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
                            const Eigen::Vector3d b((e0.obs->toeU[foot] - c.cx) / c.fx, (e0.obs->toeV[foot] - c.cy) / c.fy, 1.0);
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
                                re->setMeasurement(Eigen::Vector2d(e.obs->toeU[foot], e.obs->toeV[foot]));
                                re->setInformation(Eigen::Matrix2d::Identity() / (P.footPxSigma * P.footPxSigma));
                                auto *rk = new g2o::RobustKernelHuber();
                                rk->setDelta(kHuber2D);
                                re->setRobustKernel(rk);
                                opt.addEdge(re);
                                G.edges.push_back(re);
                                ++G.nFoot;

                                auto *t1 = new ped::EdgePedTie1(P.cRoot);
                                t1->setVertex(0, e.v);
                                t1->setVertex(1, VX);
                                t1->setVertex(2, VH);
                                t1->setMeasurement(0.0);
                                t1->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.t1Sigma * P.t1Sigma));
                                auto *rk1 = new g2o::RobustKernelHuber();
                                rk1->setDelta(kHuber1D);
                                t1->setRobustKernel(rk1);
                                opt.addEdge(t1);
                                G.edges.push_back(t1);

                                auto *t2 = new ped::EdgePedTie2(P.cLeg);
                                t2->setVertex(0, e.v);
                                t2->setVertex(1, VX);
                                t2->setVertex(2, VH);
                                t2->setMeasurement(0.0);
                                t2->setInformation(Eigen::Matrix<double, 1, 1>::Identity() / (P.t2Sigma * P.t2Sigma));
                                auto *rk2 = new g2o::RobustKernelHuber();
                                rk2->setDelta(kHuber1D);
                                t2->setRobustKernel(rk2);
                                opt.addEdge(t2);
                                G.edges.push_back(t2);
                                G.nTie += 2;
                            }
                        }
                        i = j;
                    }
                }
            }
            gVertices += static_cast<long>(G.states.size() + G.statures.size()) + G.nFootVertices;
            gEdges += static_cast<long>(G.edges.size());
            return G;
        }
    } // namespace

    PedGraph PedestrianBA::Add(g2o::SparseOptimizer &opt, const std::vector<PedGraphKF> &kfs, Map *pMap, bool metric, bool inertialPoses)
    {
        if (!PedestrianParams::Enabled())
            throw std::runtime_error("[Pedestrians] PedestrianBA::Add called with Pedestrians.Enabled: 0");
        return inertialPoses ? AddImpl<VertexPose>(opt, kfs, pMap, metric) : AddImpl<g2o::VertexSE3Expmap>(opt, kfs, pMap, metric);
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

    void PedestrianBA::WriteBack(const PedGraph &g, Map *pMap)
    {
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
        for (const auto &h : g.statures)
            pMap->SetPedStature(h.first, h.second->estimate());
    }
} // namespace ORB_SLAM3
