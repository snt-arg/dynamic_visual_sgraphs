// P9-C5: floor-level bookkeeping (association, creation, merging, transitional segments, RANSAC point selection). Header only, no g2o and no
// ORB-SLAM3 dependency, so it is unit-tested standalone (core/test/test_levels.cc). The g2o vertices and factors are in PedestrianFactors.h.
//
// Every threshold is a field of LevelParams and has NO default: the caller (settings reader) must provide it, or fail (fail-fast rule of P9-C5).
// Design constants declared in task_C5.md: creation needs >= 10 contacts from >= 2 tracks agreeing within +-2 sigma_a, association < 3 sigma_a,
// merge < 2 sigma_a, transitional window 1.5 s with >= 4 contacts and >= 0.5 m travel, RANSAC threshold 2 sigma_pt.
// Calibrated (constants_c5.md): sigma_a, sigma_pt, s_max.
// P9-L Part 4 (p9l_part4.md step 2.4): the dh/dt transitional rule of P9-L Part 2 step 3 is REVERTED to this path-length slope rule (no rate met the fixed criterion on the recalibrated
// synthetic harness); the foot gate, epoch guard, founding log and the poster detector below are P9-L additions.

#ifndef PEDESTRIAN_LEVELS_H
#define PEDESTRIAN_LEVELS_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ORB_SLAM3
{
    namespace ped
    {
        struct LevelParams
        {
            double sigmaA, sigmaPt, sMax;              // calibrated
            double assocK, createK, mergeK;            // 3, 2, 2 (multiples of sigmaA)
            int createMinContacts, createMinTracks;    // 10, 2
            double createMinExtentM;                   // 1.0: founding contacts must span >= this horizontally (c5_gate_revision.md, design change 2)
            double edgeGuardS;                         // 1.5 (= transWindowS): a contact this close in time to a transitional contact of its track never enters the pool (design change 1)
            double transWindowS, transMinTravelM;      // 1.5, 0.5
            int transMinContacts;                      // 4
            double ransacThrK, ransacRadiusM;          // 2 (x sigmaPt), 1.0
            int ransacMinInliers;                      // 5
            double ransacMaxOffsetK;                   // 3 (x sigmaA): consensus height must lie this close to z_l
            int ransacIters;                           // RansacIterations(0.99, 0.5)
        };

        // The design constants of task_C5.md / patch_stairs_ransac.md; only the three calibrated / set values are inputs.
        inline LevelParams DesignLevelParams(double sigmaA, double sigmaPt, double sMax)
        {
            LevelParams P;
            P.sigmaA = sigmaA;
            P.sigmaPt = sigmaPt;
            P.sMax = sMax;
            P.assocK = 3;
            P.createK = 2;
            P.mergeK = 2;
            P.createMinContacts = 10;
            P.createMinTracks = 2;
            P.createMinExtentM = 1.0;
            P.edgeGuardS = 1.5;
            P.transWindowS = 1.5;
            P.transMinTravelM = 0.5;
            P.transMinContacts = 4;
            P.ransacThrK = 2;
            P.ransacRadiusM = 1.0;
            P.ransacMinInliers = 5;
            P.ransacMaxOffsetK = 3;
            P.ransacIters = 7; // ceil(ln(0.01) / ln(0.5)), asserted against RansacIterations(0.99, 0.5) in the unit test
            return P;
        }

        // Number of iterations for success probability `conf` at inlier ratio `w` with a model of `m` samples: ln(1 - conf) / ln(1 - w^m).
        // The model is a 1-D height, m = 1: conf 0.99, w 0.5 -> 6.64 -> 7.
        inline int RansacIterations(double conf, double w, int m = 1)
        {
            return static_cast<int>(std::ceil(std::log(1.0 - conf) / std::log(1.0 - std::pow(w, m))));
        }

        struct Contact
        {
            int track;
            unsigned long kf = 0; // keyframe id: (kf, track) identifies a contact across overlapping BA windows
            unsigned long frame = 0; // P9-H A2.3: ORB-SLAM3 frame id of that keyframe (logging only)
            double t;           // seconds
            double x, y;        // person footprint, world (horizontal)
            double h;           // implied floor height h_c
            double footU = 0, footV = 0; // P9-L: the contact (lowest foot keypoint) pixel; only logged (ped_levels_found.log)
            bool transitional = false;
            int level = -1;     // level id or -1
        };

        // ------------------------------------------------------------------------------------------- transitional segments (patch B)
        // Marks, per track, the contacts of every window (<= transWindowS long, >= transMinContacts contacts, >= transMinTravelM walked) whose regression
        // slope of h_c on the walked horizontal distance exceeds sMax. Contacts of one track must be time-ordered. Returns the number flagged.
        inline int FlagTransitional(std::vector<Contact> &cs, const LevelParams &P)
        {
            std::map<int, std::vector<int>> byTrack;
            for (int i = 0; i < static_cast<int>(cs.size()); ++i)
                byTrack[cs[i].track].push_back(i);
            int n = 0;
            for (auto &kv : byTrack)
            {
                const std::vector<int> &ix = kv.second;
                const int m = static_cast<int>(ix.size());
                std::vector<double> s(m, 0.0); // cumulative walked distance
                for (int a = 1; a < m; ++a)
                {
                    if (cs[ix[a]].t < cs[ix[a - 1]].t)
                        throw std::runtime_error("[Levels] FlagTransitional: contacts of a track are not time-ordered");
                    s[a] = s[a - 1] + std::hypot(cs[ix[a]].x - cs[ix[a - 1]].x, cs[ix[a]].y - cs[ix[a - 1]].y);
                }
                for (int b = 0; b < m; ++b) // window ends at contact b
                {
                    int a = b;
                    while (a > 0 && cs[ix[b]].t - cs[ix[a - 1]].t <= P.transWindowS)
                        --a;
                    const int cnt = b - a + 1;
                    if (cnt < P.transMinContacts || s[b] - s[a] < P.transMinTravelM)
                        continue;
                    double ms = 0, mh = 0;
                    for (int k = a; k <= b; ++k)
                    {
                        ms += s[k];
                        mh += cs[ix[k]].h;
                    }
                    ms /= cnt;
                    mh /= cnt;
                    double sxx = 0, sxy = 0;
                    for (int k = a; k <= b; ++k)
                    {
                        sxx += (s[k] - ms) * (s[k] - ms);
                        sxy += (s[k] - ms) * (cs[ix[k]].h - mh);
                    }
                    if (sxx <= 0)
                        continue;
                    if (std::abs(sxy / sxx) > P.sMax)
                        for (int k = a; k <= b; ++k)
                            cs[ix[k]].transitional = true;
                }
            }
            for (const Contact &c : cs)
                n += c.transitional ? 1 : 0;
            return n;
        }

        // ------------------------------------------------------------------------------------------- levels
        struct Level
        {
            int id;
            double z;
            int nContacts = 0;
            std::vector<Contact> founding; // P9-L step 5: the contacts that created the level (ped_levels_found.log)
        };

        // P9-H A2.3: logging only (never read by the estimator). CREATE: id = new level, z / n = its founding median and contact count. MERGE: id = the surviving (lower) id, otherId = the absorbed one;
        // z / n and otherZ / otherN are the two levels' heights and contact counts BEFORE the merge.
        struct LevelEvent
        {
            bool merge = false;
            int id = -1, otherId = -1;
            double z = 0, otherZ = 0;
            int n = 0, otherN = 0;
        };

        class LevelSet
        {
        public:
            explicit LevelSet(const LevelParams &p) : P(p), nextId(0) {}
            std::vector<LevelEvent> TakeEvents()
            {
                std::vector<LevelEvent> out;
                out.swap(events);
                return out;
            }
            const std::vector<Level> &Levels() const { return levels; }
            int Merges() const { return merges; }

            // Association: nearest level with |h - z| < assocK sigmaA, else -1. Transitional contacts are never associated.
            int Associate(const Contact &c) const
            {
                if (c.transitional)
                    return -1;
                int best = -1;
                double bd = P.assocK * P.sigmaA;
                for (const Level &L : levels)
                {
                    const double d = std::abs(c.h - L.z);
                    if (d < bd)
                    {
                        bd = d;
                        best = L.id;
                    }
                }
                return best;
            }

            // One update: associate, pool the unassigned (non-transitional) contacts, create from the pool, merge. `pool` persists between calls (per map).
            // Contacts are assigned `level` in place. Returns the number of levels created.
            int Update(std::vector<Contact> &cs)
            {
                // edge guard (c5_gate_revision.md, design change 1): times of the transitional contacts of each track in this window
                std::map<int, std::vector<double>> transT;
                for (const Contact &c : cs)
                    if (c.transitional)
                        transT[c.track].push_back(c.t);
                auto guarded = [&](const Contact &c) {
                    auto it = transT.find(c.track);
                    if (it == transT.end())
                        return false;
                    for (double t : it->second)
                        if (std::abs(t - c.t) <= P.edgeGuardS)
                            return true;
                    return false;
                };
                // flags are retrospective (a window is complete only when its last contact arrives): a pooled contact that the current window now places within one
                // guard time of a transitional contact of its track leaves the pool
                {
                    std::vector<Contact> keep;
                    for (const Contact &c : pool)
                        if (!guarded(c))
                            keep.push_back(c);
                    pool.swap(keep);
                }
                for (Contact &c : cs)
                {
                    c.level = Associate(c);
                    const bool fresh = seen.insert({c.kf, c.track}).second; // a contact is counted / pooled once, however many BA windows contain it
                    if (c.level >= 0)
                    {
                        if (fresh)
                            ++Find(c.level).nContacts;
                    }
                    else if (!c.transitional && fresh && !guarded(c))
                        pool.push_back(c);
                }
                int created = 0;
                while (CreateFromPool())
                    ++created;
                MergeClose();
                for (Contact &c : cs) // contacts of merged levels and newly created ones
                    if (!c.transitional)
                        c.level = Associate(c);
                return created;
            }

            // Pool-based creation: candidate clusters = the +-createK sigmaA neighbourhoods of the pooled heights, tried in order of decreasing size (ties: lower height).
            // A cluster creates a level when it has >= createMinContacts contacts from >= createMinTracks tracks AND the contacts span >= createMinExtentM horizontally
            // (largest pairwise distance; a stair tread is 0.28 m deep, c5_gate_revision.md design change 2). Initial z is the median; the cluster's contacts are consumed.
            bool CreateFromPool()
            {
                if (static_cast<int>(pool.size()) < P.createMinContacts)
                    return false;
                std::vector<double> hs;
                for (const Contact &c : pool)
                    hs.push_back(c.h);
                std::sort(hs.begin(), hs.end());
                const double w = P.createK * P.sigmaA;
                std::vector<std::pair<int, double>> cand; // (-count, centre)
                for (size_t i = 0; i < hs.size(); ++i)
                {
                    if (i > 0 && hs[i] == hs[i - 1])
                        continue;
                    int n = 0;
                    for (double h : hs)
                        n += std::abs(h - hs[i]) <= w ? 1 : 0;
                    cand.push_back({-n, hs[i]});
                }
                std::sort(cand.begin(), cand.end());
                for (const auto &cd : cand)
                {
                    if (-cd.first < P.createMinContacts)
                        break; // sorted: no later candidate is larger
                    std::vector<const Contact *> in;
                    std::set<int> tracks;
                    for (const Contact &c : pool)
                        if (std::abs(c.h - cd.second) <= w)
                        {
                            in.push_back(&c);
                            tracks.insert(c.track);
                        }
                    if (static_cast<int>(tracks.size()) < P.createMinTracks)
                        continue;
                    double ext = 0;
                    for (size_t a = 0; a < in.size() && ext < P.createMinExtentM; ++a)
                        for (size_t b = a + 1; b < in.size(); ++b)
                            ext = std::max(ext, std::hypot(in[a]->x - in[b]->x, in[a]->y - in[b]->y));
                    if (ext < P.createMinExtentM)
                        continue;
                    std::vector<double> hh;
                    for (const Contact *c : in)
                        hh.push_back(c->h);
                    std::sort(hh.begin(), hh.end());
                    const double med = hh.size() % 2 ? hh[hh.size() / 2] : 0.5 * (hh[hh.size() / 2 - 1] + hh[hh.size() / 2]);
                    Level L;
                    L.id = nextId++;
                    L.z = med;
                    L.nContacts = static_cast<int>(hh.size());
                    for (const Contact *c : in)
                        L.founding.push_back(*c);
                    levels.push_back(L);
                    {
                        LevelEvent ev;
                        ev.id = L.id;
                        ev.z = L.z;
                        ev.n = L.nContacts;
                        events.push_back(ev);
                    }
                    std::vector<Contact> rest;
                    for (const Contact &c : pool)
                        if (std::abs(c.h - med) > w)
                            rest.push_back(c);
                    pool.swap(rest);
                    return true;
                }
                return false;
            }

            // Merge pairs closer than mergeK sigmaA by the contact-count weighted mean; the lower id survives.
            void MergeClose()
            {
                bool again = true;
                while (again)
                {
                    again = false;
                    for (size_t i = 0; i < levels.size() && !again; ++i)
                        for (size_t j = i + 1; j < levels.size() && !again; ++j)
                            if (std::abs(levels[i].z - levels[j].z) < P.mergeK * P.sigmaA)
                            {
                                {
                                    LevelEvent ev;
                                    ev.merge = true;
                                    ev.id = levels[i].id;
                                    ev.otherId = levels[j].id;
                                    ev.z = levels[i].z;
                                    ev.otherZ = levels[j].z;
                                    ev.n = levels[i].nContacts;
                                    ev.otherN = levels[j].nContacts;
                                    events.push_back(ev);
                                }
                                const double wi = std::max(1, levels[i].nContacts), wj = std::max(1, levels[j].nContacts); // int -> double below
                                levels[i].z = (wi * levels[i].z + wj * levels[j].z) / (wi + wj);
                                levels[i].nContacts += levels[j].nContacts;
                                levels[i].founding.insert(levels[i].founding.end(), levels[j].founding.begin(), levels[j].founding.end());
                                levels.erase(levels.begin() + j);
                                ++merges;
                                again = true;
                            }
                }
            }

            void SetHeight(int id, double z) { Find(id).z = z; }
            bool Has(int id) const
            {
                for (const Level &L : levels)
                    if (L.id == id)
                        return true;
                return false;
            }
            void Clear()
            {
                levels.clear();
                pool.clear();
                seen.clear();
            }
            size_t PoolSize() const { return pool.size(); }

        private:
            Level &Find(int id)
            {
                for (Level &L : levels)
                    if (L.id == id)
                        return L;
                throw std::runtime_error("[Levels] unknown level id");
            }
            LevelParams P;
            std::set<std::pair<unsigned long, int>> seen;
            std::vector<Level> levels;
            std::vector<Contact> pool;
            int nextId;
            int merges = 0;
            std::vector<LevelEvent> events; // P9-H A2.3, logging only
        };

        // ------------------------------------------------------------------------------------------- P9-L step 4: epoch guard
        // The level set of a map is destroyed on map reset, merge, scaled rotation and loop correction (Map::ResetPedestriansLocked) and rebuilt with ids from 0. A BA that read
        // the set in Add and writes the solved heights back later (WriteBack) must not write into a rebuilt set: its ids belong to the old epoch. The store counts resets; Add
        // records the epoch it read, WriteBack skips the update (and counts it) when the epoch has changed. Callers hold the map's level mutex around every call.
        class LevelStore
        {
        public:
            LevelSet &Get(const LevelParams &p)
            {
                if (!set)
                    set.reset(new LevelSet(p));
                return *set;
            }
            void Reset()
            {
                set.reset();
                ++epoch;
            }
            unsigned long Epoch() const { return epoch; }

        private:
            std::unique_ptr<LevelSet> set;
            unsigned long epoch = 0;
        };

        // Writes the solved heights (level id, z) of a BA that read the store at `recordedEpoch`. Returns false (and counts, if `skipped`) when the epoch has changed since.
        inline bool ApplyLevelHeights(LevelStore &store, unsigned long recordedEpoch, const std::vector<std::pair<int, double>> &heights, const LevelParams &P, long *skipped = nullptr)
        {
            if (store.Epoch() != recordedEpoch)
            {
                if (skipped)
                    ++*skipped;
                return false;
            }
            LevelSet &LS = store.Get(P);
            for (const auto &h : heights)
            {
                if (!LS.Has(h.first)) // merged away by another BA's Update since Add: same epoch, but the id is gone
                {
                    if (skipped)
                        ++*skipped;
                    continue;
                }
                LS.SetHeight(h.first, h.second);
            }
            return true;
        }

        // ------------------------------------------------------------------------------------------- P9-L step 2: foot-validity gate
        // A foot contact is rejected when its pixel lies more than tau * maskH above the expected mask bottom (root_v + 0.55 maskH; the root is defined as (cu, vb - 0.55 h), so this is
        // the bottom of the person's mask). tau <= 0 switches the gate off; a person without a mask height is not gated.
        inline bool FootContactValid(double footV, double rootV, double maskH, double tau)
        {
            if (!(tau > 0) || !(maskH > 0))
                return true;
            return (rootV + 0.55 * maskH - footV) <= tau * maskH;
        }

        // ------------------------------------------------------------------------------------------- P9-L Part 4 step 1b: poster / static-person rejection
        // GT-free. A printed person (a poster on an advertising column) is detected as a person; it is static, so its estimated world root hardly moves while the camera passes. Rule (calibrated on
        // subway_s1_24h, p9l_part4 report): a track is a poster once, within one continuous record (one map frame, gaps <= gapS), it has lasted >= minDurationS, the camera baseline since the first
        // state subtends >= minParallaxDeg at the mean range, and the horizontal bounding box of its (latest) root estimates is <= maxExtentM across. The decision is latched for the sequence.
        struct PosterParams
        {
            double minDurationS, minParallaxDeg, maxExtentM;
        };
        struct PosterInfo
        {
            double tFlag = 0, durationS = 0, extentM = 0, parallaxDeg = 0;
        };
        class PosterDetector
        {
        public:
            explicit PosterDetector(const PosterParams &p) : P(p) {}
            // One estimated person state: its keyframe id, time, root (x, y), camera centre (cx, cy, cz) in the same map frame; `epoch` identifies the map frame (a new epoch restarts the record).
            void Observe(int track, unsigned long epoch, unsigned long kf, double t, double rx, double ry, double rz, double cx, double cy, double cz)
            {
                if (flagged.count(track))
                    return;
                Rec &r = rec[track];
                if (!r.pts.empty() && (r.epoch != epoch || t - r.tMax > gapS))
                    r = Rec();
                r.epoch = epoch;
                r.tMax = std::max(r.tMax, t);
                r.pts[kf] = {t, rx, ry, rz, cx, cy, cz};
                Evaluate(track, r, t);
            }
            bool IsPoster(int track) const { return flagged.count(track) > 0; }
            const std::map<int, PosterInfo> &Flagged() const { return flagged; }
            static constexpr double gapS = 1.0;

        private:
            struct Pt
            {
                double t, x, y, z, cx, cy, cz;
            };
            struct Rec
            {
                unsigned long epoch = 0;
                double tMax = -1e30;
                std::map<unsigned long, Pt> pts;
            };
            void Evaluate(int track, const Rec &r, double now)
            {
                if (r.pts.size() < 4)
                    return;
                const Pt *first = nullptr, *last = nullptr;
                double xmin = 1e30, xmax = -1e30, ymin = 1e30, ymax = -1e30, rangeSum = 0;
                for (const auto &kv : r.pts)
                {
                    const Pt &p = kv.second;
                    if (!first || p.t < first->t) first = &p;
                    if (!last || p.t > last->t) last = &p;
                    xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x); ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
                    rangeSum += std::sqrt((p.x - p.cx) * (p.x - p.cx) + (p.y - p.cy) * (p.y - p.cy) + (p.z - p.cz) * (p.z - p.cz));
                }
                const double dur = last->t - first->t;
                if (dur < P.minDurationS)
                    return;
                const double extent = std::hypot(xmax - xmin, ymax - ymin);
                const double base = std::sqrt((last->cx - first->cx) * (last->cx - first->cx) + (last->cy - first->cy) * (last->cy - first->cy) + (last->cz - first->cz) * (last->cz - first->cz));
                const double range = rangeSum / static_cast<double>(r.pts.size());
                const double par = std::atan2(base, range) * 180.0 / M_PI;
                if (par >= P.minParallaxDeg && extent <= P.maxExtentM)
                {
                    PosterInfo f;
                    f.tFlag = now; f.durationS = dur; f.extentM = extent; f.parallaxDeg = par;
                    flagged[track] = f;
                }
            }
            PosterParams P;
            std::map<int, Rec> rec;
            std::map<int, PosterInfo> flagged;
        };

        // ------------------------------------------------------------------------------------------- RANSAC for the points of a level (patch A)
        struct PointZ
        {
            int id;
            double x, y, z;
        };
        struct PlaneFit
        {
            bool ok = false;
            double z = 0;
            std::vector<int> inliers; // PointZ::id
        };

        // Gravity-constrained RANSAC (model: a horizontal plane z = h) on the points within ransacRadiusM horizontally of any contact of the level.
        // Hypotheses: the seed z_l first, then ransacIters - 1 candidate heights drawn from the points by a fixed-seed generator (deterministic).
        // Score: inlier count within ransacThrK sigmaPt, ties to the lower |h - z_l|. The winner is refined by the inlier mean. Accepted only with
        // >= ransacMinInliers inliers and a height within ransacMaxOffsetK sigmaA of z_l; never creates a level.
        inline PlaneFit RansacLevelPoints(const std::vector<PointZ> &pts, const std::vector<Contact> &contacts, double zLevel, const LevelParams &P)
        {
            std::vector<const PointZ *> cand;
            const double r2 = P.ransacRadiusM * P.ransacRadiusM;
            for (const PointZ &p : pts)
                for (const Contact &c : contacts)
                    if ((p.x - c.x) * (p.x - c.x) + (p.y - c.y) * (p.y - c.y) <= r2)
                    {
                        cand.push_back(&p);
                        break;
                    }
            PlaneFit f;
            if (static_cast<int>(cand.size()) < P.ransacMinInliers)
                return f;
            const double thr = P.ransacThrK * P.sigmaPt;
            uint64_t state = 0x9E3779B97F4A7C15ull;
            double bestH = zLevel;
            int bestN = -1;
            for (int it = 0; it < P.ransacIters; ++it)
            {
                double h = zLevel;
                if (it > 0)
                {
                    state = state * 6364136223846793005ull + 1442695040888963407ull;
                    h = cand[(state >> 33) % cand.size()]->z;
                }
                int n = 0;
                for (const PointZ *p : cand)
                    n += std::abs(p->z - h) <= thr ? 1 : 0;
                if (n > bestN || (n == bestN && std::abs(h - zLevel) < std::abs(bestH - zLevel)))
                {
                    bestN = n;
                    bestH = h;
                }
            }
            double sum = 0;
            for (const PointZ *p : cand)
                if (std::abs(p->z - bestH) <= thr)
                {
                    f.inliers.push_back(p->id);
                    sum += p->z;
                }
            if (static_cast<int>(f.inliers.size()) < P.ransacMinInliers)
            {
                f.inliers.clear();
                return f;
            }
            f.z = sum / f.inliers.size();
            if (std::abs(f.z - zLevel) > P.ransacMaxOffsetK * P.sigmaA)
            {
                f.inliers.clear();
                return f;
            }
            f.ok = true;
            return f;
        }
    } // namespace ped
} // namespace ORB_SLAM3
#endif
