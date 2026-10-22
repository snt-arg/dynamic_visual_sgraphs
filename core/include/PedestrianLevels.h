// P9-C5: floor-level bookkeeping (association, creation, merging, transitional segments, RANSAC point selection). Header only, no g2o and no
// ORB-SLAM3 dependency, so it is unit-tested standalone (core/test/test_levels.cc). The g2o vertices and factors are in PedestrianFactors.h.
//
// Every threshold is a field of LevelParams and has NO default: the caller (settings reader) must provide it, or fail (fail-fast rule of P9-C5).
// Design constants declared in task_C5.md: creation needs >= 10 contacts from >= 2 tracks agreeing within +-2 sigma_a, association < 3 sigma_a,
// merge < 2 sigma_a, transitional window 1.5 s with >= 4 contacts and >= 0.5 m travel, RANSAC threshold 2 sigma_pt.
// Calibrated (constants_c5.md): sigma_a, sigma_pt, s_max.
#ifndef PEDESTRIAN_LEVELS_H
#define PEDESTRIAN_LEVELS_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
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
            double transWindowS, transMinTravelM;      // 1.5, 0.5
            int transMinContacts;                      // 4
            double ransacThrK, ransacRadiusM;          // 2 (x sigmaPt), 1.0
            int ransacMinInliers;                      // 5
            double ransacMaxOffsetK;                   // 3 (x sigmaA): consensus height must lie this close to z_l
            int ransacIters;                           // RansacIterations(0.99, 0.5)
        };

        // The design constants of task_C5.md / patch_stairs_ransac.md; only the three calibrated values are inputs.
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
            double t;           // seconds
            double x, y;        // person footprint, world (horizontal)
            double h;           // implied floor height h_c
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
        };

        class LevelSet
        {
        public:
            explicit LevelSet(const LevelParams &p) : P(p), nextId(0) {}
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
                for (Contact &c : cs)
                {
                    c.level = Associate(c);
                    const bool fresh = seen.insert({c.kf, c.track}).second; // a contact is counted / pooled once, however many BA windows contain it
                    if (c.level >= 0)
                    {
                        if (fresh)
                            ++Find(c.level).nContacts;
                    }
                    else if (!c.transitional && fresh)
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

            // Pool-based creation: the densest +-createK sigmaA cluster of pooled h; created when it has >= createMinContacts contacts from
            // >= createMinTracks tracks. Initial z is the median; the pooled contacts of the cluster are consumed.
            bool CreateFromPool()
            {
                if (static_cast<int>(pool.size()) < P.createMinContacts)
                    return false;
                std::vector<double> hs;
                for (const Contact &c : pool)
                    hs.push_back(c.h);
                std::sort(hs.begin(), hs.end());
                const double w = P.createK * P.sigmaA;
                int bestI = -1, bestN = 0;
                for (int i = 0; i < static_cast<int>(hs.size()); ++i)
                {
                    int n = 0;
                    for (double h : hs)
                        n += std::abs(h - hs[i]) <= w ? 1 : 0;
                    if (n > bestN)
                    {
                        bestN = n;
                        bestI = i;
                    }
                }
                std::vector<double> in;
                std::set<int> tracks;
                for (const Contact &c : pool)
                    if (std::abs(c.h - hs[bestI]) <= w)
                    {
                        in.push_back(c.h);
                        tracks.insert(c.track);
                    }
                if (static_cast<int>(in.size()) < P.createMinContacts || static_cast<int>(tracks.size()) < P.createMinTracks)
                    return false;
                std::sort(in.begin(), in.end());
                const double med = in.size() % 2 ? in[in.size() / 2] : 0.5 * (in[in.size() / 2 - 1] + in[in.size() / 2]);
                Level L;
                L.id = nextId++;
                L.z = med;
                L.nContacts = static_cast<int>(in.size());
                levels.push_back(L);
                std::vector<Contact> rest;
                for (const Contact &c : pool)
                    if (std::abs(c.h - med) > w)
                        rest.push_back(c);
                pool.swap(rest);
                return true;
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
                                const double wi = std::max(1, levels[i].nContacts), wj = std::max(1, levels[j].nContacts); // int -> double below
                                levels[i].z = (wi * levels[i].z + wj * levels[j].z) / (wi + wj);
                                levels[i].nContacts += levels[j].nContacts;
                                levels.erase(levels.begin() + j);
                                ++merges;
                                again = true;
                            }
                }
            }

            void SetHeight(int id, double z) { Find(id).z = z; }
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
