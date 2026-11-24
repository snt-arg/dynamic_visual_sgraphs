// Hierarchical stature anchor (Pedestrians.ScaleAnchorMode: hier; order of 23 Nov 2026): the per-map population offset kappa.
// kappa is NOT a BA vertex. After every local inertial BA call each track with qualifying anchor observations in the call contributes the median over them of
// x = log z_metric - log z_map (evaluated at the solved estimates); the store keeps the latest value per track (each track counted once) and kappa = median over tracks
// (mean of the two middle values for an even count). The anchor is active once at least MinTracks tracks have contributed.
// Variant F (Pedestrians.ScaleAnchorFreezeTracks = N > 0): once N tracks have contributed, kappa is frozen at the median of that moment for the map's lifetime (the live median is
// still kept, as the unfrozen counterfactual for the logs); a similarity shifts the frozen value like the stored x; Clear also unfreezes.
// Lifecycle like the map's level set: cleared by Map::clear (reset_active_map) and by ResetPedestrians (loop / merge); carried through ApplyScaledRotation:
// map lengths are multiplied by s, so every stored x = log z_metric - log z_map shifts by -log s.
#pragma once
#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

namespace ORB_SLAM3
{
    namespace ped
    {
        class AnchorKappaStore
        {
        public:
            void Update(const std::map<int, double> &trackMedians)
            {
                for (const auto &kv : trackMedians)
                    x_[kv.first] = kv.second;
            }
            // variant F: update, then freeze once freezeN (> 0) tracks have contributed; true when this call froze it
            bool Update(const std::map<int, double> &trackMedians, int freezeN, double stamp)
            {
                Update(trackMedians);
                if (frozen_ || freezeN <= 0 || N() < freezeN)
                    return false;
                frozen_ = true;
                kFrozen_ = kAtFreeze_ = KappaLive();
                nAtFreeze_ = N();
                tFreeze_ = stamp;
                return true;
            }
            int N() const { return static_cast<int>(x_.size()); }
            bool Frozen() const { return frozen_; }
            double FreezeStamp() const { return tFreeze_; }
            double KappaAtFreeze() const { return kAtFreeze_; } // as computed at the freeze (not shifted by later similarities)
            int NAtFreeze() const { return nAtFreeze_; }
            double Kappa() const { return frozen_ ? kFrozen_ : KappaLive(); }
            double KappaLive() const
            {
                if (x_.empty())
                    return 0.0;
                std::vector<double> v;
                v.reserve(x_.size());
                for (const auto &kv : x_)
                    v.push_back(kv.second);
                std::sort(v.begin(), v.end());
                const size_t n = v.size();
                return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
            }
            void Shift(double d)
            {
                for (auto &kv : x_)
                    kv.second += d;
                if (frozen_)
                    kFrozen_ += d;
            }
            void Clear()
            {
                x_.clear();
                frozen_ = false;
                kFrozen_ = kAtFreeze_ = 0.0;
                nAtFreeze_ = 0;
                tFreeze_ = -1.0;
            }

        private:
            std::map<int, double> x_; // track id -> latest per-call median x
            bool frozen_ = false;     // variant F
            double kFrozen_ = 0.0, kAtFreeze_ = 0.0, tFreeze_ = -1.0;
            int nAtFreeze_ = 0;
        };

        inline double MedianOf(std::vector<double> v)
        {
            if (v.empty())
                return 0.0;
            std::sort(v.begin(), v.end());
            const size_t n = v.size();
            return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
        }
    } // namespace ped
} // namespace ORB_SLAM3
