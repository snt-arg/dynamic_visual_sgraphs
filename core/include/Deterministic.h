/**
 * P9-B: process-wide switch for the deterministic (fully synchronous) mode of the ORB-SLAM3 core.
 *
 * Set once by System::System() from the settings key `System.Deterministic`. When on, LocalMapping,
 * LoopClosing and global BA run inside the tracking thread (System::RunBackendSynchronously), no other
 * thread may start, and every RNG is seeded. Default off: the threaded behaviour is unchanged.
 *
 * Header-only on purpose: the flag is shared by the core library, the runner and the tests.
 */
#ifndef ORB_SLAM3_DETERMINISTIC_H
#define ORB_SLAM3_DETERMINISTIC_H

#include <atomic>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string>

namespace ORB_SLAM3
{
    namespace Deterministic
    {
        inline std::atomic<bool> &Flag()
        {
            static std::atomic<bool> flag(false);
            return flag;
        }

        inline bool Enabled() { return Flag().load(std::memory_order_relaxed); }

        inline void Set(bool on) { Flag().store(on, std::memory_order_relaxed); }

        // Every site that would start a thread calls this first. It is a no-op in threaded mode.
        inline void ThrowIfThreadStart(const char *what)
        {
            if (Enabled())
                throw std::runtime_error(std::string("[Deterministic] refusing to start a thread (") + what +
                                         "): System.Deterministic is set");
        }

        // Seeded Gaussian perturbation of the undistorted keypoint coordinates (P9-B stage 2).
        // sigma_px == 0 means off. Used only by the tracking thread, so no locking.
        struct Jitter
        {
            double sigmaPx = 0.0;
            std::uint64_t seed = 0;
            std::mt19937_64 rng;
            std::normal_distribution<float> dist{0.f, 1.f};
            std::uint64_t nDraws = 0;

            void Configure(double sigma, std::uint64_t s)
            {
                sigmaPx = sigma;
                seed = s;
                rng.seed(s);
                dist.reset();
                nDraws = 0;
            }
            float Next()
            {
                ++nDraws;
                return static_cast<float>(sigmaPx) * dist(rng);
            }
        };

        inline Jitter &GetJitter()
        {
            static Jitter jitter;
            return jitter;
        }
    } // namespace Deterministic
} // namespace ORB_SLAM3

#endif
