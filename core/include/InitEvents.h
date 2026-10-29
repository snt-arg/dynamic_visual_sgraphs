/**
 * Init-trap study: event log of the monocular-inertial initialisation, plus two debug switches.
 *
 * Configure() reads three optional keys from the settings file (System::System calls it once):
 *   Debug.InitEventsFile    path of the event log (TSV). Absent or empty: no log is written.
 *   Debug.DisableNEMReset   0 (default) | 1. At 1 the LocalMapping "Not enough motion" rule still logs, but requests no reset and
 *                           does not zero mTinit.
 *   Debug.InitMinTime       seconds, default 2.0: the monocular minimum map time before InitializeIMU may solve
 *                           (the former constant `minTime = 2.0` in LocalMapping::InitializeIMU).
 * With every key absent the behaviour is that of 9862a18.
 *
 * One row per event. Columns (tab separated; numeric fields that do not apply are `nan`):
 *   t_kf       sensor time of the current keyframe (of the current frame for MAP_RESET rows written by Tracking)
 *   frame      frame index of that keyframe/frame
 *   map        map id            n_kf   keyframes in the active map            mTinit   LocalMapping::mTinit [s] (nan from Tracking)
 *   event      INIT_SOLVE | INIT_SKIP | NEM | SCALE_REFINE | MAP_RESET
 *   stage      INIT_*: 0 first solve, 1 stage#1, 2 stage#2; else -1
 *   scale      INIT_SOLVE / SCALE_REFINE: the scale solved by InertialOptimization BEFORE acceptance (map scale correction factor)
 *   roll_deg, pitch_deg   INIT_SOLVE: Rwg as ZYX Euler roll / pitch (yaw is unobservable)
 *   ba_norm, bg_norm      INIT_SOLVE: norms of the accelerometer / gyroscope bias estimate
 *   flag       INIT_SOLVE accepted (scale >= 0.1), NEM reset requested, SCALE_REFINE applied, MAP_RESET 1
 *   dist       NEM: the `dist` (map units) that tripped the rule
 *   detail     INIT_SOLVE applied=<0|1>; INIT_SKIP reason; NEM mode; MAP_RESET source:reason
 */
#ifndef ORB_SLAM3_INIT_EVENTS_H
#define ORB_SLAM3_INIT_EVENTS_H

#include <cmath>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3
{
    namespace InitEvents
    {
        struct State
        {
            std::FILE *fp = nullptr;
            bool disableNem = false;
            float initMinTime = 2.0f;
            std::mutex mtx;
        };

        inline State &S()
        {
            static State s;
            return s;
        }

        inline bool DisableNEMReset() { return S().disableNem; }
        inline float InitMinTimeMono() { return S().initMinTime; }
        inline bool Logging() { return S().fp != nullptr; }

        // Closes the log (also used by tests that reconfigure within one process).
        inline void Close()
        {
            State &s = S();
            std::lock_guard<std::mutex> l(s.mtx);
            if (s.fp)
                std::fclose(s.fp);
            s.fp = nullptr;
        }

        inline void ConfigureValues(const std::string &file, bool disableNem, float initMinTime)
        {
            if (!(initMinTime > 0.f))
                throw std::runtime_error("[InitEvents] Debug.InitMinTime must be > 0");
            Close();
            State &s = S();
            std::lock_guard<std::mutex> l(s.mtx);
            s.disableNem = disableNem;
            s.initMinTime = initMinTime;
            if (!file.empty())
            {
                s.fp = std::fopen(file.c_str(), "w");
                if (!s.fp)
                    throw std::runtime_error("[InitEvents] cannot open " + file);
                std::fprintf(s.fp, "# init_events.log (core/include/InitEvents.h); DisableNEMReset=%d InitMinTime=%g\n", (int)disableNem, (double)initMinTime);
                std::fprintf(s.fp, "t_kf\tframe\tmap\tn_kf\tmTinit\tevent\tstage\tscale\troll_deg\tpitch_deg\tba_norm\tbg_norm\tflag\tdist\tdetail\n");
                std::fflush(s.fp);
            }
        }

        inline void Configure(cv::FileStorage &fs)
        {
            std::string file;
            bool dis = false;
            float t = 2.0f;
            cv::FileNode n = fs["Debug.InitEventsFile"];
            if (!n.empty() && n.isString())
                file = (std::string)n;
            n = fs["Debug.DisableNEMReset"];
            if (!n.empty())
                dis = static_cast<int>(n) != 0;
            n = fs["Debug.InitMinTime"];
            if (!n.empty())
                t = static_cast<float>(n);
            ConfigureValues(file, dis, t);
        }

        inline double Nan() { return std::nan(""); }

        // One row; flushes, so a crashed run keeps its events.
        inline void Write(double t_kf, long frame, long map, long n_kf, double mTinit, const char *event, int stage, double scale, double roll_deg,
                          double pitch_deg, double ba_norm, double bg_norm, int flag, double dist, const std::string &detail)
        {
            State &s = S();
            if (!s.fp)
                return;
            std::lock_guard<std::mutex> l(s.mtx);
            std::fprintf(s.fp, "%.9f\t%ld\t%ld\t%ld\t%.6g\t%s\t%d\t%.9g\t%.6g\t%.6g\t%.6g\t%.6g\t%d\t%.9g\t%s\n", t_kf, frame, map, n_kf, mTinit, event,
                         stage, scale, roll_deg, pitch_deg, ba_norm, bg_norm, flag, dist, detail.c_str());
            std::fflush(s.fp);
        }

        // ZYX Euler roll / pitch of a rotation matrix given by its row-major entries R(2,0), R(2,1), R(2,2).
        inline void RollPitchDeg(double r20, double r21, double r22, double &roll, double &pitch)
        {
            const double k = 180.0 / M_PI;
            roll = std::atan2(r21, r22) * k;
            pitch = -std::asin(std::fmax(-1.0, std::fmin(1.0, r20))) * k;
        }
    } // namespace InitEvents
} // namespace ORB_SLAM3

#endif
