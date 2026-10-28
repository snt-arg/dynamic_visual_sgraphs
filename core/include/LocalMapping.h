/**
 * This file is a modified version of a file from ORB-SLAM3.
 * 
 * Modifications Copyright (C) 2023-2025 SnT, University of Luxembourg
 * Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 * 
 * Original Copyright (C) 2014-2021 University of Zaragoza:
 * Raúl Mur-Artal, Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez,
 * José M.M. Montiel, and Juan D. Tardós.
 * 
 * This file is part of vS-Graphs, which is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * vS-Graphs is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Settings.h"
#include "Types/SystemParams.h"

#include <deque>
#include <functional>
#include <mutex>

namespace ORB_SLAM3
{

    class System;
    class Tracking;
    class LoopClosing;
    class Atlas;

    class LocalMapping
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using KeyFrameCreatedCallback = std::function<void(const KeyFrame *)>;

        LocalMapping(System *pSys, Atlas *pAtlas, const float bMonocular, bool bInertial, const string &_strSeqName = std::string());

        void SetLoopCloser(LoopClosing *pLoopCloser);

        void SetTracker(Tracking *pTracker);
        void SetKeyFrameCreatedCallback(KeyFrameCreatedCallback callback);

        // Main function
        void Run();

        // [P9-B] Deterministic mode: the mapper has no thread; the tracking thread drains its queue.
        void SetSynchronous();
        void RunSynchronously();
        bool CheckNewKeyFrames();

        void InsertKeyFrame(KeyFrame *pKF);
        void EmptyQueue();

        // Thread Synch
        void RequestStop();
        void RequestReset();
        void RequestResetActiveMap(Map *pMap);
        bool Stop();
        void Release();
        bool isStopped();
        bool stopRequested();
        bool AcceptKeyFrames();
        void SetAcceptKeyFrames(bool flag);
        bool SetNotStop(bool flag);

        void InterruptBA();

        void RequestFinish();
        bool isFinished();

        int KeyframesInQueue()
        {
            unique_lock<std::mutex> lock(mMutexNewKFs);
            return mlNewKeyFrames.size();
        }

        bool IsInitializing();
        double GetCurrKFTime();
        KeyFrame *GetCurrKF();

        std::mutex mMutexImuInit;

        Eigen::MatrixXd mcovInertial;
        Eigen::Matrix3d mRwg;
        Eigen::Vector3d mbg;
        Eigen::Vector3d mba;
        double mScale;
        double mInitTime;
        double mCostTime;

        unsigned int mInitSect;
        unsigned int mIdxInit;
        unsigned int mnKFs;
        double mFirstTs = 0.0; // read by System::GetTimeFromIMUInit() every frame before the IMU is initialised
        int mnMatchesInliers;

        // ---- W2 pedestrian scale feedback (docs/evaluation/results/w2_scale_controller/) ----
        // A measurement pushed in from ROS (System::setScaleMeasurement -> common.cc callback).
        // scale is METRES PER MAP UNIT, already divided by the W1 calibration constant c, i.e.
        // exactly the factor Map::ApplyScaledRotation takes. See criterion.md sec 3.2.
        struct ScaleMeasurement
        {
            double scale = 1.0;
            double sigma = 0.0;
            double stamp = 0.0;
            double window_s = 0.0; // the observer's measurement window (Y1: no-reversal horizon)
        };
        void SetScaleMeasurement(double scale, double sigma, double stamp, double windowS);

        // Applied-correction trace, for the campaign reads (criterion.md sec 6).
        struct ScaleCorrectionRecord
        {
            double stamp = 0.0;
            double s_measured = 1.0;  // newest reading
            double level = 1.0;       // Y1: median of the last level_n readings -- what was acted on
            double sigma_level = 0.0;
            double s_applied = 1.0;
            double map_time = 0.0;
            unsigned int index = 0;
        };
        std::vector<ScaleCorrectionRecord> GetScaleCorrectionTrace();

        // For debugging (erase in normal mode)
        int mInitFr;
        int mIdxIteration;
        string strSequence;

        bool mbNotBA1;
        bool mbNotBA2;
        bool mbBadImu;

        bool mbWriteStats;

        // not consider far points (clouds)
        bool mbFarPoints;
        float mThFarPoints;

#ifdef REGISTER_TIMES
        vector<double> vdKFInsert_ms;
        vector<double> vdMPCulling_ms;
        vector<double> vdMPCreation_ms;
        vector<double> vdLBA_ms;
        vector<double> vdKFCulling_ms;
        vector<double> vdLMTotal_ms;

        vector<double> vdLBASync_ms;
        vector<double> vdKFCullingSync_ms;
        vector<int> vnLBA_edges;
        vector<int> vnLBA_KFopt;
        vector<int> vnLBA_KFfixed;
        vector<int> vnLBA_MPs;
        int nLBA_exec;
        int nLBA_abort;
#endif
    protected:
        bool RunOnce();
        void ProcessNewKeyFrame();
        void CreateNewMapPoints();

        void MapPointCulling();
        void SearchInNeighbors();
        void KeyFrameCulling();

        System *mpSystem;

        bool mbMonocular;
        bool mbInertial;

        void ResetIfRequested();
        bool mbResetRequested;
        bool mbResetRequestedActiveMap;
        Map *mpMapToReset;
        std::mutex mMutexReset;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        // [P9-B] true when the mapper runs inside the tracking thread (System.Deterministic)
        bool mbSynchronous = false;
        std::mutex mMutexFinish;

        Atlas *mpAtlas;

        LoopClosing *mpLoopCloser;
        Tracking *mpTracker;

        std::list<KeyFrame *> mlNewKeyFrames;

        KeyFrame *mpCurrentKeyFrame;
        KeyFrameCreatedCallback mKeyFrameCreatedCallback;

        std::list<MapPoint *> mlpRecentAddedMapPoints;

        std::mutex mMutexNewKFs;
        std::mutex mMutexNewRooms;

        bool mbAbortBA;

        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::mutex mMutexStop;

        bool mbAcceptKeyFrames;
        std::mutex mMutexAccept;

        void InitializeIMU(float priorG = 1e2, float priorA = 1e6, bool bFirst = false, const char *cause = "OTHER"); // P9-H A4.4: cause of the epoch change it makes (IMU_INIT0 | STAGE1 | STAGE2)
        void ScaleRefinement();

        bool bInitializing;

        Eigen::MatrixXd infoInertial;
        int mNumLM;
        int mNumKFCulling;

        float mTinit;

        // ---- W2/Y1 pedestrian scale feedback state, LocalMapping-owned ----
        // W2 acted on the single newest reading, which X1a showed is by construction an upper-tail
        // one (trigger/level 1.05-1.23). Y1 queues every reading and acts on the median of the last
        // level_n that survived the plausibility bound and postdate the last correction.
        void MaybeApplyScaleCorrection();
        void ApplyScaleCorrection(Map *pMap, double now, double mapTime,
                                  const ScaleMeasurement &meas, double level, double sigmaLevel);
        // Y2 single checkpoint state
        ScaleMeasurement mLatestScaleMeas;
        bool mbHaveLatestScaleMeas = false;
        bool mbScaleCheckpointDecided = false;
        std::mutex mMutexScaleMeas;
        std::vector<ScaleMeasurement> mvScaleMeasPending;
        std::deque<ScaleMeasurement> mdqScaleLevel;
        double mdLastScaleCorrectionStamp = -1e9;
        int mnLastScaleCorrectionSign = 0;
        unsigned int mnScaleCorrections = 0;
        // Y1: per-test rejection census, printed with every decision line.
        unsigned long mnScaleRejPlausibility = 0, mnScaleRejStale = 0, mnScaleRejSigma = 0,
                      mnScaleRejDeadband = 0, mnScaleRejReversal = 0, mnScaleRejLevelNotReady = 0;
        std::mutex mMutexScaleTrace;
        std::vector<ScaleCorrectionRecord> mvScaleCorrectionTrace;

        int countRefinement;

        // DEBUG
        ofstream f_lm;
    };

}

#endif
