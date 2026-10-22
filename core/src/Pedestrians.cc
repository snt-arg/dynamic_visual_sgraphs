#include "Pedestrians.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

namespace ORB_SLAM3
{
    PedestrianParams &PedestrianParams::Get()
    {
        static PedestrianParams p;
        return p;
    }

    namespace
    {
        double Req(cv::FileStorage &fs, const char *key, bool allowZero = false)
        {
            const cv::FileNode n = fs[key];
            if (n.empty())
                throw std::runtime_error(std::string("[Pedestrians] Pedestrians.Enabled is set but ") + key + " is missing from the settings file");
            return static_cast<double>(n);
        }
    } // namespace

    bool PedestrianParams::Configure(cv::FileStorage &fs)
    {
        PedestrianParams &p = Get();
        p = PedestrianParams();
        const cv::FileNode e = fs["Pedestrians.Enabled"];
        p.enabled = !e.empty() && static_cast<int>(e) != 0;
        if (!p.enabled)
            return false;
        p.rootPxSigma = Req(fs, "Pedestrians.RootPxSigma");
        p.logDepthSigma = Req(fs, "Pedestrians.LogDepthSigma");
        p.skelOverStature = Req(fs, "Pedestrians.SkeletonOverStature");
        p.statureFixedM = Req(fs, "Pedestrians.StatureFixedM");
        p.qv = Req(fs, "Pedestrians.Qv");
        p.footPxSigma = Req(fs, "Pedestrians.FootprintPxSigma");
        p.t1Sigma = Req(fs, "Pedestrians.T1Sigma");
        p.t2Sigma = Req(fs, "Pedestrians.T2Sigma");
        p.cRoot = Req(fs, "Pedestrians.CRoot");
        p.cLeg = Req(fs, "Pedestrians.CLeg");
        p.minTrackKeyframes = static_cast<int>(Req(fs, "Pedestrians.MinTrackKeyframes"));
        p.minStanceBoxPx = Req(fs, "Pedestrians.MinStanceBoxPx");
        p.minFootObservations = static_cast<int>(Req(fs, "Pedestrians.MinFootObservations"));
        p.minLpx = Req(fs, "Pedestrians.MinLpx", true);
        p.huberScale = Req(fs, "Pedestrians.HuberScale");
        p.maskHeightM = Req(fs, "Pedestrians.MaskHeightM");
        p.gateCenter = Req(fs, "Pedestrians.GateCenter", true);
        p.gateSigma = Req(fs, "Pedestrians.GateSigma");
        p.gateK = Req(fs, "Pedestrians.GateK", true);
        const cv::FileNode fc = fs["Pedestrians.FloorContact"];
        if (fc.empty())
            throw std::runtime_error("[Pedestrians] Pedestrians.Enabled is set but Pedestrians.FloorContact (none | regional | joint) is missing from the settings file");
        const std::string fm = static_cast<std::string>(fc);
        if (fm == "none")
            p.floorMode = FloorMode::None;
        else if (fm == "regional")
            p.floorMode = FloorMode::Regional;
        else if (fm == "joint")
            p.floorMode = FloorMode::Joint;
        else if (fm == "shadow")
            p.floorMode = FloorMode::Shadow;
        else if (fm == "regional_px")
            p.floorMode = FloorMode::RegionalPx;
        else if (fm == "joint_px")
            p.floorMode = FloorMode::JointPx;
        else if (fm == "levels")
            p.floorMode = FloorMode::Levels;
        else if (fm == "levels_shadow")
            p.floorMode = FloorMode::LevelsShadow;
        else
            throw std::runtime_error("[Pedestrians] Pedestrians.FloorContact must be none, regional, joint, shadow, regional_px, joint_px, levels or levels_shadow, got " + fm);
        if (p.floorMode == FloorMode::Levels || p.floorMode == FloorMode::LevelsShadow)
        {
            p.floorMidDeltaM = Req(fs, "Pedestrians.FloorMidDeltaM");
            p.floorSigmaPxU = Req(fs, "Pedestrians.FloorSigmaPxU");
            p.floorSigmaPxV = Req(fs, "Pedestrians.FloorSigmaPxV");
            p.biasSigma = Req(fs, "Pedestrians.BiasSigma");
            p.biasMinContacts = static_cast<int>(Req(fs, "Pedestrians.BiasMinContacts", true));
            p.levelSigmaA = Req(fs, "Pedestrians.LevelSigmaA");
            p.levelSigmaPt = Req(fs, "Pedestrians.LevelSigmaPt");
            p.levelSMax = Req(fs, "Pedestrians.LevelSMax");
            for (double v : {p.floorMidDeltaM, p.floorSigmaPxU, p.floorSigmaPxV, p.biasSigma, p.levelSigmaA, p.levelSigmaPt, p.levelSMax})
                if (!(v > 0))
                    throw std::runtime_error("[Pedestrians] levels: FloorMidDeltaM, FloorSigmaPx*, BiasSigma, LevelSigmaA, LevelSigmaPt, LevelSMax must be > 0");
            if (p.biasMinContacts < 0)
                throw std::runtime_error("[Pedestrians] Pedestrians.BiasMinContacts must be >= 0");
            const cv::FileNode ph = fs["Pedestrians.PlatformHeight"];
            if (ph.empty())
                throw std::runtime_error("[Pedestrians] FloorContact levels needs Pedestrians.PlatformHeight (off | on)");
            const std::string phs = static_cast<std::string>(ph);
            if (phs == "on")
            {
                p.platformHeight = true;
                p.platformHeightM = Req(fs, "Pedestrians.PlatformHeightM");
                p.platformSigmaM = Req(fs, "Pedestrians.PlatformSigmaM");
                if (!(p.platformHeightM > 0 && p.platformSigmaM > 0))
                    throw std::runtime_error("[Pedestrians] PlatformHeightM and PlatformSigmaM must be > 0");
            }
            else if (phs == "off")
                p.platformHeight = false;
            else
                throw std::runtime_error("[Pedestrians] Pedestrians.PlatformHeight must be off or on, got " + phs);
            std::cout << "[Pedestrians] floor contact " << fm << ": delta_v " << p.floorMidDeltaM << " m, sigma px " << p.floorSigmaPxU << "/" << p.floorSigmaPxV << ", bias sigma " << p.biasSigma
                      << " (min contacts " << p.biasMinContacts << "), sigma_a " << p.levelSigmaA << " m, sigma_pt " << p.levelSigmaPt << " m, s_max " << p.levelSMax << ", platform height "
                      << (p.platformHeight ? "on" : "off") << std::endl;
        }
        else if (p.floorMode != FloorMode::None)
        {
            const bool px = p.floorMode == FloorMode::Shadow || p.floorMode == FloorMode::RegionalPx || p.floorMode == FloorMode::JointPx;
            if (!px)
            {
                p.floorSigmaRadialM = Req(fs, "Pedestrians.FloorSigmaRadialM");
                p.floorSigmaTangentialM = Req(fs, "Pedestrians.FloorSigmaTangentialM");
                p.floorContactOffsetM = Req(fs, "Pedestrians.FloorContactOffsetM", true);
            }
            else
            {
                p.floorMidDeltaM = Req(fs, "Pedestrians.FloorMidDeltaM");
                p.floorSigmaPxU = Req(fs, "Pedestrians.FloorSigmaPxU");
                p.floorSigmaPxV = Req(fs, "Pedestrians.FloorSigmaPxV");
                p.floorZfBiasM = Req(fs, "Pedestrians.FloorZfBiasM", true);
                p.floorRaisedMaxM = Req(fs, "Pedestrians.FloorRaisedMaxM");
                p.floorMinElevDeg = Req(fs, "Pedestrians.FloorMinElevDeg");
                p.floorPlaneSigmaMaxM = Req(fs, "Pedestrians.FloorPlaneSigmaMaxM");
                for (double v : {p.floorMidDeltaM, p.floorSigmaPxU, p.floorSigmaPxV, p.floorRaisedMaxM, p.floorPlaneSigmaMaxM})
                    if (!(v > 0))
                        throw std::runtime_error("[Pedestrians] midpoint floor contact: FloorMidDeltaM, FloorSigmaPx*, FloorRaisedMaxM, FloorPlaneSigmaMaxM must be > 0");
            }
            p.floorRegionRadiusM = Req(fs, "Pedestrians.FloorRegionRadiusM");
            p.biasSigma = Req(fs, "Pedestrians.BiasSigma");
            p.biasMinContacts = static_cast<int>(Req(fs, "Pedestrians.BiasMinContacts", true));
            if (p.biasMinContacts < 0)
                throw std::runtime_error("[Pedestrians] Pedestrians.BiasMinContacts must be >= 0");
            p.floorLowModeFraction = Req(fs, "Pedestrians.FloorLowModeFraction");
            p.floorMinPoints = static_cast<int>(Req(fs, "Pedestrians.FloorMinPoints"));
            p.floorBinM = Req(fs, "Pedestrians.FloorBinM");
            p.floorBelowCameraM = Req(fs, "Pedestrians.FloorBelowCameraM", true);
            p.floorPlausM = Req(fs, "Pedestrians.FloorPlausM");
            if (p.floorMode == FloorMode::Joint || p.floorMode == FloorMode::JointPx || p.floorMode == FloorMode::Shadow)
            {
                p.floorNeighbours = static_cast<int>(Req(fs, "Pedestrians.FloorNeighbours"));
                p.floorNeighbourRadiusM = Req(fs, "Pedestrians.FloorNeighbourRadiusM");
                p.floorBandM = Req(fs, "Pedestrians.FloorBandM");
                if (p.floorNeighbours < 3)
                    throw std::runtime_error("[Pedestrians] Pedestrians.FloorNeighbours must be >= 3");
                if (!(p.floorNeighbourRadiusM > 0 && p.floorBandM > 0))
                    throw std::runtime_error("[Pedestrians] joint floor contact: FloorNeighbourRadiusM and FloorBandM must be > 0");
            }
            for (double v : {p.floorRegionRadiusM, p.floorBinM, p.floorPlausM, p.floorLowModeFraction, p.biasSigma})
                if (!(v > 0))
                    throw std::runtime_error("[Pedestrians] every floor-contact sigma / radius / bin must be > 0");
            if (!px)
                for (double v : {p.floorSigmaRadialM, p.floorSigmaTangentialM})
                    if (!(v > 0))
                        throw std::runtime_error("[Pedestrians] every floor-contact sigma / radius / bin must be > 0");
            if (p.floorMinPoints < 1)
                throw std::runtime_error("[Pedestrians] Pedestrians.FloorMinPoints must be >= 1");
            std::cout << "[Pedestrians] floor contact " << fm << ": bias sigma " << p.biasSigma << " (min contacts " << p.biasMinContacts << "), region " << p.floorRegionRadiusM << " m, low-mode fraction " << p.floorLowModeFraction << ", min points " << p.floorMinPoints
                      << ", bin " << p.floorBinM << " m, below camera " << p.floorBelowCameraM;
            if (px)
                std::cout << ", midpoint delta " << p.floorMidDeltaM << " m, sigma px " << p.floorSigmaPxU << "/" << p.floorSigmaPxV << ", z_f bias " << p.floorZfBiasM << " m, raised max " << p.floorRaisedMaxM << " m, min elevation "
                          << p.floorMinElevDeg << " deg, plane sigma max " << p.floorPlaneSigmaMaxM << " m";
            else
                std::cout << ", sigma radial " << p.floorSigmaRadialM << " m, tangential " << p.floorSigmaTangentialM << " m, keypoint offset " << p.floorContactOffsetM << " m";
            if (p.floorMode == FloorMode::Joint || p.floorMode == FloorMode::JointPx || p.floorMode == FloorMode::Shadow)
                std::cout << ", k " << p.floorNeighbours << ", neighbour radius " << p.floorNeighbourRadiusM << " m, band " << p.floorBandM << " m";
            std::cout << std::endl;
        }
        // P9-C4 (keys required, no defaults: a missing key must not silently mean "base")
        const cv::FileNode mp = fs["Pedestrians.MotionPrior"];
        if (mp.empty())
            throw std::runtime_error("[Pedestrians] Pedestrians.Enabled is set but Pedestrians.MotionPrior (isotropic | anisotropic) is missing from the settings file");
        const std::string mps = static_cast<std::string>(mp);
        if (mps == "isotropic")
            p.motionPrior = MotionPrior::Isotropic;
        else if (mps == "anisotropic")
        {
            p.motionPrior = MotionPrior::Anisotropic;
            p.qvH = Req(fs, "Pedestrians.QvH");
            p.qvZ = Req(fs, "Pedestrians.QvZ");
            if (!(p.qvH > 0 && p.qvZ > 0))
                throw std::runtime_error("[Pedestrians] Pedestrians.QvH and Pedestrians.QvZ must be > 0");
        }
        else
            throw std::runtime_error("[Pedestrians] Pedestrians.MotionPrior must be isotropic or anisotropic, got " + mps);
        const int gateMode = static_cast<int>(Req(fs, "Pedestrians.InnovGate", true));
        if (gateMode < 0 || gateMode > 2)
            throw std::runtime_error("[Pedestrians] Pedestrians.InnovGate must be 0 (off), 1 (on) or 2 (shadow: log only)");
        p.innovGate = gateMode != 0;
        p.innovShadow = gateMode == 2;
        if (p.innovGate)
        {
            p.innovChi2Dof2 = Req(fs, "Pedestrians.InnovChi2Dof2");
            p.innovChi2Dof3 = Req(fs, "Pedestrians.InnovChi2Dof3");
            p.innovFreeKeyframes = static_cast<int>(Req(fs, "Pedestrians.InnovFreeKeyframes", true));
            p.innovReinitRejections = static_cast<int>(Req(fs, "Pedestrians.InnovReinitRejections"));
            p.innovP0PosXY = Req(fs, "Pedestrians.InnovP0PosXY");
            p.innovP0PosZ = Req(fs, "Pedestrians.InnovP0PosZ");
            p.innovP0VelXY = Req(fs, "Pedestrians.InnovP0VelXY");
            p.innovP0VelZ = Req(fs, "Pedestrians.InnovP0VelZ");
            p.innovScale[0] = Req(fs, "Pedestrians.InnovScaleK3");
            p.innovScale[1] = Req(fs, "Pedestrians.InnovScaleK4");
            p.innovScale[2] = Req(fs, "Pedestrians.InnovScaleK5");
            p.innovScale[3] = Req(fs, "Pedestrians.InnovScaleK6");
            for (double sc : p.innovScale)
                if (!(sc > 0))
                    throw std::runtime_error("[Pedestrians] innovation gate: every InnovScaleK* must be > 0");
            for (double v : {p.innovChi2Dof2, p.innovChi2Dof3, p.innovP0PosXY, p.innovP0PosZ, p.innovP0VelXY, p.innovP0VelZ})
                if (!(v > 0))
                    throw std::runtime_error("[Pedestrians] innovation gate: every threshold / P0 standard deviation must be > 0");
            if (p.innovFreeKeyframes < 0 || p.innovReinitRejections < 1)
                throw std::runtime_error("[Pedestrians] innovation gate: InnovFreeKeyframes must be >= 0 and InnovReinitRejections >= 1");
        }
        std::cout << "[Pedestrians] motion prior " << mps;
        if (p.motionPrior == MotionPrior::Anisotropic)
            std::cout << " (q_h " << p.qvH << ", q_z " << p.qvZ << ")";
        std::cout << ", innovation gate " << (p.innovShadow ? "SHADOW (log only)" : (p.innovGate ? "ON" : "off"));
        if (p.innovGate)
            std::cout << " (chi2 " << p.innovChi2Dof2 << " / " << p.innovChi2Dof3 << ", free " << p.innovFreeKeyframes << ", reinit after " << p.innovReinitRejections << ", scale " << p.innovScale[0] << "/" << p.innovScale[1] << "/" << p.innovScale[2] << "/" << p.innovScale[3] << ", P0 pos " << p.innovP0PosXY << "/" << p.innovP0PosZ << " m, vel " << p.innovP0VelXY << "/" << p.innovP0VelZ << " m/s)";
        std::cout << std::endl;
        const cv::FileNode d = fs["Pedestrians.Dir"];
        if (d.empty())
            throw std::runtime_error("[Pedestrians] Pedestrians.Enabled is set but Pedestrians.Dir is missing from the settings file");
        p.dir = static_cast<std::string>(d);
        for (double v : {p.rootPxSigma, p.logDepthSigma, p.skelOverStature, p.statureFixedM, p.qv, p.footPxSigma, p.t1Sigma,
                         p.t2Sigma, p.cRoot, p.cLeg, p.huberScale})
            if (!(v > 0))
                throw std::runtime_error("[Pedestrians] every sigma / constant must be > 0");
        std::cout << "[Pedestrians] on: root " << p.rootPxSigma << " px, log-range " << p.logDepthSigma << ", L/H " << p.skelOverStature
                  << ", fixed stature " << p.statureFixedM << " m, q_v " << p.qv << ", foot " << p.footPxSigma
                  << " px, T1 " << p.t1Sigma << ", T2 " << p.t2Sigma << ", c_root " << p.cRoot << ", c_leg " << p.cLeg
                  << ", min track KFs " << p.minTrackKeyframes << ", min stance box " << p.minStanceBoxPx << " px, min foot obs "
                  << p.minFootObservations << ", min l_px " << p.minLpx << ", huber scale " << p.huberScale << ", gate k " << p.gateK << " (center " << p.gateCenter << ", sigma " << p.gateSigma << ", H_mask " << p.maskHeightM << ")" << ", dir " << p.dir << std::endl;
        return true;
    }

    PedestrianFrameReader::PedestrianFrameReader(const std::string &dir) : mDir(dir)
    {
        if (!PedestrianParams::Enabled())
            throw std::runtime_error("[Pedestrians] PedestrianFrameReader constructed with Pedestrians.Enabled: 0");
    }

    std::shared_ptr<PedFrame> PedestrianFrameReader::Read(double stamp) const
    {
        char name[64];
        std::snprintf(name, sizeof(name), "%019lld.ped.txt", static_cast<long long>(std::llround(stamp * 1e6)));
        const std::string path = mDir + "/" + name;
        std::ifstream f(path);
        if (!f)
            throw std::runtime_error("[Pedestrians] missing per-frame file " + path);
        ++mnOpened;
        auto pf = std::make_shared<PedFrame>();
        pf->stamp = stamp;
        std::string line;
        int lastTrack = -1;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream ss(line);
            std::string tag;
            ss >> tag;
            if (tag == "P")
            {
                PedObs o;
                ss >> o.trackId >> o.rootU >> o.rootV >> o.lpx >> o.maskH;
                if (ss.fail())
                    throw std::runtime_error("[Pedestrians] malformed line in " + path + ": " + line);
                if (o.trackId <= lastTrack)
                    throw std::runtime_error("[Pedestrians] track ids must be strictly increasing in " + path);
                lastTrack = o.trackId;
                pf->obs.push_back(o);
            }
            else if (tag == "C")
            {
                // C claimed_stamp track claimL claimR toeLu toeLv toeRu toeRv box_h
                double cs;
                PedClaim c;
                int cl, cr;
                ss >> cs >> c.trackId >> cl >> cr >> c.toeU[0] >> c.toeV[0] >> c.toeU[1] >> c.toeV[1] >> c.boxH;
                if (ss.fail())
                    throw std::runtime_error("[Pedestrians] malformed line in " + path + ": " + line);
                c.claim = {cl != 0, cr != 0};
                if (pf->lateClaims.empty() || std::abs(pf->lateClaims.back().stamp - cs) > 1e-9)
                {
                    PedLateClaims lc;
                    lc.stamp = cs;
                    pf->lateClaims.push_back(lc);
                }
                pf->lateClaims.back().claims.push_back(c);
            }
            else if (tag == "F")
            {
                // F track u v score: lowest foot keypoint of the person of the P row with that track id (P9-C3)
                int tid;
                double fu, fv, fs;
                ss >> tid >> fu >> fv >> fs;
                if (ss.fail())
                    throw std::runtime_error("[Pedestrians] malformed line in " + path + ": " + line);
                PedObs *target = nullptr;
                for (PedObs &o : pf->obs)
                    if (o.trackId == tid)
                        target = &o;
                if (!target)
                    throw std::runtime_error("[Pedestrians] F row without a P row of the same track in " + path + ": " + line);
                if (target->hasFoot)
                    throw std::runtime_error("[Pedestrians] two F rows for one track in " + path + ": " + line);
                target->hasFoot = true;
                target->footU = fu;
                target->footV = fv;
                target->footScore = fs;
            }
            else
                throw std::runtime_error("[Pedestrians] unknown row tag in " + path + ": " + line);
        }
        return pf;
    }
} // namespace ORB_SLAM3
