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
        double Req(cv::FileStorage &fs, const char *key)
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
        p.statureMean = Req(fs, "Pedestrians.StatureMean");
        p.statureSigma = Req(fs, "Pedestrians.StatureSigma");
        p.qv = Req(fs, "Pedestrians.Qv");
        p.footPxSigma = Req(fs, "Pedestrians.FootprintPxSigma");
        p.t1Sigma = Req(fs, "Pedestrians.T1Sigma");
        p.t2Sigma = Req(fs, "Pedestrians.T2Sigma");
        p.cRoot = Req(fs, "Pedestrians.CRoot");
        p.cLeg = Req(fs, "Pedestrians.CLeg");
        p.minTrackKeyframes = static_cast<int>(Req(fs, "Pedestrians.MinTrackKeyframes"));
        p.minStanceBoxPx = Req(fs, "Pedestrians.MinStanceBoxPx");
        p.minFootObservations = static_cast<int>(Req(fs, "Pedestrians.MinFootObservations"));
        const cv::FileNode d = fs["Pedestrians.Dir"];
        if (d.empty())
            throw std::runtime_error("[Pedestrians] Pedestrians.Enabled is set but Pedestrians.Dir is missing from the settings file");
        p.dir = static_cast<std::string>(d);
        for (double v : {p.rootPxSigma, p.logDepthSigma, p.skelOverStature, p.statureMean, p.statureSigma, p.qv, p.footPxSigma, p.t1Sigma,
                         p.t2Sigma, p.cRoot, p.cLeg})
            if (!(v > 0))
                throw std::runtime_error("[Pedestrians] every sigma / constant must be > 0");
        std::cout << "[Pedestrians] on: root " << p.rootPxSigma << " px, log-range " << p.logDepthSigma << ", L/H " << p.skelOverStature
                  << ", stature " << p.statureMean << "+-" << p.statureSigma << " m, q_v " << p.qv << ", foot " << p.footPxSigma
                  << " px, T1 " << p.t1Sigma << ", T2 " << p.t2Sigma << ", c_root " << p.cRoot << ", c_leg " << p.cLeg
                  << ", min track KFs " << p.minTrackKeyframes << ", min stance box " << p.minStanceBoxPx << " px, min foot obs "
                  << p.minFootObservations << ", dir " << p.dir << std::endl;
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
            PedObs o;
            int cl, cr;
            ss >> tag >> o.trackId >> o.rootU >> o.rootV >> o.lpx >> o.maskH >> o.boxH >> cl >> cr >> o.toeU[0] >> o.toeV[0] >> o.toeU[1] >> o.toeV[1];
            if (tag != "P" || ss.fail())
                throw std::runtime_error("[Pedestrians] malformed line in " + path + ": " + line);
            if (o.trackId <= lastTrack)
                throw std::runtime_error("[Pedestrians] track ids must be strictly increasing in " + path);
            lastTrack = o.trackId;
            o.claim = {cl != 0, cr != 0};
            pf->obs.push_back(o);
        }
        return pf;
    }
} // namespace ORB_SLAM3
