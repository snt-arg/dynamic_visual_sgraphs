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
            else
                throw std::runtime_error("[Pedestrians] unknown row tag in " + path + ": " + line);
        }
        return pf;
    }
} // namespace ORB_SLAM3
