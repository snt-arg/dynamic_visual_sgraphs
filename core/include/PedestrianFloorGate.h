// P9-C3c: the gates of the midpoint floor contact as pure functions (used by PedestrianBA.cc and unit-tested on hand-made cases).
//   stage 2  raised surface: the regional z_f is more than FloorRaisedMaxM above the map's global lowest floor mode
//   stage 3  elevation: the foot ray is less than FloorMinElevDeg below the horizontal (this also rejects rays at or above the horizon)
//   stage 4  B': fewer than FloorNeighbours qualifying static points (within FloorBandM of the corrected z_f, within FloorNeighbourRadiusM of the contact)
//   stage 5  B': the qualifying points scatter in height by FloorPlaneSigmaMaxM or more
// Stage 1 (no regional z_f) is decided by the estimator itself. 0 = passed.
#ifndef PEDESTRIAN_FLOOR_GATE_H
#define PEDESTRIAN_FLOOR_GATE_H

#include "Pedestrians.h"

namespace ORB_SLAM3
{
    namespace ped
    {
        inline int FloorStageA(const PedestrianParams &P, double zfRaw, double zfGlobal, double elevDeg)
        {
            if (zfRaw - zfGlobal > P.floorRaisedMaxM)
                return 2;
            if (!(elevDeg >= P.floorMinElevDeg))
                return 3;
            return 0;
        }
        inline int FloorStageB(const PedestrianParams &P, int nQualifying, double sigmaQualifying)
        {
            if (nQualifying < P.floorNeighbours)
                return 4;
            if (!(sigmaQualifying < P.floorPlaneSigmaMaxM))
                return 5;
            return 0;
        }
    } // namespace ped
} // namespace ORB_SLAM3

#endif
