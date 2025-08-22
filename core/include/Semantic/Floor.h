/**
 * This file is part of Visual S-Graphs (vS-Graphs).
 * Copyright (C) 2023-2025 SnT, University of Luxembourg
 *
 * 📝 Authors: Ali Tourani, Saad Ejaz, Hriday Bavle, Jose Luis Sanchez-Lopez, and Holger Voos
 *
 * vS-Graphs is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation, either
 * version 3 of the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details: https://www.gnu.org/licenses/
 */

#ifndef FLOOR_H
#define FLOOR_H

#include "Map.h"
#include "Room.h"

namespace ORB_SLAM3
{
    class Room;

    class Floor
    {
    private:
        int id;                               // Floor's ID
        int opId;                             // Floor's ID in the local optimizer
        int opIdG;                            // Floor's ID in the global optimizer
        std::string name;                     // The name devoted for each room (optional)
        Eigen::Vector3d centroid;             // Floor's centroid in the global reference
        std::vector<ORB_SLAM3::Room *> rooms; // Floor's rooms and corridors

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Floor();
        ~Floor();

        int getId() const;
        void setId(int value);

        int getOpId() const;
        void setOpId(int value);

        int getOpIdG() const;
        void setOpIdG(int value);

        std::string getName() const;
        void setName(std::string value);

        Eigen::Vector3d getCentroid() const;
        void setCentroid(Eigen::Vector3d value);

        void addRoom(ORB_SLAM3::Room *value);
        std::vector<ORB_SLAM3::Room *> getRooms() const;
        void setRooms(const std::vector<ORB_SLAM3::Room *> &value);

        ORB_SLAM3::Map *getMap();
        void setMap(ORB_SLAM3::Map *pMap);

    protected:
        ORB_SLAM3::Map *mpMap;
        std::mutex mMutexMap;
    };
}

#endif