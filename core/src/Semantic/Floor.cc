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

#include "Semantic/Floor.h"

namespace ORB_SLAM3
{
    Floor::Floor() {}
    Floor::~Floor() {}

    int Floor::getId() const
    {
        return id;
    }

    void Floor::setId(int value)
    {
        id = value;
    }

    int Floor::getOpId() const
    {
        return opId;
    }

    void Floor::setOpId(int value)
    {
        opId = value;
    }

    int Floor::getOpIdG() const
    {
        return opIdG;
    }

    void Floor::setOpIdG(int value)
    {
        opIdG = value;
    }

    std::string Floor::getName() const
    {
        return name;
    }

    void Floor::setName(std::string value)
    {
        name = value;
    }

    Eigen::Vector3d Floor::getCentroid() const
    {
        return centroid;
    }

    void Floor::setCentroid(Eigen::Vector3d value)
    {
        centroid = value;
    }

    std::vector<ORB_SLAM3::Room *> Floor::getRooms() const
    {
        return rooms;
    }

    void Floor::addRoom(ORB_SLAM3::Room *value)
    {
        rooms.push_back(value);
    }

    void Floor::setRooms(const std::vector<ORB_SLAM3::Room *> &value)
    {
        rooms = value;
    }

    ORB_SLAM3::Map *Floor::getMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void Floor::setMap(ORB_SLAM3::Map *pMap)
    {
        unique_lock<mutex> lock(mMutexMap);
        mpMap = pMap;
    }
}