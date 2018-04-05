/**
* This file is part of iORB-SLAM and is modified version of Map.h in ORB-SLAM2
*
* Copyright (C) 2016 Hayyan Daoud <hayyan dot d at gmail dot com> (University of Malaya)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* iORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* iORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Converter.h"
#include <set>

#include <mutex>



namespace iORB_SLAM
{
    
class MapPoint;
class KeyFrame;

class Map
{
public:
    Map(unsigned int id);
    Map(const Map &map);

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<KeyFrame*> MMGetAllKeyFrames();
    std::vector<MapPoint*> MMGetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<KeyFrame*> GetAllKeyFramesAfter(KeyFrame* pKF);

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    void setMaxKFid(long unsigned int Id);
    long unsigned int GetMaxKFid();
    long unsigned int GetMaxMPid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    
    unsigned int mnId; // For MultiMapper use
    unsigned int mnNxtId;
    Map* mpMatchedMap;
    KeyFrame* mpMatchingKF;//in the current map
    KeyFrame* mpMatchedKF; // in the matched map
    KeyFrame* mpLastKF;
    KeyFrame* mpFirstKF;
    
//    LocalMapping* mpLocalMapper; //This is used in Multi-robot scenario. It's initialized in tracking thread
//    LoopClosing* mpLoopCloser;
    
    //Use FileStorage to write to xml file instead of TinyXml
    void write(cv::FileStorage& fs) const;
    void read(const cv::FileNode& node);
    
    //Used for multimapping
    void attachToMap(Map* pMap, g2o::Sim3 relativePose);
    std::vector<Map*> getAttachedMaps();
    g2o::Sim3 relativePoseToAttachedMap(Map* pMap);
    
    bool isAttachedToMap(Map* pMap);
    bool isAttached();
    void printAttachedMaps();

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;
    
    //Relative poses between this map and matched maps
    std::map<Map*,g2o::Sim3> mRelativePoses;

    long unsigned int mnMaxKFid;
    long unsigned int mnMaxMPid;
    bool mbIsAttached;

    std::mutex mMutexMap;
};

} //namespace iORB_SLAM

#endif // MAP_H
