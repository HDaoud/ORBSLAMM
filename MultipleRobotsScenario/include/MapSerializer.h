/**
* This file is part of iORB-SLAM.
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
/* 
 * File:   MapSerializer.h
 * Author: hdaoud
 *
 * Created on November 21, 2016, 6:04 PM
 */

#ifndef MAPSERIALIZER_H
#define	MAPSERIALIZER_H

#include <fstream>
#include <vector>
#include <map>
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"

#include "tinyxml.h"
#include<opencv2/core/core.hpp>

namespace iORB_SLAM
{
    class Map;
    class KeyFrame;
    class MapPoint;
    
    #define MAP_XML_ID "iORB-SLAM_Map"
    #define MAP_VERSION "1.0"
    
class MapSerializer
{
    public:
        enum MapStatus {
            MAP_OK,
            MAP_FAILED,
            MAP_EXISTS
        };
        MapSerializer();
        MapSerializer(Map* map);
        MapSerializer(std::vector<Map*> maps);
        ~MapSerializer();
        
        bool Init();
    
        MapStatus LoadMap(Map* pMap, std::string sMapFileName);
        MapStatus SaveMap(Map* pMap, std::string folderName);

        MapStatus SaveMaps(std::vector<Map*> mvpMaps, std::string folderName);
        MapStatus LoadMaps(std::string folderName);

private:    
    bool _SaveAKeyFrame(KeyFrame* pKF, TiXmlElement* keyFramesNode );
    bool _SaveKeyFrames(TiXmlElement* rootNode );
    
    bool _SaveAMapPoint(MapPoint* pMP, TiXmlElement* mapPointsNode );
    bool _SaveMapPoints(TiXmlElement* rootNode );
    
    bool _LoadAKeyFrame(TiXmlHandle &hKF);
    bool _LoadKeyFrames(TiXmlHandle &hRoot);
    
    bool _LoadAMapPoint(TiXmlHandle &hMP);
    bool _LoadMapPoints(TiXmlHandle &hRoot);
    
    Map* mpMap;
    std::vector<Map*> mvpMaps;
    bool mbOK;
    //std::mutex mMutexMapSerializer;
        
        
};

}


#endif	/* MAPSERIALIZER_H */

