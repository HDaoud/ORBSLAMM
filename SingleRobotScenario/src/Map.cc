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

#include "Map.h"
#include "Sim3Solver.h"

#include<mutex>

namespace iORB_SLAM
{

Map::Map(unsigned int Id = 0):mnMaxKFid(0),mnMaxMPid(0),mbIsAttached(false)
{
    mnId = Id;
    mnNxtId = Id + 1;
}


void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    mpLastKF = pKF;
    
    if(!mpFirstKF)
        mpFirstKF = pKF;
       
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;    
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);

    mspMapPoints.insert(pMP);
    if(pMP->mnId > mnMaxMPid)
        mnMaxMPid = pMP->mnId;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<KeyFrame*> Map::MMGetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    vector<KeyFrame*> vpKFs = vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
    if(this->isAttached())
    {
        vector<Map*> vpAttachedMaps = this->getAttachedMaps();
        for(std::vector<Map*>::iterator it = vpAttachedMaps.begin(), itend = vpAttachedMaps.end(); it != itend; it++)
        {
            Map* pmMap = *it;
            vector<KeyFrame*> vpAttachedKFs = pmMap->GetAllKeyFrames();            
            vpKFs.insert(vpKFs.end(), vpAttachedKFs.begin(), vpAttachedKFs.end());            
        }
    }
    return vpKFs;
}

std::vector<KeyFrame*> Map::GetAllKeyFramesAfter(KeyFrame* pKF)
{
    std::vector<KeyFrame*> vKeyFrames = GetAllKeyFrames();
    std::vector<KeyFrame*> vKeyFramesResult;
    for(std::vector<KeyFrame*>::iterator KFit = vKeyFrames.begin(), KFitEnd=vKeyFrames.end(); KFit != KFitEnd; KFit++)
    {
        KeyFrame* pKFi = *KFit;
        if(pKFi->mnId > pKF->mnId)
        {
            vKeyFramesResult.push_back(pKFi);
        }
    }
    
    return vKeyFramesResult;

}


vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

vector<MapPoint*> Map::MMGetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    vector<MapPoint*> vpMP = vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
    if(this->isAttached())
    {
        vector<Map*> vpAttachedMaps = this->getAttachedMaps();
        for(std::vector<Map*>::iterator it = vpAttachedMaps.begin(), itend = vpAttachedMaps.end(); it != itend; it++)
        {
            Map* pmMap = *it;
           
            vector<MapPoint*> vpAttachedMPs = pmMap->GetAllMapPoints();
            
            vpMP.insert(vpMP.end(), vpAttachedMPs.begin(), vpAttachedMPs.end());
        }
    }

return vpMP;
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

void Map::setMaxKFid(long unsigned int Id)
{
    unique_lock<mutex> lock(mMutexMap);
    //cout<<"Setting Map"<<this->mnId<<" Max KFID to "<<Id;
    mnMaxKFid = Id;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

long unsigned int Map::GetMaxMPid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxMPid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::attachToMap(Map* pMap, g2o::Sim3 Pose)
{
    mbIsAttached = true;
    mRelativePoses[pMap] = Pose;
        
}

bool Map::isAttachedToMap(Map* pMap)
{
    return mRelativePoses.count(pMap);
}

bool Map::isAttached()
{
    return mbIsAttached;
}

std::vector<Map*> Map::getAttachedMaps() 
{
    std::vector<Map*> vMaps;
    
    for(std::map<Map*, g2o::Sim3>::iterator it = mRelativePoses.begin(), itend=mRelativePoses.end(); it!=itend; it++)
        vMaps.push_back(it->first);
    
    return vMaps;
}

g2o::Sim3 Map::relativePoseToAttachedMap(Map* pMap)
{
    if(this->isAttachedToMap(pMap))
        return mRelativePoses[pMap];
    
    return g2o::Sim3();
}



void Map::printAttachedMaps()
{
    cout<<"Map"<<this->mnId<<" relative poses: \n";
    for(std::map<Map*, g2o::Sim3>::iterator it = mRelativePoses.begin(), itend=mRelativePoses.end(); it!=itend; it++)
    {
        Map* pMap = it->first;
        cv::Mat pose = Converter::toCvMat(it->second);
        cout<<pMap->mnId<<endl;
        cout<<pose<<endl;
        cout<<"-----\n";
    }
    cout<<"---------------------------------\n";
}

//Use FileStorage to write Maps to xml instead of TinyXml
//void Map::write(cv::FileStorage& fs) const
//{
//    fs<< "{" << "KeyFrames";
//    //write the mspKeyFrames 
//    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
//    {
//        KeyFrame* pKFi=*sit;
//        pKFi.write(fs);
//    }
//
//    fs<< "MapPoints";
//    //write mspMapPoints
//    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
//    {
//        MapPoint* pMPi = *sit;
//        pMPi.write(fs);
//    }
//
//    fs<< "}";
//}


} //namespace iORB_SLAM
