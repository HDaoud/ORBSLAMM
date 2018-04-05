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
 * File:   xmlMapDrawer.h
 * Author: hdaoud
 *
 * Created on February 1, 2017, 1:43 AM
 */

#ifndef XMLMAPDRAWER_H
#define XMLMAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace iORB_SLAM
{

class xmlMapDrawer
{
public:
    xmlMapDrawer(Map* pMap, const string &strMapFile);

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:
    
    Map* mpMap;

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
};

} //namespace iORB_SLAM


#endif /* XMLMAPDRAWER_H */

