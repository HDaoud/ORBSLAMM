/**
* 
*
* Copyright (C) 2016 Hayyan Daoud <hayyan dot d at gmail dot come> (University of Malaya)
* For more information see <https://github.com/hdaoud/ORBSLAMM>
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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/opencv.hpp>


#include"System.h"

using namespace std;


int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_Bebop path_to_vocabulary path_to_settings V4L_Device_number [0|1]for_Multi_Maps_Usage" << endl;
        return 1;
    }

    bool bUseMMaps = string(argv[4]).compare("1") == 0;
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    iORB_SLAM::System SLAM(argv[1],argv[2],iORB_SLAM::System::MONOCULAR,true,bUseMMaps);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    int MAX_SIZE = 1000;
    vTimesTrack.resize(MAX_SIZE);

    cout << endl << "-------" << endl;
    cout << "  VideoSource_Linux: Opening video source..." << endl;
    
    cv::VideoCapture cap(atoi(argv[3]));
    if(!cap.isOpened())
    {
        cout<<"Couldn't open the video stream!"<<endl;
        return -1;
    }
    
    double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    int dFramerate = cap.get(CV_CAP_PROP_FPS);
    
    cout<<"Frame size: "<<dWidth<<"x"<<dHeight<<endl;
    cout<<"Frame rate: "<<dFramerate<<endl;

    cout << "  ... got video source." << endl;

    // Main loop
    cv::Mat im;
    //int ni = 0;
    
    while(true)
    {
        // Read image from v4l device
        bool bSuccess = cap.read(im);
        auto t = std::chrono::high_resolution_clock::now();
        
        if(!bSuccess || im.empty())
        {
            cout<<"Couldn't read video frame!"<<endl;
            break;
        }
        
        double tframe = t.time_since_epoch().count();


//#ifdef COMPILEDWITHC11
//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//#else
//        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
//#endif

        // Pass the image to the SLAM system
        
        cv::Mat im_gray;
        cv::cvtColor(im, im_gray, cv::COLOR_RGB2GRAY);
        SLAM.TrackMonocular(im_gray,tframe);

//#ifdef COMPILEDWITHC11
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//#else
//        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
//#endif

        //double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        

//        vTimesTrack[ni]=ttrack;
//        ni++;
//        if(ni++ > MAX_SIZE)
//        {
//            MAX_SIZE = ni + 1000;
//            vTimesTrack.resize(MAX_SIZE);
//        }

        // Wait to load the next frame
//        double T=0;
//        if(ni<nImages-1)
//            T = vTimestamps[ni+1]-tframe;
//        else if(ni>0)
//            T = tframe-vTimestamps[ni-1];
//
//        if(ttrack<T)
//            usleep((T-ttrack)*1e6);
        
        if(cv::waitKey(100)==27)
        {
            cout<<"Closing..."<<endl;
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
//    vTimesTrack.resize(ni);
//    sort(vTimesTrack.begin(),vTimesTrack.end());
//    float totaltime = 0;
//    for(int i=0; i<ni; i++)
//    {
//        totaltime+=vTimesTrack[i];
//    }
//    cout << "-------" << endl << endl;
//    cout << "median tracking time: " << vTimesTrack[ni/2] << endl;
//    cout << "mean tracking time: " << totaltime/ni << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

