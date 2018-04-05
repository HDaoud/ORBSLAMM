/**
* This file is part of iORB-SLAM.
*
* Copyright (C) 2016-2017 Hayyan Daoud <hayyan dot d at gmail dot com> (University of Malaya)
* For more information see <https://github.com/hdaoud/ORBSLAMM>
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
* along with ORBSLAMM. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include "MultiMapper.h"

#include<opencv2/core/core.hpp>

#include"System.h"
#include<sys/time.h>
#include<thread>

using namespace std;


void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
void RunSLAM(int& start, int& nImages, iORB_SLAM::System& SLAM, 
        vector<string>& vstrImageFilenames, vector<double>& vTimestamps);
void RunSLAM2(int& start, int& nImages, iORB_SLAM::System& SLAM, 
        vector<string>& vstrImageFilenames, vector<double>& vTimestamps);


int main(int argc, char **argv)
{
    if(argc < 7)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings_of_sequence1 path_to_sequence1 [0|1]for_Multi_Maps_Usage path_to_sequence2 path_to_settings_of_sequence2" << endl;
        return 1;
    }


    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
    
    vector<string> vstrImageFilenames2;
    vector<double> vTimestamps2;
    if(argv[5])
        LoadImages(string(argv[5]), vstrImageFilenames2, vTimestamps2);
    
    bool bUseMMaps = string(argv[4]).compare("1") == 0;

    int nImages = vstrImageFilenames.size();
    
    int nImages2 = vstrImageFilenames2.size();
    
    int start1 = 0, start2= 0;
    
    //Start time
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    //Create the multi-mapper
    iORB_SLAM::MultiMapper* pMMapper = new iORB_SLAM::MultiMapper();
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    iORB_SLAM::System SLAM(argv[1],argv[2],iORB_SLAM::System::MONOCULAR,true, bUseMMaps);
    //Assign the multi-mapper
    SLAM.SetMultiMapper(pMMapper);
    
    //Run the thread (Robot1)
    //thread Run(RunSLAM, ref(start1), ref(end1), ref(SLAM), ref(vstrImgFN1), ref(vTS1));
    
    thread Run(RunSLAM, ref(start1), ref(nImages), ref(SLAM), ref(vstrImageFilenames), ref(vTimestamps));
    
    
    //Create the second SLAM system
    iORB_SLAM::System SLAM2(argv[1],argv[6],iORB_SLAM::System::MONOCULAR,false, bUseMMaps);
    //Assign the mutli-mapper
    SLAM2.SetMultiMapper(pMMapper);  
    
    //Run the thread (Robot2)
//    thread Run2(RunSLAM2, ref(start2), ref(end2), ref(SLAM2), ref(vstrImgFN2), ref(vTS2));
    
    thread Run2(RunSLAM2, ref(start2), ref(nImages2), ref(SLAM2), ref(vstrImageFilenames2), ref(vTimestamps2));
    
    //Run the Multi-mapper thread
    std::thread* ptMultiMapping = new thread(&iORB_SLAM::MultiMapper::Run, pMMapper);

    
    cout << endl << "-------" << endl;
    cout << "Start processing sequences ..." << endl;
    cout << "Images in the both sequences: " << nImages + nImages2 << endl << endl;

    
    
    
    
    Run.join();
    Run2.join();
    ptMultiMapping->join();
    
    //End time
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    
    //Duration
    double duration= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    
    cout<<"\n\nProgram took "<<duration<<" to finish both sequences!\n";

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}

void RunSLAM(int& start, int& nImages, iORB_SLAM::System& SLAM, vector<string>& vstrImageFilenames, vector<double>& vTimestamps)
{
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    
    // Main loop
    cv::Mat im;
    for(int ni=start; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    string date;
    struct timeval tv;
    struct tm* tm_info;
    
    gettimeofday(&tv,NULL);
    tm_info = localtime(&tv.tv_sec);
    
    char time_s[26];
    strftime(time_s, 26, "%Y:%m:%d %H:%M:%S", tm_info);
    
    date = time_s;
    
    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("KeyFrameTrajectory_kitti"+date+".txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory"+date+".txt");
    SLAM.SaveMultipleMapsTrajectories("Maps"+date+".txt");
}

void RunSLAM2(int& start, int& nImages, iORB_SLAM::System& SLAM, vector<string>& vstrImageFilenames, vector<double>& vTimestamps)
{
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    
    // Main loop
    cv::Mat im;
    for(int ni=start; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    string date;
    struct timeval tv;
    struct tm* tm_info;
    
    gettimeofday(&tv,NULL);
    tm_info = localtime(&tv.tv_sec);
    
    char time_s[26];
    strftime(time_s, 26, "%Y:%m:%d %H:%M:%S", tm_info);
    
    date = time_s;
    
    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("KeyFrameTrajectory_kitti"+date+".txt");
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory"+date+".txt");
    //SLAM.SaveMultipleMapsTrajectories("Maps"+date+".txt");
}
