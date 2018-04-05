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
#include<iomanip>

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
    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_NewCollege path_to_vocabulary path_to_settings path_to_sequence [0|1]for_Multi_Maps_Usage" << endl;
        return 1;
    }


    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
    
    bool bUseMMaps = string(argv[4]).compare("1") == 0;

    
    vector<string>::iterator begin = vstrImageFilenames.begin();
    vector<string>::iterator end = vstrImageFilenames.begin() + vstrImageFilenames.size()/2;
    vector<string> vstrImgFN1(begin, end);
    vector<string> vstrImgFN2(end+1, vstrImageFilenames.end());
    
    vector<double>::iterator beginT = vTimestamps.begin();
    vector<double>::iterator endT = vTimestamps.begin() + vTimestamps.size()/2;
    vector<double> vTS1(beginT, endT);
    vector<double> vTS2(endT+1, vTimestamps.end());

    int nImages = vstrImageFilenames.size();
    int start1 = 0, end1 = (nImages/2)-200, start2= 0, end2=nImages/2;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    
    //Start time
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    
    //Create the multi-mapper
    iORB_SLAM::MultiMapper* pMMapper = new iORB_SLAM::MultiMapper();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
//    iORB_SLAM::System SLAM(pVocabulary,argv[2],iORB_SLAM::System::MONOCULAR,false,bUseMMaps);
    iORB_SLAM::System SLAM(argv[1],argv[2],iORB_SLAM::System::MONOCULAR,true,bUseMMaps);
    //Assign the multi-mapper
    SLAM.SetMultiMapper(pMMapper);
    
    //Run the thread (Robot1)
    thread Run(RunSLAM, ref(start1), ref(end1), ref(SLAM), ref(vstrImgFN1), ref(vTS1));
    
    
    //Create the second SLAM system
//    iORB_SLAM::System SLAM2(pVocabulary,argv[2],iORB_SLAM::System::MONOCULAR,true, bUseMMaps);
    iORB_SLAM::System SLAM2(argv[1],argv[2],iORB_SLAM::System::MONOCULAR,false,bUseMMaps);
    //Assign the mutli-mapper
    SLAM2.SetMultiMapper(pMMapper);  
    
    //Run the thread (Robot2)
    thread Run2(RunSLAM2, ref(start2), ref(end2), ref(SLAM2), ref(vstrImgFN2), ref(vTS2));
    
    //Run the Multi-mapper thread
    std::thread* ptMultiMapping = new thread(&iORB_SLAM::MultiMapper::Run, pMMapper);
        
    //
    

    Run.join();
    Run2.join();

    //pMMapper->SaveTrajectory("MMaps.txt");
    
    ptMultiMapping->join();
    
    //End time
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    
    //Duration
    double duration= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    
    cout<<"\n\nProgram takes "<<duration<<" to finish the sequence!\n";

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes,fNames;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    string strPathNamesFile = strPathToSequence + "/right/filenames.txt";
    fTimes.open(strPathTimeFile.c_str());
    fNames.open(strPathNamesFile.c_str());
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
        getline(fNames, s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            string name;
            ss >> name;
            vstrImageFilenames.push_back(strPathToSequence + "/right/"+name);
        }
    }
}

void RunSLAM(int& start, int& nImages, iORB_SLAM::System& SLAM, vector<string>& vstrImageFilenames, vector<double>& vTimestamps)
{
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    
    cout<<"\n\nRobot1 is processing "<<nImages<<" images\n\n";
    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
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

    // Save camera trajectory
    string date;
    struct timeval tv;
    struct tm* tm_info;
    
    gettimeofday(&tv,NULL);
    tm_info = localtime(&tv.tv_sec);
    
    char time_s[26];
    strftime(time_s, 26, "%Y:%m:%d %H:%M:%S", tm_info);
    
    date = time_s;
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_"+date+".txt");
}

void RunSLAM2(int& start, int& nImages, iORB_SLAM::System& SLAM, vector<string>& vstrImageFilenames, vector<double>& vTimestamps)
{
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    
    cout<<"\n\nRobot2 is processing "<<nImages<<" images\n\n";
    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
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

    // Save camera trajectory
    string date;
    struct timeval tv;
    struct tm* tm_info;
    
    gettimeofday(&tv,NULL);
    tm_info = localtime(&tv.tv_sec);
    
    char time_s[26];
    strftime(time_s, 26, "%Y:%m:%d %H:%M:%S", tm_info);
    
    date = time_s;
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_"+date+".txt");
}