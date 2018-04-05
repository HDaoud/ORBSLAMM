#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<string>
#include "Converter.h"

#include<opencv2/core/core.hpp>

using namespace std;

void ConvertToQuater(string filename);

int main(int argc, char** argv)
{
    ConvertToQuater(argv[1]);
}

void ConvertToQuater(string filename)
{
    ifstream f(filename);
    ofstream fout("Quat.txt");
    
    fout << fixed;
    
    cout << endl << "Convert trajectory to Quaternion ..." << endl;
    
    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            
            cv::Mat T(3,4, CV_32FC1);
            
            
            ss >> T.at<float>(0,0) >> T.at<float>(0,1)  >> T.at<float>(0,2) >> T.at<float>(0,3) >>
             T.at<float>(1,0) >> T.at<float>(1,1)  >> T.at<float>(1,2) >> T.at<float>(1,3) >>
                    T.at<float>(2,0) >> T.at<float>(2,1)  >> T.at<float>(2,2) >> T.at<float>(2,3);
            
            //cout<<T;
            
            cv::Mat R = T.rowRange(0,3).colRange(0,3).t();
            cv::Mat t = T.rowRange(0,3).col(3);
            
            vector<float> q = iORB_SLAM::Converter::toQuaternion(R);
            
            fout << setprecision(7) << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
    
    }
    
    f.close();
    fout.close();
    
}