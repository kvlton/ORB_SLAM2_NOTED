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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<unistd.h>

using namespace std;

/// 加载图片
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // 根据关联文件获取 RGB图、深度图子路径，以及图片时间戳
    vector<string> vstrImageFilenamesRGB;               // RGB图 子路径
    vector<string> vstrImageFilenamesD;                 // 深度图 子路径
    vector<double> vTimestamps;                         // 图片 时间戳
    string strAssociationFilename = string(argv[4]);    // GRB-D 关联文件
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // 检查 RGD图和深度图 的数量是否相等，并且不为空
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // 初始化SLAM系统 (ORB字典文件，参数配置文件，模式设置为RGB-D，可视化)
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);

    // 追踪时间统计
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // SLAM系统主循环，循环对每张RGB-D图进行追踪
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
        // 读取当前RGB图、深度图、时间戳
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        // 追踪前的系统时间
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // 对当前RGB-D图进行追踪
        SLAM.TrackRGBD(imRGB,imD,tframe);

        // 追踪后的系统时间
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        // 当前RGB-D图的追踪时间
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[ni]=ttrack;

        // 准备追踪下一帧图片，看成实时处理
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;  // 当前帧->下一帧 的接收时间间隔
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);        // 等待（时间间隔-当前帧处理时间）us，直到接收到下一帧
    }

    // 关闭所有的线程
    SLAM.Shutdown();

    // 追踪时间统计
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;  // 中位数
    cout << "mean tracking time: " << totaltime/nImages << endl;         // 平均时间

    // 保存相机的位姿
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");           // 每一帧的位姿
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt"); // 关键帧的位姿

    return 0;
}

// 加载图片
void LoadImages(const string &strAssociationFilename,    // 输入：GRB-D关联文件
                vector<string> &vstrImageFilenamesRGB,   // 输出：RGB图子路径
                vector<string> &vstrImageFilenamesD,     // 输出：深度图子路径
                vector<double> &vTimestamps)             // 输出：图片时间戳
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());   // 读取GRB-D关联文件
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);                    // 图片时间戳
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);       // RGB图子路径
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);           // 深度图子路径

        }
    }
}
