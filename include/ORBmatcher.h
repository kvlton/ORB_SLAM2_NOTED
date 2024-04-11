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


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"


namespace ORB_SLAM2
{

class ORBmatcher
{    
public:

    /// 初始化 ORB匹配器
    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    /// 计算两个ORB描述子之间的汉明距离
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    ///对当前帧视野范围内的局部地图点，通过投影进行特征匹配，用于局部地图跟踪（当前帧，局部地图点，扩大搜索范围的倍数）
    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3);

    /// 将上一帧的地图点投影到当前帧，用于跟踪上一帧（当前帧，上一帧，搜索范围，是否为单目）
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

    /// 对当前帧和候选帧，通过投影，进行特征匹配，用于重定位
    int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
     int SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th);

    /// 根据词袋，对当前帧和参考帧进行特征匹配（对参考帧的地图点进行匹配）
    int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);

    int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0);

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint);

public:

    static const int TH_LOW;       // 误匹配的阈值
    static const int TH_HIGH;
    static const int HISTO_LENGTH; // 角度直方图，用于剔除误匹配


protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

    /// 局部地图跟踪，投影匹配的范围
    float RadiusByViewingCos(const float &viewCos);

    /// 计算直方图中，频数最高的三个组
    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;          // 最优匹配和次优匹配的阈值（相差较大表示匹配较好）
    bool mbCheckOrientation;  // 是否通过特征点的旋转来剔除误匹配（所有的特征点的变化一致）
};

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
