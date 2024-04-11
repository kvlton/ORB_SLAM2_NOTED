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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:
    Frame();

    /// 复制帧
    Frame(const Frame &frame);

    /// 构造当前帧（双目，RGB-D，单目）
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    /// 调用ORB提取器的接口，提取当前帧的ORB特征
    void ExtractORB(int flag, const cv::Mat &im);

    /// 计算词袋向量
    void ComputeBoW();

    /// 设置当前帧的位姿
    void SetPose(cv::Mat Tcw);

    /// 根据位姿，更新当前帧的 R，t
    void UpdatePoseMatrices();

    /// 获取相机的光心位置，旋转矩阵的转置
    inline cv::Mat GetCameraCenter(){ return mOw.clone(); }
    inline cv::Mat GetRotationInverse(){ return mRwc.clone(); }

    /// 检查局部地图点，是否在当前帧的观测范围内（局部地图点，视角的余弦值）
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    /// 计算关键点在哪个格子
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    /// 搜索在(x,y)附近的 矩形范围内的 特征点（并且在固定的金字塔层数范围内）
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    /// 通过RGB-D图，计算双目模型
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    /// 将第i个特征点，反投影到3D世界坐标系
    cv::Mat UnprojectStereo(const int &i);

public:

    /// 成员变量
    ORBVocabulary* mpORBvocabulary;                         // ORB字典
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight; // ORB提取器（左图、右图）

    double mTimeStamp;  // 当前帧的时间戳

    // 相机内参
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    float mbf;          // b*f
    float mb;           // 基线b
    float mThDepth;     // 用来判断是否为近点的深度

    int N;              // 提取到的特征总数

    std::vector<cv::KeyPoint> mvKeys, mvKeysRight; // 关键点（左图，右图）
    std::vector<cv::KeyPoint> mvKeysUn;            // 关键点（左图去畸变）
    std::vector<float> mvuRight;                   // 右图关键点的X坐标
    std::vector<float> mvDepth;                    // 关键点的深度
    DBoW2::BowVector mBowVec;                      // 当前帧的词袋向量
    DBoW2::FeatureVector mFeatVec;                 // 当前帧的正向索引列表<子树，特征列表>
    cv::Mat mDescriptors, mDescriptorsRight;       // 描述子（左图，右图）

    std::vector<MapPoint*> mvpMapPoints;   // 地图点列表
    std::vector<bool> mvbOutlier;          // 是否为外点

    // 将关键点划分成小格子，便于特征匹配
    static float mfGridElementWidthInv;    // 格子数：列数
    static float mfGridElementHeightInv;   // 格子数：行数
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS]; // 每个格子里的特征列表

    cv::Mat mTcw;                      // 当前帧的位姿
    static long unsigned int nNextId;  // 给当前帧的ID赋值
    long unsigned int mnId;            // 当前帧的ID
    KeyFrame* mpReferenceKF;           // 当前帧的参考关键帧

    // 图像金字塔的参数
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // 去畸变后的图像边界
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    // 对于静态成员变量，只需要计算一遍即可
    static bool mbInitialComputations;


private:

    /// 关键点去畸变
    void UndistortKeyPoints();

    /// 计算去畸变后的图像边界
    void ComputeImageBounds(const cv::Mat &imLeft);

    /// 将关键点按格子划分，以减少特征匹配的时间
    void AssignFeaturesToGrid();

    // 通过当前帧的位姿得到
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw;  // twc，相机光心在世界坐标下的位置
};

}// namespace ORB_SLAM

#endif // FRAME_H
