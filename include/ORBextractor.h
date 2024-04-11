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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

/// 四叉数节点类
class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    /// 一个节点，分裂为四个节点
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    std::vector<cv::KeyPoint> vKeys;          // 节点里的，角点列表
    cv::Point2i UL, UR, BL, BR;               // 节点的左上角、右上角、左下角、右下角
    std::list<ExtractorNode>::iterator lit;   // 节点的迭代器
    bool bNoMore;                             // 不能再分裂标志（节点里只有一个角点）
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    /// ORB提取器初始化，设置特征总数，比例因子，金字塔层数，初始阈值，更小阈值
    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    /// ORB特征提取API，根据输入的灰度图，得到关键点以及描述子
    void operator()( cv::InputArray image, cv::InputArray mask,
                     std::vector<cv::KeyPoint>& keypoints,
                     cv::OutputArray descriptors);

    /// 获取内部变量的接口
    int inline GetLevels(){
        return nlevels;}                                // 金字塔层数
    float inline GetScaleFactor(){
        return scaleFactor;}                            // 尺度因子
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;                           // 每一层的尺度因子
    }
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;                        // 每一层的逆尺度
    }
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;                           // 尺度的平方
    }
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;                        // 逆尺度的平方
    }
    std::vector<cv::Mat> mvImagePyramid;                //图像金字塔

protected:

    /// 获取图像金字塔
    void ComputePyramid(cv::Mat image);

    /// 提取关键点
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    /// 成员变量
    std::vector<cv::Point> pattern; // 点对模板，用于计算描述子

    int nfeatures;      // 特征总数
    double scaleFactor; // 尺度因子
    int nlevels;        // 金字塔层数
    int iniThFAST;      // FAST提取阈值 20%
    int minThFAST;      // 更小阈值 7%

    std::vector<int> mnFeaturesPerLevel;  // 每一层需要提取的特征数
    std::vector<int> umax;                // v对应的最大u，用于在一个圆内计算特征的旋照
    std::vector<float> mvScaleFactor;     // 每层的尺度因子
    std::vector<float> mvInvScaleFactor;  // 每层的逆尺度
    std::vector<float> mvLevelSigma2;     // 每层的尺度平方
    std::vector<float> mvInvLevelSigma2;  // 每层的逆尺度平方
};

} //namespace ORB_SLAM

#endif

