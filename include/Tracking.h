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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:

    /// 初始化跟踪
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    /// 跟踪的API入口（双目，RGB-D，单目）
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    /// 设置和其它线程的连接
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    /// 改变追踪模式（定位模式，综合模式）
    void InformOnlyTracking(const bool &flag);

public:

    // 追踪状态
    enum eTrackingState{
        SYSTEM_NOT_READY=-1, // 系统未准备就绪
        NO_IMAGES_YET=0,     // 没有接受到图片
        NOT_INITIALIZED=1,   // 未初始化
        OK=2,                // 追踪成功
        LOST=3               // 跟丢
    };

    /// 成员变量
    eTrackingState mState;                   // 当前帧的跟踪状态
    eTrackingState mLastProcessedState;      // 上一帧的跟踪结果
    int mSensor;                             // 传感器模式
    Frame mCurrentFrame;                     // 当前帧
    cv::Mat mImGray;                         // 灰度图（左图）

    // 单目的初始化参数？
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // 用来保存每一帧的跟踪结果
    list<cv::Mat> mlRelativeFramePoses; // 相对于参考关键帧的位姿
    list<KeyFrame*> mlpReferences;      // 参考关键帧
    list<double> mlFrameTimes;          // 时间戳
    list<bool> mlbLost;                 // 是否跟丢
    bool mbOnlyTracking;                // 当前帧的追踪模式

    /// 重启跟踪线程
    void Reset();

protected:

    /// 对当前帧进行跟踪（跟踪的主函数）
    void Track();

    /// 双目初始化（包括RGB-D）
    void StereoInitialization();

    /// 单目初始化
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    /// 检查并更新 最后一帧中的地图点
    void CheckReplacedInLastFrame();

    /// 参考帧模型进行跟踪
    bool TrackReferenceKeyFrame();

    /// 更新上一帧（更新位姿，添加地图点）
    void UpdateLastFrame();

    /// 运动模型进行跟踪
    bool TrackWithMotionModel();

    /// 重定位（三次匹配三次优化）
    bool Relocalization();

    /// 局部地图跟踪
    void UpdateLocalMap();       // 更新局部地图（局部关键帧，局部地图点）
    void UpdateLocalPoints();    // 更新局部地图的地图点
    void UpdateLocalKeyFrames(); // 更新局部地图的关键帧
    bool TrackLocalMap();        /// 局部地图跟踪
    void SearchLocalPoints();    /// 在局部地图中，查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配

    /// 插入关键帧
    bool NeedNewKeyFrame();   // 是否需要插入关键帧
    void CreateNewKeyFrame(); // 插入关键帧

    // 定位模式下使用，表示上一帧的跟踪效果不好，下一帧偏向于重定位
    bool mbVO;

    // 实例化
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;
    Initializer* mpInitializer;               // 单目初始化
    KeyFrame* mpReferenceKF;                  // 参考关键帧（局部地图跟踪会改变参考关键帧）
    std::vector<KeyFrame*> mvpLocalKeyFrames; // 局部地图关键帧（局部地图跟踪）
    std::vector<MapPoint*> mvpLocalMapPoints; // 局部地图地图点（局部地图跟踪）
    System* mpSystem;
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Map* mpMap;

    // 相机内参
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    // 阈值
    int mMinFrames;        // 插入关键帧的阈值（0）
    int mMaxFrames;        // 插入关键帧的阈值（fps）
    float mThDepth;        // 近点的深度阈值（近点认为是可信，可以单帧插入地图点，远点需要匹配插入）

    float mDepthMapFactor; // 深度图的比例因子

    // 当前帧优化后内点数（匹配到的特征点数）
    int mnMatchesInliers;

    KeyFrame* mpLastKeyFrame;        // 上一关键帧
    Frame mLastFrame;                // 上一帧
    unsigned int mnLastKeyFrameId;   // 上一关键帧ID
    unsigned int mnLastRelocFrameId; // 上一次重定位的ID

    // 运动模型的速度（前后两帧的相对位姿）
    cv::Mat mVelocity;

    // 颜色通道的顺序
    bool mbRGB;

    // 上一帧新添的地图点（用来在后面删除这些新添加的地图点）
    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
