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


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
public:

    // 传感器类型
    enum eSensor{
        MONOCULAR=0,  // 单目
        STEREO=1,     // 双目
        RGBD=2        // RGB-D
    };

public:

    /// SLAM系统初始化
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

    /// 单帧图片的跟踪API
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);  // 双目追踪
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);       // RGB-D追踪
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);                           // 单目追踪

    /// SLAM系统的一些API
    void ActivateLocalizationMode();   // 设置为定位模式
    void DeactivateLocalizationMode(); // 恢复 局部地图线程
    bool MapChanged();
    void Reset();                      // 重启系统
    void Shutdown();                   // 关闭系统，结束所有线程

    /// 保存相机的位姿
    void SaveTrajectoryTUM(const string &filename);         // 所有位姿
    void SaveKeyFrameTrajectoryTUM(const string &filename); // 关键帧位姿
    void SaveTrajectoryKITTI(const string &filename);       // 所有位姿

    /// 获取追踪状态（系统未准备就绪、没有接收到图片、未初始化、跟踪成功、跟丢）
    int GetTrackingState(); // -1,0,1,2,3

    /// 获取当前帧的地图点和关键点
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

private:

    /// 实例化对象
    eSensor mSensor;                      // 传感器类型
    ORBVocabulary* mpVocabulary;          // ORB字典
    KeyFrameDatabase* mpKeyFrameDatabase; // 关键帧数据库
    Map* mpMap;                           // 地图
    Tracking* mpTracker;                  // 追踪
    LocalMapping* mpLocalMapper;          // 局部地图
    LoopClosing* mpLoopCloser;            // 回环检测
    Viewer* mpViewer;                     // 可视化
    FrameDrawer* mpFrameDrawer;           // 显示当前帧
    MapDrawer* mpMapDrawer;               // 显示地图

    /// SLAM系统线程
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    /// 重启系统
    std::mutex mMutexReset;
    bool mbReset;

    /// 追踪模式（定位模式、综合模式）
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;   // 定位模式
    bool mbDeactivateLocalizationMode; // 恢复局部地图线程

    /// 追踪状态（系统未准备就绪、没有接收到图片、未初始化、跟踪成功、跟丢）
    std::mutex mMutexState;
    int mTrackingState;

    /// 当前帧的地图点和关键点
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;

};

}// namespace ORB_SLAM

#endif // SYSTEM_H
