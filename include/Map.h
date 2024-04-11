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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class Map
{
public:

    /// 初始化地图
    Map();

    void AddKeyFrame(KeyFrame* pKF);    /// 在地图中，插入关键帧
    void AddMapPoint(MapPoint* pMP);    /// 在地图中，插入地图点
    void EraseMapPoint(MapPoint* pMP);  /// 从地图中，删除地图点
    void EraseKeyFrame(KeyFrame* pKF);  /// 从地图中，删除关键帧

    /// 设置参考地图点，仅用来显示
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    /// ？
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();       /// 获取所有关键帧
    std::vector<MapPoint*> GetAllMapPoints();       /// 获取所有地图点
    std::vector<MapPoint*> GetReferenceMapPoints(); /// 获取参考地图点（仅用于显示）
    long unsigned int MapPointsInMap();             /// 获取地图点的数量
    long unsigned  KeyFramesInMap();                /// 获取关键帧的数量
    long unsigned int GetMaxKFid();                 /// 获取最大关键帧ID

    /// 清空地图，用于重启系统
    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;  // 初始关键帧？
    std::mutex mMutexMapUpdate;
    std::mutex mMutexPointCreation;        // 防止地图点ID冲突（地图点可以由 跟踪线程 和 局部地图线程 两个线程插入）

protected:

    std::set<MapPoint*> mspMapPoints;             // 地图点集合
    std::set<KeyFrame*> mspKeyFrames;             // 关键帧集合
    std::vector<MapPoint*> mvpReferenceMapPoints; // 参考帧地图点（仅用于显示）
    long unsigned int mnMaxKFid;                  // 最大的关键帧ID

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM

#endif // MAP_H
