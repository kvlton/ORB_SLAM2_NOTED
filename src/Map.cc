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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

/// 初始化地图
Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

/// 在地图中，插入关键帧
void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF); // 插入关键帧
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;  // 最大的关键帧ID
}

/// 在地图中，插入地图点
void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

/// 从地图中，删除地图点
void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
}

/// 从地图中，删除关键帧
void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
}

/// 设置参考地图点，仅用来显示
void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

/// 获取所有关键帧
vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

/// 获取所有地图点
vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

/// 获取地图点的数量
long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

/// 获取关键帧的数量
long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

/// 获取参考地图点（仅用于显示）
vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

/// 获取最大关键帧ID
long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

/// 清空地图，重启系统
void Map::clear()
{
    // 删除所有的地图点
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    // 删除所有的关键帧
    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();          // 清空地图点列表
    mspKeyFrames.clear();          // 清空关键帧列表
    mnMaxKFid = 0;                 // 最大的关键帧ID
    mvpReferenceMapPoints.clear(); // 参考帧地图点
    mvpKeyFrameOrigins.clear();    // 初始关键帧？
}

} //namespace ORB_SLAM
