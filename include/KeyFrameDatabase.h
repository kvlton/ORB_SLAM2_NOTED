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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>


namespace ORB_SLAM2
{

class KeyFrame;
class Frame;


class KeyFrameDatabase
{
public:

   /// 初始化关键帧数据库
   KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);   /// 添加关键帧
   void erase(KeyFrame* pKF); /// 删除关键帧
   void clear();              /// 清空关键帧数据库

   // Loop Detection
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   /// 寻找重定位的候选关键帧
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:

  const ORBVocabulary* mpVoc;
  std::vector<list<KeyFrame*> > mvInvertedFile; // 单词的关键帧列表
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
