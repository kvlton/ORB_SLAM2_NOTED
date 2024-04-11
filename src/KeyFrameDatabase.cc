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

#include "KeyFrameDatabase.h"

#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"

#include<mutex>

using namespace std;

namespace ORB_SLAM2
{

/// 初始化关键帧数据库
KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):  // 输入：ORB字典
    mpVoc(&voc)                         // ORB字典
{
    mvInvertedFile.resize(voc.size());  // 单词的关键帧列表
}

/// 添加关键帧
void KeyFrameDatabase::add(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutex);

    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF); // 单词的关键帧列表
}

/// 删除关键帧
void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutex);

    // 遍历关键帧的单词
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // 当前单词对应的关键帧列表
        list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

        // 将关键帧，从当前关键帧列表中删除
        for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

/// 清空关键帧数据库
void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}


vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)
{
    set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
    list<KeyFrame*> lKFsSharingWords;

    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    {
        unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;
                if(pKFi->mnLoopQuery!=pKF->mnId)
                {
                    pKFi->mnLoopWords=0;
                    if(!spConnectedKeyFrames.count(pKFi))
                    {
                        pKFi->mnLoopQuery=pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }
    }

    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lScoreAndMatch;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnLoopWords>maxCommonWords)
            maxCommonWords=(*lit)->mnLoopWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    int nscores=0;

    // Compute similarity score. Retain the matches whose score is higher than minScore
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;

        if(pKFi->mnLoopWords>minCommonWords)
        {
            nscores++;

            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

            pKFi->mLoopScore = si;
            if(si>=minScore)
                lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }

    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    list<pair<float,KeyFrame*> > lAccScoreAndMatch;
    float bestAccScore = minScore;

    // Lets now accumulate score by covisibility
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = it->first;
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
            {
                accScore+=pKF2->mLoopScore;
                if(pKF2->mLoopScore>bestScore)
                {
                    pBestKF=pKF2;
                    bestScore = pKF2->mLoopScore;
                }
            }
        }

        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;

    set<KeyFrame*> spAlreadyAddedKF;
    vector<KeyFrame*> vpLoopCandidates;
    vpLoopCandidates.reserve(lAccScoreAndMatch.size());

    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        if(it->first>minScoreToRetain)
        {
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpLoopCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }


    return vpLoopCandidates;
}

/// 寻找重定位的候选关键帧
vector<KeyFrame*> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F)
{
    // 寻找和当前帧存在相同单词的关键帧
    list<KeyFrame*> lKFsSharingWords;
    {
        unique_lock<mutex> lock(mMutex);

        // 遍历当前帧的所有单词
        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            // 存在当前单词的所有关键帧
            list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];
            for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                KeyFrame* pKFi=*lit;

                // 关键帧还未与当前帧关联的话，进行关联
                if(pKFi->mnRelocQuery!=F->mnId)
                {
                    pKFi->mnRelocWords=0;
                    pKFi->mnRelocQuery=F->mnId;
                    lKFsSharingWords.push_back(pKFi);
                }

                // 关键帧与当前帧关联的单词数
                pKFi->mnRelocWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return vector<KeyFrame*>();

    // 将最多的关联单词数*0.8,作为最低阈值
    int maxCommonWords=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if((*lit)->mnRelocWords>maxCommonWords)
            maxCommonWords=(*lit)->mnRelocWords;
    }
    int minCommonWords = maxCommonWords*0.8f;

    // 对超过阈值的关联帧，进行相似性评分（打分越高，越相似）
    list<pair<float,KeyFrame*> > lScoreAndMatch;
    int nscores=0;
    for(list<KeyFrame*>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        if(pKFi->mnRelocWords>minCommonWords)
        {
            nscores++;
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec); // 词袋向量是经过归一化的，使用L1范数评分
            pKFi->mRelocScore=si;
            lScoreAndMatch.push_back(make_pair(si,pKFi));
        }
    }
    if(lScoreAndMatch.empty())
        return vector<KeyFrame*>();

    // 将这些关联帧分组（共视），计算每个组的总分及其最高分
    list<pair<float,KeyFrame*> > lAccScoreAndMatch; // <当前组的总分，分数最高的关键帧>
    float bestAccScore = 0;          // 最高的组分
    for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        // 与当前关键帧，有共视的10个关键帧，看成一个组
        KeyFrame* pKFi = it->second;
        vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        // 统计当前组的（相似性总分，分数最高的关键帧）
        float bestScore = it->first; // 当前组的最高分
        float accScore = bestScore;  // 当前组的总分
        KeyFrame* pBestKF = pKFi;
        for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        {
            // 组内的关键帧不一定都与当前帧有关联
            KeyFrame* pKF2 = *vit;
            if(pKF2->mnRelocQuery!=F->mnId)
                continue;
            accScore+=pKF2->mRelocScore;    // 只统计有关联的

            // 组内分数最高的关键帧
            if(pKF2->mRelocScore>bestScore)
            {
                pBestKF=pKF2;
                bestScore = pKF2->mRelocScore;
            }

        }
        lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));

        // 总分最高的组
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // 将最高总分*0.75，作为阈值，筛选出总分较高的组，将组内分数最高的关键帧作为候选帧返回
    float minScoreToRetain = 0.75f*bestAccScore;
    set<KeyFrame*> spAlreadyAddedKF;      // 已经是候选帧的关键帧列表
    vector<KeyFrame*> vpRelocCandidates;  // 候选帧列表
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        // 总分超过阈值的组
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            // 组内分数最高的关键帧，作为候选帧
            KeyFrame* pKFi = it->second;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    // 返回候选帧列表
    return vpRelocCandidates;
}

} //namespace ORB_SLAM
