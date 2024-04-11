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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

/// 声明静态成员变量
long unsigned int Frame::nNextId=0;                                            // 给当前帧的ID赋值
bool Frame::mbInitialComputations=true;                                        // 对于静态成员变量，只需要计算一遍即可
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;  // 相机的内参
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;              // 去畸变后的图像边界
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;             // 将关键点按照格子划分（格子数：行列数）

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

/// 构造当前帧（RGB-D）
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth,      // 灰度图，深度图
             const double &timeStamp, ORBextractor* extractor,   // 时间戳，提取器
             ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef,  // ORB字典，内参，畸变
             const float &bf, const float &thDepth)              // b*f，近点的深度
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // 当前帧的ID
    mnId=nNextId++;

    // 图像金字塔参数
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // 提取当前帧的ORB特征
    ExtractORB(0,imGray);

    // 提取到的关键点数量
    N = mvKeys.size();
    if(mvKeys.empty())
        return;

    // 对提取到的关键点，去畸变
    UndistortKeyPoints();

    // 通过RGB-D图，计算双目模型
    ComputeStereoFromRGBD(imDepth);

    // 初始化地图点列表
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false); //是否为外点标志

    // 只做一次
    if(mbInitialComputations)
    {
        // 计算去畸变后的图像边界
        ComputeImageBounds(imGray);

        // 划分格子的比例（格子数：行列数）
        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        // 相机内参
        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    // 基线b
    mb = mbf/fx;

    // 将关键点按格子划分，以减少特征匹配的时间
    AssignFeaturesToGrid();
}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

/// 将关键点按格子划分，以减少特征匹配的时间
void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))         // 计算当前关键点在哪个格子（可能不在任何一个格子里？）
            mGrid[nGridPosX][nGridPosY].push_back(i); // 并将关键点的索引加入到该格子中
    }
}

/// 调用ORB提取器的接口，提取当前帧的ORB特征
void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)  // 0为左图，1为右图
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

/// 设置当前帧的位姿
void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

/// 根据位姿，更新当前帧的 R，t
void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;  // twc，相机光心在世界坐标系中的位置
}

/// 检查局部地图点，是否在当前帧的观测范围内（局部地图点，视角的余弦值）
bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    // 预设为不能观测到
    pMP->mbTrackInView = false;

    // 3D世界坐标
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D相机坐标
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);
    if(PcZ<0.0f)
        return false;

    // 2D像素坐标
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;
    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // 检查到光心的距离，是否在尺度不变区域
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);
    if(dist<minDistance || dist>maxDistance)
        return false;

    // 检查角度是否在视野范围内
    cv::Mat Pn = pMP->GetNormal();         // 当前帧的平均视角
    const float viewCos = PO.dot(Pn)/dist; // 地图点的视角，与平均视角的余弦值
    if(viewCos<viewingCosLimit)
        return false; // 余弦值越小，角度越大

    // 预测地图点在当前帧 金字塔的层数
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // 设置地图点的属性（用于局部地图跟踪）
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;        // 右图的u
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

/// 搜索在(x,y)附近的 矩形范围内的 特征点（并且在固定的金字塔层数范围内）
vector<size_t> Frame::GetFeaturesInArea(
const float &x, const float  &y, const float  &r,  // 搜索范围
const int minLevel, const int maxLevel) const      // 金字塔层数（最低层，最高层）
{
    // 用来存放找到的特征点ID
    vector<size_t> vIndices;
    vIndices.reserve(N);

    // 搜索范围覆盖的小格子，在小格子内寻找关键点
    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)  // 最左边的格子
        return vIndices;
    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)                 // 最右边的格子
        return vIndices;
    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)  // 最上面的格子
        return vIndices;
    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)                 // 最下面的格子
        return vIndices;

    // 是否限定金字塔的层数
    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    // 遍历覆盖到的每一个格子
    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            // 当前格子包含的所有特征点
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            // 遍历格子内的每一个特征点
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                // 当前特征点
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];

                // 是否在金字塔层数范围内
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                // 是否在矩形搜索范围内
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;
                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    // 返回寻到的特征ID
    return vIndices;
}

/// 计算关键点在哪个格子
bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    // 因为关键点去畸变了，所以可能不在任何一个格子里？
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

/// 计算当前帧的词袋向量
void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

/// 关键点去畸变
void Frame::UndistortKeyPoints()
{
    // 如果相机的畸变系数为0,则不用去畸变
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // 关键点去畸变
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;  // 去畸变前的关键点列表
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK); // 调用OpenCV的接口，去畸变
    mat=mat.reshape(1);
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;                     // 去畸变后的关键点列表
    }
}

/// 计算去畸变后的图像边界
void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;                 // 左上角
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;         // 右上角
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;         // 左下角
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows; // 右下角

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));   // x的范围
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));   // y的范围
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        // 没有畸变的情况的
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}

/// 通过RGB-D图，计算双目模型
void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1); // 右图x坐标
    mvDepth = vector<float>(N,-1);  // 深度

    // 根据关键点及其深度，计算关键点在右图的x坐标
    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];    // 去畸变前
        const cv::KeyPoint &kpU = mvKeysUn[i]; // 去畸变后

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;                // 深度
            mvuRight[i] = kpU.pt.x-mbf/d;  // 右图x坐标
        }
    }
}

/// 将第i个特征点，反投影到3D世界坐标系
cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        // 相机坐标系
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        // 相机坐标系->世界坐标系
        return mRwc*x3Dc+mOw; // Rx+t
    }
    else
        return cv::Mat();
}

} //namespace ORB_SLAM
