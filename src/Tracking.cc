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


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<unistd.h>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

/// 初始化跟踪
Tracking::Tracking(System *pSys, ORBVocabulary* pVoc,                 // SLAM系统，ORB字典
                   FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,  // 显示相机位姿，显示地图
                   Map *pMap, KeyFrameDatabase* pKFDB,                // 地图，关键帧数据库
                   const string &strSettingPath, const int sensor):   // 配置文件、传感器类型
                mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
                mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
                mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // 读取相机内参
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    // 读取相机畸变
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    // 基线*fx
    mbf = fSettings["Camera.bf"];

    // 30帧/s
    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // 插入关键帧、检查重定位 的帧数间隔
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;

    // 按照RGB，还是BGR的顺序
    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // ORB特征提取参数
    int nFeatures = fSettings["ORBextractor.nFeatures"];         // 待提取的特征总数
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];  // 比例因子
    int nLevels = fSettings["ORBextractor.nLevels"];             // 金字塔层数
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];        // 提取FAST角点的阈值
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];        // 提取FAST角点的更小阈值（提取失败时使用）
    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    if(sensor==System::MONOCULAR)                                // 单目初始化需要提取的特征数翻倍
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    // 用来判断是否为近点的深度
    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx; // 3.09米
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    // 深度图的尺度因子（倒数）
    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor; // 取倒数
    }
}

/// 设置和其它线程的连接
void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}
void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}
void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

/// 接收双目图，并进行跟踪
cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    // 读取左右图，并灰度化
    mImGray = imRectLeft;              // 左图
    cv::Mat imGrayRight = imRectRight; // 右图
    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    // 构造当前帧（双目）
    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    // 对当前帧进行跟踪
    Track();

    // 返回当前帧的位姿
    return mCurrentFrame.mTcw.clone();
}

/// 接收RGB-D图，并进行跟踪
cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    // 读取RGB图、深度图
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    // RGB图 灰度化
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    // 深度图，比例缩放到实际的单位（m）
    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor); // 尺度因子已经取了倒数

    // 构造当前帧（RGB-D）
    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    // 对当前帧进行跟踪
    Track();

    // 返回当前帧的位姿
    return mCurrentFrame.mTcw.clone();
}

/// 接收单目图，并进行跟踪
cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    // 读取彩色图，并灰度化
    mImGray = im;
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    // 构造当前帧（单目），单目初始化需要提取的特征数翻倍
    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    // 对当前帧进行跟踪
    Track();

    // 返回当前帧的位姿
    return mCurrentFrame.mTcw.clone();
}

/// 对当前帧进行跟踪（跟踪的主函数）
void Tracking::Track()
{
    // 如果图像复位过、或者第一次运行，则为NO_IMAGE_YET状态
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    // 保存上一帧的跟踪状态
    mLastProcessedState=mState;

    // 会在地图中插入关键帧
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    // 初始化
    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();     // 双目初始化（RGB-D）
        else
            MonocularInitialization();  // 单目初始化

        mpFrameDrawer->Update(this);

        if(mState!=OK)                  // 初始化成功后，状态为OK
            return;
    }
    else
    {
        /// 开始跟踪
        bool bOK;

        // 定位模式，还是综合模式
        if(!mbOnlyTracking)
        {
            /// 综合模式
            if(mState==OK)
            {
                // 检查并更新 上一帧中的地图点
                CheckReplacedInLastFrame();

                // 速度为空，或者刚完成重定位
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    /// 参考帧模型进行跟踪
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    /// 运动模型进行跟踪
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                /// 重定位
                bOK = Relocalization();
            }
        }
        else
        {
            /// 定位模式
            if(mState==LOST)
            {
                /// 重定位
                bOK = Relocalization();
            }
            else
            {
                // 在上一帧是否，没有跟踪到了足够的地图点
                if(!mbVO)
                {
                    /// 跟踪到了足够多的地图点

                    if(!mVelocity.empty())
                    {
                        /// 运动模型进行跟踪
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        /// 参考帧模型进行跟踪
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    /// 未跟踪到足够多的地图点，跟踪效果不好
                    /// 运动模型跟踪+重定位（二者有一个成功即可，两者都成功的话选择重定位的结果）

                    bool bOKMM = false;
                    bool bOKReloc = false;

                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        /// 运动模型跟踪
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }

                    /// 重定位
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        /// 仅仅运动模型跟踪成功
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        /// 未跟踪到足够的地图点，跟踪效果不好
                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    // 将当前帧的检测到的地图点的查找次数增加
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        /// 重定位成功
                        mbVO = false; // 跟踪效果设置为好
                    }

                    // 运动模型和重定位，成功一个即可
                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        // 设置当前帧的参考帧
        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        /// 局部地图跟踪
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        // 当前帧的跟踪状态
        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // 显示当前帧
        mpFrameDrawer->Update(this);

        /// 跟踪成功
        if(bOK)
        {
            // 更新运动模型
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            // 显示当前帧位姿
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // 清理VO匹配？
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // 删除临时地图点
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            /// 插入关键帧
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // 剔除外点
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // 如果刚初始化完没多久就跟丢，则重启系统
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        // 设置当前帧的参考关键帧
        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // 保存当前帧
        mLastFrame = Frame(mCurrentFrame);
    }

    /// 保存当前帧的位姿
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // 跟丢时，会出现这种情况
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}

/// 双目初始化（包括RGB-D）
void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // 设置初始位姿
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // 创建关键帧
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // 在地图中插入关键帧
        mpMap->AddKeyFrame(pKFini);

        // 创建地图点，并建立和关键帧的关系
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);    // 反投影到3D世界坐标系
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap); // 构建新的地图点
                pNewMP->AddObservation(pKFini,i);                  // 添加能 观测到该地图点 的关键帧
                pKFini->AddMapPoint(pNewMP,i);                     // 在关键帧上添加地图点
                pNewMP->ComputeDistinctiveDescriptors();           // 从众多观测到该地图点的特征点中挑选区分度最高的描述子
                pNewMP->UpdateNormalAndDepth();                    // 更新平均观测方向以及观测距离
                mpMap->AddMapPoint(pNewMP);                        // 将地图点加入到地图
                mCurrentFrame.mvpMapPoints[i]=pNewMP;              // 将地图点加入到当前帧
            }
        }
        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;


        mpLocalMapper->InsertKeyFrame(pKFini);          // 在局部地图中插入关键帧
        mLastFrame = Frame(mCurrentFrame);              // 上一普通帧
        mnLastKeyFrameId=mCurrentFrame.mnId;            // 上一关键帧对应普通帧的ID
        mpLastKeyFrame = pKFini;                        // 上一关键帧
        mvpLocalKeyFrames.push_back(pKFini);            // 在局部地图的关键帧
        mvpLocalMapPoints=mpMap->GetAllMapPoints();     // 在局部地图的地图点
        mpReferenceKF = pKFini;                         // 跟踪的参考帧
        mCurrentFrame.mpReferenceKF = pKFini;           // 当前帧的参考帧
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);// 将当前局部地图点，作为整个地图的参考地图点，用于显示
        mpMap->mvpKeyFrameOrigins.push_back(pKFini);    // 将关键帧，加入地图的原始关键帧
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw); // 将当前帧加入地图观测器

        mState=OK; // 初始化完成后，状态设置为OK
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

/// 检查并更新 最后一帧中的地图点
void Tracking::CheckReplacedInLastFrame()
{
    // 遍历上一帧加入的地图点
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

/// 参考帧模型进行跟踪
bool Tracking::TrackReferenceKeyFrame()
{
    // 计算当前帧的词袋向量
    mCurrentFrame.ComputeBoW();

    // 根据词袋，对当前帧和参考帧进行特征匹配
    ORBmatcher matcher(0.7,true);        // 最优匹配和次优匹配的阈值（相差较大表示匹配较好），是否根据特征角度的变化来剔除误匹配
    vector<MapPoint*> vpMapPointMatches;
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
    if(nmatches<15)
        return false;  // 匹配的特征点数少于15个，表示跟踪失败

    // 将匹配点设置为当前帧的地图点
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;

    // 上一帧的位姿设置为当前帧的位姿，再对当前帧的位姿进行优化
    mCurrentFrame.SetPose(mLastFrame.mTcw);
    Optimizer::PoseOptimization(&mCurrentFrame);

    // 剔除外点
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                // 将当前外点（地图点），从当前帧剔除
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;                // 地图点不在跟踪范围内了
                pMP->mnLastFrameSeen = mCurrentFrame.mnId; // 地图点最后一次被观测到的帧

                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++; // 内点（地图点），也被其他帧观测到了，才表示成功匹配
        }
    }

    // 跟踪10个点以上，才表示跟踪成功
    return nmatchesMap>=10;
}

/// 更新上一帧（更新位姿，添加地图点）
void Tracking::UpdateLastFrame()
{
    // 确定上一帧的位姿
    KeyFrame* pRef = mLastFrame.mpReferenceKF; // 上一帧的关键帧
    cv::Mat Tlr = mlRelativeFramePoses.back(); // 上一帧的相对位姿
    mLastFrame.SetPose(Tlr*pRef->GetPose());   // 上一帧的位姿（相对位姿*关键帧位姿）

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // 根据深度，对上一帧的关键点排序
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }
    if(vDepthIdx.empty())
        return;
    sort(vDepthIdx.begin(),vDepthIdx.end());

    // 将上一帧中，最近的100个（在探测范围内）的关键点 都插入到地图中
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        // 将不在地图中的关键点，插入到地图点中
        bool bCreateNew = false;
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }
        if(bCreateNew)
        {
            // 将当前关键点，插入到地图中
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);
            mLastFrame.mvpMapPoints[i]=pNewMP;
            mlpTemporalPoints.push_back(pNewMP); // 上一帧新添的地图点（用来在后面删除这些新添加的地图点）
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        // 前100个，并且在探测范围内
        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

/// 运动模型进行跟踪
bool Tracking::TrackWithMotionModel()
{
    // 构造匹配器
    ORBmatcher matcher(0.9,true);

    // 更新上一帧（位姿，添加地图点）
    UpdateLastFrame();

    // 恒速模型，确定当前帧的位姿
    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    // 通过投影，进行特征匹配
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
    int th;
    if(mSensor!=System::STEREO)
        th=15; // 双目的搜索范围更大
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // 如果匹配到的特征过少，扩大搜索范围，清空列表，重新进行特征匹配
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    // 跟踪失败
    if(nmatches<20)
        return false;

    // 对当前帧的位姿进行优化
    Optimizer::PoseOptimization(&mCurrentFrame);

    // 剔除外点
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                // 将当前外点（地图点），从当前帧剔除
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;                // 地图点不在跟踪范围内了
                pMP->mnLastFrameSeen = mCurrentFrame.mnId; // 地图点最后一次被观测到的帧
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++; // 内点（地图点），也被其他帧观测到了，才表示成功匹配
        }
    }    

    // 返回跟踪状态
    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10; // 没有跟踪到足够的地图点
        return nmatches>20;    // 定位模式需要匹配20个点以上
    }
    return nmatchesMap>=10;    // 匹配10个点以上，才表示跟踪成功
}

/// 局部地图跟踪
bool Tracking::TrackLocalMap()
{
    // 更新局部地图（局部关键帧，局部地图点）
    UpdateLocalMap();

    // 局部地图点，与当前帧的特征点，通过投影进行特征匹配
    SearchLocalPoints();

    // 优化当前的位姿
    Optimizer::PoseOptimization(&mCurrentFrame);

    // 更新内点和外点
    mnMatchesInliers = 0;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                // 内点
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)

                //外点
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // 刚刚重定位完，内点数少于50
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;  // 跟踪失败
    // 内点数少于30
    if(mnMatchesInliers<30)
        return false;  // 跟踪失败
    else
        return true;
}

/// 是否需要插入关键帧
bool Tracking::NeedNewKeyFrame()
{
    /// 不需要插入关键帧的情况

    // 是否为纯跟踪模式
    if(mbOnlyTracking)
        return false;

    // 局部地图已停止，或者被请求暂停
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    // 重定位不久，并且关键帧的数量比较多
    const int nKFs = mpMap->KeyFramesInMap();
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    /// 需要插入关键帧的情况

    // 参考帧中，被观察到2-3次以上的地图点数量
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // 局部地图是否空闲
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // 计算当前帧的近点数（未被跟踪，被跟踪上）
    int nNonTrackedClose = 0; // 未被跟踪的近点数
    int nTrackedClose= 0;     // 被跟踪上的近点数
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            // 在观测范围的特征点（近点）
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    // 是否要插入近点
    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // 内点数量的阈值
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;
    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // 1.当前帧距离上一关键帧是否相隔30帧以上
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // 2.当前帧距离上一关键帧是否相隔0帧以上 && 局部地图空闲
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    // 3.不是单目 && （ 内点数少 || 插入近点）
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // 4.内点数>15 && ( 内点数少 || 插入近点)
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    // 根据上面四个条件，判断是否需要插入关键帧
    if((c1a||c1b||c1c)&&c2)
    {
        // 局部地图是否空闲
        if(bLocalMappingIdle)
        {
            return true;           /// 需要插入
        }
        else
        {
            // 在局部地图线程中，中断BA
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                // 队列中的关键帧少于3帧的话，也需要插入关键帧
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;   /// 需要插入
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

/// 创建关键帧，并插入到局部地图线程
void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    // 将当前帧构造成关键帧
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    // 将当前帧设置为参考关键帧
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    // 对于双目或RGB-D摄像头，为当前帧生成新的地图点
    if(mSensor!=System::MONOCULAR)
    {
        // 更新当前帧的R，t
        mCurrentFrame.UpdatePoseMatrices();

        // 获取特征点的深度
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }
        if(!vDepthIdx.empty())
        {
            // 根据深度对特征点排序
            sort(vDepthIdx.begin(),vDepthIdx.end());

            // 将前100个近点插入到地图中
            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                // 当前特征点，是否要插入到地图
                int i = vDepthIdx[j].second;
                bool bCreateNew = false;
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    // 将特征点，插入到地图，并与当前关键帧建立联系
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                // 只插入前100个近点
                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    // 将当前关键帧，插入局部地图线程
    mpLocalMapper->InsertKeyFrame(pKF);
    mpLocalMapper->SetNotStop(false);

    // 更新关键帧
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

/// 在局部地图中，查找在当前帧视野范围内的点，将视野范围内的点和当前帧的特征点进行投影匹配
void Tracking::SearchLocalPoints()
{
    // 遍历当前帧的地图点，标记这些地图点不参与之后的搜索
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    // 判断局部地图点 是否在当前帧的视野范围内
    int nToMatch=0;
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        if(mCurrentFrame.isInFrustum(pMP,0.5)) // 是否在视野范围内，平均视角的60度范围
        {
            pMP->IncreaseVisible();            // 观测到该地图点的帧数+1
            nToMatch++;
        }
    }

    // 对视野范围内的局部地图点，通过投影，进行特征匹配
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;  // 刚重定位完，扩大搜索范围范围
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

/// 更新局部地图
void Tracking::UpdateLocalMap()
{
    // 用于显示局部地图点
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // 更新局部地图
    UpdateLocalKeyFrames(); // 更新关键帧
    UpdateLocalPoints();    // 更新地图点
}

/// 更新局部地图的地图点
void Tracking::UpdateLocalPoints()
{
    // 清空局部地图
    mvpLocalMapPoints.clear();

    // 遍历局部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        // 将局部关键帧的地图点，放入局部地图
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;  // 已经加入到局部地图
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

/// 更新局部地图的关键帧
void Tracking::UpdateLocalKeyFrames()
{
    /// 遍历当前帧的地图点，记录所有能观测到当前帧地图点的关键帧
    map<KeyFrame*,int> keyframeCounter; // 关键帧，共视数
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }
    if(keyframeCounter.empty())
        return;

    /// 添加局部关键帧（共视关键帧，子关键帧，父关键帧）
    int max=0;                                          // 最大共视
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);     // 最大共视的关键帧
    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());// 局部关键帧列表

    // 能观测到当前帧地图点的关键帧作为局部关键帧
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;
        if(pKF->isBad())
            continue;
        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;    // 最大共视关键帧
        }
        mvpLocalKeyFrames.push_back(it->first);

        // 将局部关键帧与当前帧关联，防止后面重复加入
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }

    // 与 得到的局部关键帧 共视程度很高 的关键帧 作为局部关键帧
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // 添加局部关键帧，直到80帧
        if(mvpLocalKeyFrames.size()>80)
            break;

        // 当前局部关键帧
        KeyFrame* pKF = *itKF;

        // 1.最佳共视的10帧
        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);
        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                // 是否已经是 当前帧的局部关键帧？
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 2.自己的子关键帧
        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                // 是否已经是 当前帧的局部关键帧？
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        // 3.自己的父关键帧
        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                // 是否已经是 当前帧的局部关键帧？
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    /// 更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

/// 重定位（三次匹配三次优化）
bool Tracking::Relocalization()
{
    // 计算当前帧的词袋向量
    mCurrentFrame.ComputeBoW();

    // 根据关键帧数据库，寻找候选关键帧
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();        // 候选帧的数量
    ORBmatcher matcher(0.75,true);                 // 构造匹配器（通过词袋匹配）
    vector<PnPsolver*> vpPnPsolvers;               // PnP求解器
    vpPnPsolvers.resize(nKFs);
    vector<vector<MapPoint*> > vvpMapPointMatches; // 特征匹配结果
    vvpMapPointMatches.resize(nKFs);
    vector<bool> vbDiscarded;                      // 候选帧是否被抛弃
    vbDiscarded.resize(nKFs);
    int nCandidates=0;                             // 有效候选帧数量

    // 遍历候选关键帧
    for(int i=0; i<nKFs; i++)
    {
        // 当前候选帧
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;      // 抛弃
        else
        {
            /// 当前帧与候选帧，通过词袋，进行特征匹配
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;  // 抛弃该候选帧
                continue;
            }
            else
            {
                // 成功匹配，添加EPnP求解器
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991); // 设置RANSAC迭代的参数
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // 一直循环，直到重定位成功，或者所有候选帧被抛弃
    bool bMatch = false;             // 成功匹配（重定位成功）
    ORBmatcher matcher2(0.9,true);   // 构造匹配器（通过重投影匹配）
    while(nCandidates>0 && !bMatch)
    {
        // 遍历有效的候选帧
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            /// 进行5次EPnP迭代，求解当前位姿
            vector<bool> vbInliers; // 是否为内点
            int nInliers;           // 内点数量
            bool bNoMore;           // 迭代到最大迭代次数，无法再迭代
            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // 迭代到最大迭代次数，无法再迭代
            if(bNoMore)
            {
                vbDiscarded[i]=true;// 抛弃该候选帧
                nCandidates--;
            }

            if(!Tcw.empty())
            {
                // 将PnP的内点，加入地图
                Tcw.copyTo(mCurrentFrame.mTcw);
                set<MapPoint*> sFound; // 记录已经匹配上地图点的集合
                const int np = vbInliers.size();
                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        // 将PnP的内点，加入到地图
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                /// 对当前帧位姿进行优化
                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                if(nGood<10)
                    continue;

                // 剔除位姿优化的外点
                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // 如果位姿优化的内点数大于50，表示重定位成功;否则继续
                if(nGood<50)
                {
                    /// 当前帧与候选帧，通过重投影，进行特征匹配
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        /// 继续对当前帧位姿进行优化
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        if(nGood>30 && nGood<50)
                        {
                            // 将上一次特征匹配到的 新的地图点，加入集合;用于下一次特征匹配
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);

                            /// 当前帧与候选帧，再次通过重投影，进行特征匹配
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            if(nGood+nadditional>=50)
                            {
                                /// 最后一次，对当前帧位姿进行优化
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                // 剔除位姿优化的外点
                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }

                // 位姿优化的内点数多于50个，表示重定位成功，提前结束循环
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    // 返回重定位结果
    if(!bMatch)
    {
        return false;   // 重定位失败
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;    // 重定位成功
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;

    // 关闭可视化线程
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // 重启局部地图线程
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // 重启闭环检测线程
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // 清空关键帧数据库
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // 清空地图
    mpMap->clear();

    KeyFrame::nNextId = 0;  // 关键帧ID，从0开始
    Frame::nNextId = 0;     // 普通帧ID，从0开始
    mState = NO_IMAGES_YET; // 状态设置为，还没有接收到图片

    // 重新单目初始化
    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    // 清空每一帧的数据
    mlRelativeFramePoses.clear(); // 相对位姿
    mlpReferences.clear();        // 参考帧
    mlFrameTimes.clear();         // 时间戳
    mlbLost.clear();              // 是否跟丢

    // 重启可视化线程
    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
