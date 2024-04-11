/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
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

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#include <iostream>

#include "PnPsolver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include "Thirdparty/DBoW2/DUtils/Random.h"
#include <algorithm>

using namespace std;

namespace ORB_SLAM2
{

/// 构造EPnP求解器
PnPsolver::PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches):
    pws(0), us(0), alphas(0), pcs(0),  // 3D世界坐标，2D像素坐标，表达系数，3D相机坐标
    maximum_number_of_correspondences(0), number_of_correspondences(0), mnInliersi(0),
    mnIterations(0), mnBestInliers(0), N(0)
{
    // 根据点数初始化容器的大小
    mvpMapPointMatches = vpMapPointMatches;           // 地图点列表（和特征列表大小一致）
    mvP2D.reserve(F.mvpMapPoints.size());             // 2D像素坐标
    mvSigma2.reserve(F.mvpMapPoints.size());          // 比例因子的平方（地图点所在的层）
    mvP3Dw.reserve(F.mvpMapPoints.size());            // 3D世界坐标
    mvKeyPointIndices.reserve(F.mvpMapPoints.size()); // 地图点在特征列表的索引
    mvAllIndices.reserve(F.mvpMapPoints.size());      // 地图点的绝对索引（用来随机选择）

    // 循环遍历每个地图点
    int idx=0;
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];

        if(pMP)
        {
            if(!pMP->isBad())
            {
                /// 当前地图点
                const cv::KeyPoint &kp = F.mvKeysUn[i];

                mvP2D.push_back(kp.pt);                         // 2D像素坐标
                mvSigma2.push_back(F.mvLevelSigma2[kp.octave]); // 比例因子的平方（地图点所在的层）
                cv::Mat Pos = pMP->GetWorldPos();
                mvP3Dw.push_back(cv::Point3f(Pos.at<float>(0),Pos.at<float>(1), Pos.at<float>(2)));
                mvKeyPointIndices.push_back(i);                 // 地图点在特征列表的索引
                mvAllIndices.push_back(idx);                    // 地图点的绝对索引（用来随机选择）

                idx++;
            }
        }
    }

    // 相机内参矩阵
    fu = F.fx;
    fv = F.fy;
    uc = F.cx;
    vc = F.cy;

    // 设置迭代的参数
    SetRansacParameters();
}

PnPsolver::~PnPsolver()
{
  delete [] pws;
  delete [] us;
  delete [] alphas;
  delete [] pcs;
}

/// 设置EPnP迭代的参数（置信度，最小内点数，最大迭代次数，单次迭代的匹配点数，内点数：总点数，区分内点的阈值）
void PnPsolver::SetRansacParameters(double probability, int minInliers, int maxIterations, int minSet, float epsilon, float th2)
{
    mRansacProb = probability;      // 置信度
    mRansacMinInliers = minInliers; // 最小内点数（单次迭代是否成功）
    mRansacMaxIts = maxIterations;  // 最大累积迭代次数
    mRansacEpsilon = epsilon;       // 内点数：总点数（期望的）
    mRansacMinSet = minSet;         // 单次迭代需要的匹配点数

    N = mvP2D.size();      // 匹配点的数量
    mvbInliersi.resize(N); // 是否为内点

    // 最小内点数（单次迭代是否成功）
    int nMinInliers = N*mRansacEpsilon;
    if(nMinInliers<mRansacMinInliers)
        nMinInliers=mRansacMinInliers;
    if(nMinInliers<minSet)
        nMinInliers=minSet;
    mRansacMinInliers = nMinInliers;

    // 内点数：总点数（期望的）
    if(mRansacEpsilon<(float)mRansacMinInliers/N)
        mRansacEpsilon=(float)mRansacMinInliers/N;

    // 最大累积迭代次数
    int nIterations;
    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(mRansacEpsilon,3)));
    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    // 区分内点的阈值（金字塔）
    mvMaxError.resize(mvSigma2.size());
    for(size_t i=0; i<mvSigma2.size(); i++)
        mvMaxError[i] = mvSigma2[i]*th2;
}

cv::Mat PnPsolver::find(vector<bool> &vbInliers, int &nInliers)
{
    bool bFlag;
    return iterate(mRansacMaxIts,bFlag,vbInliers,nInliers);    
}

/// 执行EPnP算法，迭代多次（指定迭代次数，是否无法再迭代，是否为内点，内点数量）
cv::Mat PnPsolver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
{
    bNoMore = false;   // 不能再进行迭代（超过最大累积迭代次数，匹配点数小于最小内点数）
    vbInliers.clear(); // 是否为内点
    nInliers=0;        // 内点的数量

    // 设置单次迭代需要的匹配点数
    set_maximum_number_of_correspondences(mRansacMinSet);

    // 匹配点数小于最小内点数
    if(N<mRansacMinInliers)
    {
        bNoMore = true;
        return cv::Mat();
    }

    // 进行多次迭代（超过累积迭代次数，达到指定迭代次数）
    vector<size_t> vAvailableIndices;    // 随机列表
    int nCurrentIterations = 0;          // 当前迭代次数
    while(mnIterations<mRansacMaxIts || nCurrentIterations<nIterations)
    {
        nCurrentIterations++;            // 当前迭代次数
        mnIterations++;                  // 累积迭代次数
        reset_correspondences();         // 初始化当前迭代的匹配点序号
        vAvailableIndices = mvAllIndices;// 随机列表（地图点的绝对索引）

        // 循环添加，当前迭代需要的随机匹配点
        for(short i = 0; i < mRansacMinSet; ++i)
        {
            // 随机选择一对匹配点，并添加
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];
            add_correspondence(mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,mvP2D[idx].x,mvP2D[idx].y);

            // 将这对匹配点，从随机列表中移除
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        // 计算相机的位姿（旋转矩阵，平移向量）
        compute_pose(mRi, mti);

        // 检查是否为内点（所有的匹配点）
        CheckInliers();

        // 检查内点数
        if(mnInliersi>=mRansacMinInliers)
        {
            // 保存内点数最多的结果
            if(mnInliersi>mnBestInliers)
            {
                mvbBestInliers = mvbInliersi;         // 是否为内点
                mnBestInliers = mnInliersi;           // 最多内点数

                cv::Mat Rcw(3,3,CV_64F,mRi);
                cv::Mat tcw(3,1,CV_64F,mti);
                Rcw.convertTo(Rcw,CV_32F);
                tcw.convertTo(tcw,CV_32F);
                mBestTcw = cv::Mat::eye(4,4,CV_32F);  // 相机位姿
                Rcw.copyTo(mBestTcw.rowRange(0,3).colRange(0,3));
                tcw.copyTo(mBestTcw.rowRange(0,3).col(3));
            }

            // 根据内点数最多的结果，剔除外点，再进行迭代（提纯）
            if(Refine())
            {
                // 提纯成功，保留结果
                nInliers = mnRefinedInliers;                               // 内点数
                vbInliers = vector<bool>(mvpMapPointMatches.size(),false); // 是否为内点
                for(int i=0; i<N; i++)
                {
                    if(mvbRefinedInliers[i])
                        vbInliers[mvKeyPointIndices[i]] = true;
                }
                return mRefinedTcw.clone();  // 返回相机的位姿
            }

        }
    }

    // 累积迭代次数超过限制
    if(mnIterations>=mRansacMaxIts)
    {
        bNoMore=true;
        if(mnBestInliers>=mRansacMinInliers)
        {
            nInliers=mnBestInliers;                                   // 内点数
            vbInliers = vector<bool>(mvpMapPointMatches.size(),false);// 是否为内点
            for(int i=0; i<N; i++)
            {
                if(mvbBestInliers[i])
                    vbInliers[mvKeyPointIndices[i]] = true;
            }
            return mBestTcw.clone();    // 返回相机的位姿
        }
    }

    return cv::Mat();
}

/// 根据内点数最多的结果，剔除外点，再进行迭代（提纯）
bool PnPsolver::Refine()
{
    // 剔除外点
    vector<int> vIndices;
    vIndices.reserve(mvbBestInliers.size());
    for(size_t i=0; i<mvbBestInliers.size(); i++)
    {
        if(mvbBestInliers[i])
        {
            vIndices.push_back(i);
        }
    }

    // 设置单次迭代需要的匹配点数
    set_maximum_number_of_correspondences(vIndices.size());

    // 初始化当前迭代的匹配点序号
    reset_correspondences();

    // 循环添加，当前迭代需要的随机匹配点，将所有的内点都加入
    for(size_t i=0; i<vIndices.size(); i++)
    {
        int idx = vIndices[i];
        add_correspondence(mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,mvP2D[idx].x,mvP2D[idx].y);
    }

    // 计算相机的位姿（旋转矩阵，平移向量）
    compute_pose(mRi, mti);

    // 检查是否为内点（所有的匹配点）
    CheckInliers();

    // 提纯后的内点
    mnRefinedInliers =mnInliersi;
    mvbRefinedInliers = mvbInliersi;

    // 提纯后的内点数，是否达到要求
    if(mnInliersi>mRansacMinInliers)
    {
        cv::Mat Rcw(3,3,CV_64F,mRi);
        cv::Mat tcw(3,1,CV_64F,mti);
        Rcw.convertTo(Rcw,CV_32F);
        tcw.convertTo(tcw,CV_32F);
        mRefinedTcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(mRefinedTcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(mRefinedTcw.rowRange(0,3).col(3));
        return true; // 提纯成功
    }

    return false;    // 提纯失败
}

/// 检查是否为内点（所有的匹配点）
void PnPsolver::CheckInliers()
{
    // 内点的个数
    mnInliersi=0;

    // 遍历EPnP所有的匹配点（不仅仅是一次迭代的4个）
    for(int i=0; i<N; i++)
    {
        cv::Point3f P3Dw = mvP3Dw[i];     // 3D世界坐标
        cv::Point2f P2D = mvP2D[i];       // 2D像素坐标（实际的）

        float Xc = mRi[0][0]*P3Dw.x+mRi[0][1]*P3Dw.y+mRi[0][2]*P3Dw.z+mti[0];         // 3D相机坐标
        float Yc = mRi[1][0]*P3Dw.x+mRi[1][1]*P3Dw.y+mRi[1][2]*P3Dw.z+mti[1];
        float invZc = 1/(mRi[2][0]*P3Dw.x+mRi[2][1]*P3Dw.y+mRi[2][2]*P3Dw.z+mti[2]);

        double ue = uc + fu * Xc * invZc; // 2D像素坐标（重投影）
        double ve = vc + fv * Yc * invZc;

        float distX = P2D.x-ue;
        float distY = P2D.y-ve;
        float error2 = distX*distX+distY*distY; // 重投影误差

        // 根据重投影误差，判断是否为内点
        if(error2<mvMaxError[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
        {
            mvbInliersi[i]=false;
        }
    }
}

/// 设置单次迭代的最大匹配点数
void PnPsolver::set_maximum_number_of_correspondences(int n)
{
  if (maximum_number_of_correspondences < n) {
    if (pws != 0) delete [] pws;
    if (us != 0) delete [] us;
    if (alphas != 0) delete [] alphas;
    if (pcs != 0) delete [] pcs;

    maximum_number_of_correspondences = n;                      // 单次迭代需要的匹配点数
    pws = new double[3 * maximum_number_of_correspondences];    // 3D世界坐标
    us = new double[2 * maximum_number_of_correspondences];     // 2D像素坐标
    alphas = new double[4 * maximum_number_of_correspondences]; // 表达系数
    pcs = new double[3 * maximum_number_of_correspondences];    // 3D相机坐标
  }
}

/// 初始化当前迭代的匹配点序号
void PnPsolver::reset_correspondences(void)
{
  number_of_correspondences = 0;
}

/// 添加当前匹配点对（3D世界坐标，2D像素坐标）
void PnPsolver::add_correspondence(double X, double Y, double Z, double u, double v)
{
  pws[3 * number_of_correspondences    ] = X; // 3D世界坐标
  pws[3 * number_of_correspondences + 1] = Y;
  pws[3 * number_of_correspondences + 2] = Z;

  us[2 * number_of_correspondences    ] = u;  // 2D像素坐标
  us[2 * number_of_correspondences + 1] = v;

  number_of_correspondences++;                // 匹配点序号+1
}

/// 选择控制点
void PnPsolver::choose_control_points(void)
{
  /// 第一个控制点：3D世界坐标的质心（单次迭代仅有4对匹配点）
  cws[0][0] = cws[0][1] = cws[0][2] = 0;
  for(int i = 0; i < number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      cws[0][j] += pws[3 * i + j];
  for(int j = 0; j < 3; j++)
    cws[0][j] /= number_of_correspondences;

  /// 其他三个控制点：通过PCA分解得到

  // 3D世界坐标->矩阵形式
  CvMat * PW0 = cvCreateMat(number_of_correspondences, 3, CV_64F);

  // SVD分解的A,W,U'
  double pw0tpw0[3 * 3], dc[3], uct[3 * 3];     // A,W,U'
  CvMat PW0tPW0 = cvMat(3, 3, CV_64F, pw0tpw0); // 协方差矩阵A
  CvMat DC      = cvMat(3, 1, CV_64F, dc);      // SVD分解的W
  CvMat UCt     = cvMat(3, 3, CV_64F, uct);     // SVD分解的U'

  // 计算协方差矩阵（除或不除样本数量n,对求出的特征向量没有影响）
  for(int i = 0; i < number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      PW0->data.db[3 * i + j] = pws[3 * i + j] - cws[0][j]; // 去质心
  cvMulTransposed(PW0, &PW0tPW0, 1);                        // 协方差矩阵（P'P/n）

  // 对协方差矩阵，进行SVD分解 A=UWV' （A，W，U'，V，flag）
  cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T); // flag:允许改变A，返回U的转置

  // 释放矩阵空间
  cvReleaseMat(&PW0);

  // 获取其余三个控制点的坐标
  for(int i = 1; i < 4; i++)
  {
    // 当前控制点对应的奇异值（前面的协方差矩阵，没有除以样本数n）
    double k = sqrt(dc[i - 1] / number_of_correspondences); // 奇异值开根号

    // 控制点的3D世界坐标
    for(int j = 0; j < 3; j++)
      cws[i][j] = cws[0][j] + k * uct[3 * (i - 1) + j];     // 加上质心的坐标
  }
}

/// 计算表达系数（匹配点，可以由四个控制点表示）（求逆的方法）
void PnPsolver::compute_barycentric_coordinates(void)
{
  // 后三个控制点世界坐标，去质心后，构成的矩阵
  double cc[3 * 3], cc_inv[3 * 3];
  CvMat CC     = cvMat(3, 3, CV_64F, cc);
  CvMat CC_inv = cvMat(3, 3, CV_64F, cc_inv);
  for(int i = 0; i < 3; i++)
    for(int j = 1; j < 4; j++)                   // 和cws是转置关系
      cc[3 * i + j - 1] = cws[j][i] - cws[0][i]; // 控制点，去质心

  // 对上面得到的矩阵进行求逆
  cvInvert(&CC, &CC_inv, CV_SVD);
  double * ci = cc_inv;

  // 将每一个3D世界坐标，用控制点表达
  for(int i = 0; i < number_of_correspondences; i++)
  {
    double * pi = pws + 3 * i;   // 3D世界坐标
    double * a = alphas + 4 * i; // 表达系数，保存的位置

    // 表达系数 αi2,αi3,αi4
    for(int j = 0; j < 3; j++)
      a[1 + j] =
                ci[3 * j    ] * (pi[0] - cws[0][0]) +  // 3D世界坐标，去质心
                ci[3 * j + 1] * (pi[1] - cws[0][1]) +
                ci[3 * j + 2] * (pi[2] - cws[0][2]);

    // 表达系数 αi1
    a[0] = 1.0f - a[1] - a[2] - a[3];
  }
}

/// 填充M矩阵（填充一对匹配点）
void PnPsolver::fill_M(CvMat * M,
		  const int row, const double * as, const double u, const double v)
          // 填充的起始行数，表达系数αi，像素坐标u，像素坐标v
{
  // 需要填充的两行
  double * M1 = M->data.db + row * 12; // 第一行的起始位置
  double * M2 = M1 + 12;               // 第二行的起始位置

  // 循环填入表达系数 αi1-αi2
  for(int i = 0; i < 4; i++)
  {
    M1[3 * i    ] = as[i] * fu;        // 第一行：|αi1*fu  0  αi1(uc-ui)| ...
    M1[3 * i + 1] = 0.0;
    M1[3 * i + 2] = as[i] * (uc - u);

    M2[3 * i    ] = 0.0;               // 第二行：|0  αi1*fv  αi1(vc-vi)| ...
    M2[3 * i + 1] = as[i] * fv;
    M2[3 * i + 2] = as[i] * (vc - v);
  }
}

/// 根据 Mx=0 的解，得到控制点的3D相机坐标
void PnPsolver::compute_ccs(const double * betas, const double * ut)
{
  // 初始化控制点的3D相机坐标
  for(int i = 0; i < 4; i++)
    ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0f;

  // β1U1+β2U2+β3U3+β4U4
  for(int i = 0; i < 4; i++)
  {
    // 当前解向量Ui
    const double * v = ut + 12 * (11 - i);

    // Ui=4×3（四个控制点的3D相机坐标）
    for(int j = 0; j < 4; j++)
      for(int k = 0; k < 3; k++)
        ccs[j][k] += betas[i] * v[3 * j + k];
  }
}

/// 计算匹配点的3D相机坐标
void PnPsolver::compute_pcs(void)
{
  for(int i = 0; i < number_of_correspondences; i++)
  {
    double * a = alphas + 4 * i; // 当前表达系数
    double * pc = pcs + 3 * i;   // 当前匹配点

    // 3D相机坐标，与3D世界坐标 的表达系数一样
    for(int j = 0; j < 3; j++)
      pc[j] = a[0] * ccs[0][j] + a[1] * ccs[1][j] + a[2] * ccs[2][j] + a[3] * ccs[3][j];
  }
}

/// 计算相机的位姿（旋转矩阵，平移向量）
double PnPsolver::compute_pose(double R[3][3], double t[3])
{
  // 选择控制点
  choose_control_points();

  // 计算表达系数（匹配点，可以由四个控制点表示）
  compute_barycentric_coordinates();

  // 构造M矩阵，用来求解控制点的3D相机坐标
  CvMat * M = cvCreateMat(2 * number_of_correspondences, 12, CV_64F);
  for(int i = 0; i < number_of_correspondences; i++)
    fill_M(M, 2 * i, alphas + 4 * i, us[2 * i], us[2 * i + 1]);

  // 求解 Mx=0（奇异值分解M'M）
  double mtm[12 * 12], d[12], ut[12 * 12];
  CvMat MtM = cvMat(12, 12, CV_64F, mtm);
  CvMat D   = cvMat(12,  1, CV_64F, d);
  CvMat Ut  = cvMat(12, 12, CV_64F, ut);
  cvMulTransposed(M, &MtM, 1);
  cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T); // U'的最下面四行，为Mx=0的解向量
  cvReleaseMat(&M);

  // 准备求解 Lβ=ρ
  double l_6x10[6 * 10], rho[6];
  CvMat L_6x10 = cvMat(6, 10, CV_64F, l_6x10); // 矩阵L
  CvMat Rho    = cvMat(6,  1, CV_64F, rho);    // 两两控制点之间的距离ρ
  compute_L_6x10(ut, l_6x10);  // 填充L
  compute_rho(rho);            // 计算ρ

  /// 进行三次高斯牛顿优化，取最优的那一次作为结果
  double Betas[4][4], rep_errors[4]; // 优化后结果β，重投影误差
  double Rs[4][3][3], ts[4][3];      // 旋转矩阵R,平移向量t

  // 第一次优化
  find_betas_approx_1(&L_6x10, &Rho, Betas[1]);               // 寻找β初值（SVD分解）
  gauss_newton(&L_6x10, &Rho, Betas[1]);                      // 进行高斯牛顿优化
  rep_errors[1] = compute_R_and_t(ut, Betas[1], Rs[1], ts[1]);// 计算相机的位姿，并返回重投影误差

  // 第二次优化
  find_betas_approx_2(&L_6x10, &Rho, Betas[2]);
  gauss_newton(&L_6x10, &Rho, Betas[2]);
  rep_errors[2] = compute_R_and_t(ut, Betas[2], Rs[2], ts[2]);

  // 第三次优化
  find_betas_approx_3(&L_6x10, &Rho, Betas[3]);
  gauss_newton(&L_6x10, &Rho, Betas[3]);
  rep_errors[3] = compute_R_and_t(ut, Betas[3], Rs[3], ts[3]);

  // 选择效果最好的那次优化
  int N = 1;
  if (rep_errors[2] < rep_errors[1]) N = 2;
  if (rep_errors[3] < rep_errors[N]) N = 3;

  // 拷贝结果R,t，作为函数的输出
  copy_R_and_t(Rs[N], ts[N], R, t);

  // 返回重投影误差
  return rep_errors[N];
}

/// 拷贝R,t
void PnPsolver::copy_R_and_t(const double R_src[3][3], const double t_src[3], double R_dst[3][3], double t_dst[3])
{
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
      R_dst[i][j] = R_src[i][j];
    t_dst[i] = t_src[i];
  }
}

/// 两个坐标点的距离平方
double PnPsolver::dist2(const double * p1, const double * p2)
{
  return
    (p1[0] - p2[0]) * (p1[0] - p2[0]) +
    (p1[1] - p2[1]) * (p1[1] - p2[1]) +
    (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

/// 两个向量的点积
double PnPsolver::dot(const double * v1, const double * v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/// 计算重投影误差
double PnPsolver::reprojection_error(const double R[3][3], const double t[3])
{
  double sum2 = 0.0;

  for(int i = 0; i < number_of_correspondences; i++)
  {
    double * pw = pws + 3 * i;                    // 3D世界坐标

    double Xc = dot(R[0], pw) + t[0];             // 3D相机坐标
    double Yc = dot(R[1], pw) + t[1];
    double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);

    double ue = uc + fu * Xc * inv_Zc;            // 2D像素坐标（重投影）
    double ve = vc + fv * Yc * inv_Zc;

    double u = us[2 * i], v = us[2 * i + 1];      // 2D像素坐标（实际的）

    sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
  }

  // 返回重投影误差
  return sum2 / number_of_correspondences;
}

/// ICP 求解相机的位姿
void PnPsolver::estimate_R_and_t(double R[3][3], double t[3])
{
  // 计算质心 pc0，pw0
  double pc0[3], pw0[3];
  pc0[0] = pc0[1] = pc0[2] = 0.0; // 相机坐标系的质心
  pw0[0] = pw0[1] = pw0[2] = 0.0; // 世界坐标系的质心
  for(int i = 0; i < number_of_correspondences; i++)
  {
    const double * pc = pcs + 3 * i;
    const double * pw = pws + 3 * i;

    for(int j = 0; j < 3; j++)
    {
      pc0[j] += pc[j];
      pw0[j] += pw[j];
    }
  }
  for(int j = 0; j < 3; j++)
  {
    pc0[j] /= number_of_correspondences;
    pw0[j] /= number_of_correspondences;
  }

  // SVD分解得到 AB'=UDV'
  double abt[3 * 3], abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
  CvMat ABt   = cvMat(3, 3, CV_64F, abt);
  CvMat ABt_D = cvMat(3, 1, CV_64F, abt_d);
  CvMat ABt_U = cvMat(3, 3, CV_64F, abt_u);
  CvMat ABt_V = cvMat(3, 3, CV_64F, abt_v);
  cvSetZero(&ABt);
  for(int i = 0; i < number_of_correspondences; i++)
  {
    double * pc = pcs + 3 * i;
    double * pw = pws + 3 * i;

    // 计算 AB'
    for(int j = 0; j < 3; j++)
    {
      abt[3 * j    ] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
      abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
      abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
    }
  }

  // SVD分解得到 AB'=UDV'
  cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A); // 得到是V，不是V'

  // 计算旋转矩阵R
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j); // U的第i行 和 V的第j行 的点积

  // 判断R的符号是否正确
  const double det =
    R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
    R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];
  if (det < 0) {
    R[2][0] = -R[2][0];
    R[2][1] = -R[2][1];
    R[2][2] = -R[2][2];
  }

  // 根据R，计算平移向量t，t=pc0-R*pw0
  t[0] = pc0[0] - dot(R[0], pw0);
  t[1] = pc0[1] - dot(R[1], pw0);
  t[2] = pc0[2] - dot(R[2], pw0);
}

void PnPsolver::print_pose(const double R[3][3], const double t[3])
{
  cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << t[0] << endl;
  cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << t[1] << endl;
  cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << t[2] << endl;
}

/// 根据匹配点3D相机坐标的深度，判断 Mx=0的解 符号是否取反
void PnPsolver::solve_for_sign(void)
{
  if (pcs[2] < 0.0)
  {
    for(int i = 0; i < 4; i++)
      for(int j = 0; j < 3; j++)
	ccs[i][j] = -ccs[i][j];

    for(int i = 0; i < number_of_correspondences; i++)
    {
      pcs[3 * i    ] = -pcs[3 * i];
      pcs[3 * i + 1] = -pcs[3 * i + 1];
      pcs[3 * i + 2] = -pcs[3 * i + 2];
    }
  }
}

/// 计算相机的位姿，并返回重投影误差
double PnPsolver::compute_R_and_t(const double * ut, const double * betas, double R[3][3], double t[3])
{
  compute_ccs(betas, ut);  // 计算控制点的3D相机坐标
  compute_pcs();           // 计算匹配点的3D相机坐标
  solve_for_sign();        // 判断 Mx=0的解 符号是否取反
  estimate_R_and_t(R, t);  // ICP 求解相机的位姿

  // 返回重投影误差
  return reprojection_error(R, t);
}

/// 找到第一组初值 β1，β3，β3，β4（用于高斯牛顿优化）
void PnPsolver::find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas)
{
  /// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
  /// betas_approx_1 = [B11 B12     B13         B14]

  // 准备求解 Lβ=ρ（L是6*4的部分）
  double l_6x4[6 * 4], b4[4];
  CvMat L_6x4 = cvMat(6, 4, CV_64F, l_6x4);     // 矩阵L（6*4部分）
  CvMat B4    = cvMat(4, 1, CV_64F, b4);        // Lβ=ρ 的解
  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x4, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x4, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x4, i, 2, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x4, i, 3, cvmGet(L_6x10, i, 6));
  }

  // 求解 Lβ=ρ（SVD分解）
  cvSolve(&L_6x4, Rho, &B4, CV_SVD);

  // 计算 β1，β3，β3，β4
  if (b4[0] < 0)
  {
    betas[0] = sqrt(-b4[0]);      // β1
    betas[1] = -b4[1] / betas[0]; // β2
    betas[2] = -b4[2] / betas[0]; // β3
    betas[3] = -b4[3] / betas[0]; // β4
  }
  else
  {
    betas[0] = sqrt(b4[0]);
    betas[1] = b4[1] / betas[0];
    betas[2] = b4[2] / betas[0];
    betas[3] = b4[3] / betas[0];
  }
}

/// 找到第二组初值β1，β3，β3，β4（用于高斯牛顿优化）
void PnPsolver::find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas)
{
  /// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
  /// betas_approx_2 = [B11 B12 B22]

  // 准备求解 Lβ=ρ（L是6*3的部分）
  double l_6x3[6 * 3], b3[3];
  CvMat L_6x3  = cvMat(6, 3, CV_64F, l_6x3);    // 矩阵L（6*3部分）
  CvMat B3     = cvMat(3, 1, CV_64F, b3);       // Lβ=ρ 的解
  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x3, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x3, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x3, i, 2, cvmGet(L_6x10, i, 2));
  }

  // 求解 Lβ=ρ（SVD分解）
  cvSolve(&L_6x3, Rho, &B3, CV_SVD);

  // 计算 β1，β3，β3，β4
  if (b3[0] < 0) {
    betas[0] = sqrt(-b3[0]);                     // β1
    betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0; // β2
  }
  else
  {
    betas[0] = sqrt(b3[0]);
    betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
  }
  if (b3[1] < 0) betas[0] = -betas[0];           // β1与β2符号相反
  betas[2] = 0.0;                                // β3
  betas[3] = 0.0;                                // β4
}

/// 找到第三组初值β1，β3，β3，β4（用于高斯牛顿优化）
void PnPsolver::find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
  /// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
  /// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

  // 准备求解 Lβ=ρ（L是6*5的部分）
  double l_6x5[6 * 5], b5[5];
  CvMat L_6x5 = cvMat(6, 5, CV_64F, l_6x5);     // 矩阵L（6*5部分）
  CvMat B5    = cvMat(5, 1, CV_64F, b5);        // Lβ=ρ 的解
  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x5, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x5, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x5, i, 2, cvmGet(L_6x10, i, 2));
    cvmSet(&L_6x5, i, 3, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x5, i, 4, cvmGet(L_6x10, i, 4));
  }

  // 求解 Lβ=ρ（SVD分解）
  cvSolve(&L_6x5, Rho, &B5, CV_SVD);

  // 计算 β1，β3，β3，β4
  if (b5[0] < 0) {
    betas[0] = sqrt(-b5[0]);                     // β1
    betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0; // β2
  } else {
    betas[0] = sqrt(b5[0]);
    betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
  }
  if (b5[1] < 0) betas[0] = -betas[0];           // β1与β2符号相反
  betas[2] = b5[3] / betas[0];                   // β3
  betas[3] = 0.0;                                // β4
}

/// 填充L
void PnPsolver::compute_L_6x10(const double * ut, double * l_6x10)
{
  // SVD分解的U'，的最下面四行，为 Mx=0 的解向量
  const double * v[4];
  v[0] = ut + 12 * 11;
  v[1] = ut + 12 * 10;
  v[2] = ut + 12 *  9;
  v[3] = ut + 12 *  8;

  // 循环对每一个解向量处理
  double dv[4][6][3];
  for(int i = 0; i < 4; i++)
  {
    // 每一个解向量可分为：v1,v2,v3,v4  ------------------>  // v1-v2
    int a = 0, b = 1;                                     // v1-v3
    for(int j = 0; j < 6; j++)                            // v1-v4
    {                                                     // v2-v3
      dv[i][j][0] = v[i][3 * a    ] - v[i][3 * b];        // v2-v4
      dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];    // v3-v4
      dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

      b++;
      if (b > 3)
      {
        a++;
        b = a + 1;
      }
    }
  }

  // v1-v2，v1-v3，v1-v4，v2-v3，v2-v4，v3-v4
  for(int i = 0; i < 6; i++)
  {
    double * row = l_6x10 + 10 * i;

    row[0] =        dot(dv[0][i], dv[0][i]);  // β11
    row[1] = 2.0f * dot(dv[0][i], dv[1][i]);  // β12
    row[2] =        dot(dv[1][i], dv[1][i]);  // β22
    row[3] = 2.0f * dot(dv[0][i], dv[2][i]);  // β13
    row[4] = 2.0f * dot(dv[1][i], dv[2][i]);  // β23
    row[5] =        dot(dv[2][i], dv[2][i]);  // β33
    row[6] = 2.0f * dot(dv[0][i], dv[3][i]);  // β14
    row[7] = 2.0f * dot(dv[1][i], dv[3][i]);  // β24
    row[8] = 2.0f * dot(dv[2][i], dv[3][i]);  // β34
    row[9] =        dot(dv[3][i], dv[3][i]);  // β44
  }
}

/// 计算两两控制点之间的距离ρ
void PnPsolver::compute_rho(double * rho)
{
  rho[0] = dist2(cws[0], cws[1]);
  rho[1] = dist2(cws[0], cws[2]);
  rho[2] = dist2(cws[0], cws[3]);
  rho[3] = dist2(cws[1], cws[2]);
  rho[4] = dist2(cws[1], cws[3]);
  rho[5] = dist2(cws[2], cws[3]);
}

/// 计算高斯牛顿迭代的梯度（A为雅克比J，b=ρ-Lβ=-f）
void PnPsolver::compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho, double betas[4], CvMat * A, CvMat * b)
{
  // 对每一个行进行处理（一行，代表一个误差项）
  for(int i = 0; i < 6; i++)
  {
    // L的当前行（一个误差项）
    const double * rowL = l_6x10 + i * 10;

    // 当前误差项，对 β1，β3，β3，β4 的偏导
    double * rowA = A->data.db + i * 4;    // [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
    rowA[0] = 2 * rowL[0] * betas[0] +     rowL[1] * betas[1] +     rowL[3] * betas[2] +     rowL[6] * betas[3];
    rowA[1] =     rowL[1] * betas[0] + 2 * rowL[2] * betas[1] +     rowL[4] * betas[2] +     rowL[7] * betas[3];
    rowA[2] =     rowL[3] * betas[0] +     rowL[4] * betas[1] + 2 * rowL[5] * betas[2] +     rowL[8] * betas[3];
    rowA[3] =     rowL[6] * betas[0] +     rowL[7] * betas[1] +     rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

    // b=ρ-Lβ=-f
    cvmSet(b, i, 0, rho[i] -
	   (
	    rowL[0] * betas[0] * betas[0] +
	    rowL[1] * betas[0] * betas[1] +
	    rowL[2] * betas[1] * betas[1] +
	    rowL[3] * betas[0] * betas[2] +
	    rowL[4] * betas[1] * betas[2] +
	    rowL[5] * betas[2] * betas[2] +
	    rowL[6] * betas[0] * betas[3] +
	    rowL[7] * betas[1] * betas[3] +
	    rowL[8] * betas[2] * betas[3] +
	    rowL[9] * betas[3] * betas[3]
	    ));
  }
}

/// 高斯牛顿迭代优化（L，ρ，β）
void PnPsolver::gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double betas[4])
{
  /// J'J△X=-J'f  ----->   A'A△X=A'B   ------>   AX=B

  // 迭代次数
  const int iterations_number = 5;

  double a[6*4], b[6], x[4];
  CvMat A = cvMat(6, 4, CV_64F, a); // A=J
  CvMat B = cvMat(6, 1, CV_64F, b); // B=ρ-Lβ=-f
  CvMat X = cvMat(4, 1, CV_64F, x); // X=△X

  // 进行5次迭代优化
  for(int k = 0; k < iterations_number; k++)
  {
    // 计算高斯牛顿迭代的梯度（A为雅克比J，b=ρ-Lβ=-f）
    compute_A_and_b_gauss_newton(L_6x10->data.db, Rho->data.db, betas, &A, &B);

    // 求解 AX=b（QR分解）
    qr_solve(&A, &B, &X);

    // 更新 β的值
    for(int i = 0; i < 4; i++)
      betas[i] += x[i];
  }
}

/// 求解 AX=b（QR分解）
void PnPsolver::qr_solve(CvMat * A, CvMat * b, CvMat * X)
{
  static int max_nr = 0;
  static double * A1, * A2;

  const int nr = A->rows;        // 6行
  const int nc = A->cols;        // 4列
  if (max_nr != 0 && max_nr < nr)
  {
    delete [] A1;
    delete [] A2;
  }
  if (max_nr < nr)
  {
    max_nr = nr;
    A1 = new double[nr];
    A2 = new double[nr];
  }

  // 循环对每一列，进行正交化
  double * pA = A->data.db, * ppAkk = pA;        // ppAkk 用来取当前列的数据
  for(int k = 0; k < nc; k++)
  {
    // 判断当前列，元素是否全为0
    double * ppAik = ppAkk, eta = fabs(*ppAik);  // ppAik 用来取单个数据
    for(int i = k + 1; i < nr; i++)
    {
      double elt = fabs(*ppAik);
      if (eta < elt)
          eta = elt; // 绝对值中的最大值
      ppAik += nc;   // 遍历当前列的元素
    }

    if (eta == 0)
    {
      // 当前列，元素全为0，不应该发生
      A1[k] = A2[k] = 0.0;
      cerr << "God damnit, A is singular, this shouldn't happen." << endl;
      return;
    }
    else
    {
      /// 对当前列，进行正交化

      double * ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
      for(int i = k; i < nr; i++)
      {
        *ppAik *= inv_eta;
        sum += *ppAik * *ppAik;
        ppAik += nc;
      }

      double sigma = sqrt(sum);
      if (*ppAkk < 0)
        sigma = -sigma;
      *ppAkk += sigma;
      A1[k] = sigma * *ppAkk;
      A2[k] = -eta * sigma;

      for(int j = k + 1; j < nc; j++)
      {
        double * ppAik = ppAkk, sum = 0;
        for(int i = k; i < nr; i++)
        {
          sum += *ppAik * ppAik[j - k];
          ppAik += nc;
        }
        double tau = sum / A1[k];
        ppAik = ppAkk;
        for(int i = k; i < nr; i++)
        {
          ppAik[j - k] -= tau * *ppAik;
          ppAik += nc;
        }
      }
    }

    // 指向下一列
    ppAkk += nc + 1;
  }

  // b <- Qt b
  double * ppAjj = pA, * pb = b->data.db;
  for(int j = 0; j < nc; j++)
  {
    double * ppAij = ppAjj, tau = 0;
    for(int i = j; i < nr; i++)
    {
      tau += *ppAij * pb[i];
      ppAij += nc;
    }
    tau /= A1[j];
    ppAij = ppAjj;
    for(int i = j; i < nr; i++)
    {
      pb[i] -= tau * *ppAij;
      ppAij += nc;
    }
    ppAjj += nc + 1;
  }

  // X = R-1 b
  double * pX = X->data.db;
  pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
  for(int i = nc - 2; i >= 0; i--) {
    double * ppAij = pA + i * nc + (i + 1), sum = 0;

    for(int j = i + 1; j < nc; j++) {
      sum += *ppAij * pX[j];
      ppAij++;
    }
    pX[i] = (pb[i] - sum) / A2[i];
  }
}

void PnPsolver::relative_error(double & rot_err, double & transl_err,
			  const double Rtrue[3][3], const double ttrue[3],
			  const double Rest[3][3],  const double test[3])
{
  double qtrue[4], qest[4];

  mat_to_quat(Rtrue, qtrue);
  mat_to_quat(Rest, qest);

  double rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) +
			 (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
			 (qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) +
			 (qtrue[3] - qest[3]) * (qtrue[3] - qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  double rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) +
			 (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
			 (qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) +
			 (qtrue[3] + qest[3]) * (qtrue[3] + qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  rot_err = min(rot_err1, rot_err2);

  transl_err =
    sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) +
	 (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
	 (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
    sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}

void PnPsolver::mat_to_quat(const double R[3][3], double q[4])
{
  double tr = R[0][0] + R[1][1] + R[2][2];
  double n4;

  if (tr > 0.0f) {
    q[0] = R[1][2] - R[2][1];
    q[1] = R[2][0] - R[0][2];
    q[2] = R[0][1] - R[1][0];
    q[3] = tr + 1.0f;
    n4 = q[3];
  } else if ( (R[0][0] > R[1][1]) && (R[0][0] > R[2][2]) ) {
    q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
    q[1] = R[1][0] + R[0][1];
    q[2] = R[2][0] + R[0][2];
    q[3] = R[1][2] - R[2][1];
    n4 = q[0];
  } else if (R[1][1] > R[2][2]) {
    q[0] = R[1][0] + R[0][1];
    q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
    q[2] = R[2][1] + R[1][2];
    q[3] = R[2][0] - R[0][2];
    n4 = q[1];
  } else {
    q[0] = R[2][0] + R[0][2];
    q[1] = R[2][1] + R[1][2];
    q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
    q[3] = R[0][1] - R[1][0];
    n4 = q[2];
  }
  double scale = 0.5f / double(sqrt(n4));

  q[0] *= scale;
  q[1] *= scale;
  q[2] *= scale;
  q[3] *= scale;
}

} //namespace ORB_SLAM
