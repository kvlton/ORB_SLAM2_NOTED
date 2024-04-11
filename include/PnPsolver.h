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

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2
{

class PnPsolver {
 public:

  /// 构造EPnP求解器
  PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches);

  ~PnPsolver();

  /// 设置RANSAC迭代的参数（置信度，最小内点数，最大迭代次数，单次迭代的匹配点数，内点数：总点数，区分内点的阈值）
  void SetRansacParameters(double probability = 0.99, int minInliers = 8 , int maxIterations = 300, int minSet = 4, float epsilon = 0.4,
                           float th2 = 5.991);

  cv::Mat find(vector<bool> &vbInliers, int &nInliers);

  /// 执行EPnP算法，进行多次RANSAC迭代
  cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

 private:

  /// 检查是否为内点（所有的匹配点）
  void CheckInliers();

  /// 根据内点数最多的结果，剔除外点，再进行迭代（提纯）
  bool Refine();

  /// 设置单次迭代需要的匹配点数
  void set_maximum_number_of_correspondences(const int n);

  /// 初始化当前迭代的匹配点序号
  void reset_correspondences(void);

  /// 添加一对匹配点对（3D世界坐标，2D像素坐标）
  void add_correspondence(const double X, const double Y, const double Z,
              const double u, const double v);

  /// 计算相机的位姿（旋转矩阵，平移向量）
  double compute_pose(double R[3][3], double T[3]);

  void relative_error(double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  void print_pose(const double R[3][3], const double t[3]);

  /// 计算重投影误差
  double reprojection_error(const double R[3][3], const double t[3]);

  /// 选择控制点
  void choose_control_points(void);

  /// 计算表达系数（匹配点，可以由四个控制点表示）（求逆的方法）
  void compute_barycentric_coordinates(void);

  /// 填充M矩阵（填充一对匹配点）
  void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);

  /// 根据 Mx=0 的解，得到
  void compute_ccs(const double * betas, const double * ut); // 控制点的3D相机坐标
  void compute_pcs(void);                                    // 匹配点的3D相机坐标
  void solve_for_sign(void);   // 根据匹配点3D相机坐标的深度，判断 Mx=0的解 符号是否取反

  /// 找到三组初值β1，β3，β3，β4（用于高斯牛顿优化）
  void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas); // 第一组
  void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas); // 第二组
  void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas); // 第三组

  /// 求解 AX=b（QR分解）
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  /// 准备求解 Lβ=ρ
  double dot(const double * v1, const double * v2);       // 两个向量的点积
  double dist2(const double * p1, const double * p2);     // 两个坐标点的距离平方
  void compute_rho(double * rho);                         // 计算两两控制点之间的距离ρ
  void compute_L_6x10(const double * ut, double * l_6x10);// 填充L

  /// 高斯牛顿迭代优化
  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);

  /// 计算高斯牛顿迭代的梯度（A为雅克比J，b=ρ-Lβ=-f）
  void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho, double cb[4], CvMat * A, CvMat * b);

  /// 计算相机的位姿，并返回重投影误差
  double compute_R_and_t(const double * ut, const double * betas, double R[3][3], double t[3]);

  /// ICP 求解相机的位姿
  void estimate_R_and_t(double R[3][3], double t[3]);

  /// 拷贝R,t
  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  void mat_to_quat(const double R[3][3], double q[4]);

  /// 单次RANSAC迭代的数据
  double uc, vc, fu, fv;                 // 相机内参
  double * pws, * us, * alphas, * pcs;   // 3D世界坐标，2D像素坐标，表达系数，3D相机坐标（随机选择几对匹配点，进行一次迭代）
  int maximum_number_of_correspondences; // 单次迭代需要的匹配点数
  int number_of_correspondences;         // 单次迭代的匹配点序号（用来保存匹配点）
  double cws[4][3], ccs[4][3];           // 控制点的3D世界坐标
  double cws_determinant;

  /// EPnP所有的数据
  vector<MapPoint*> mvpMapPointMatches; // 地图点列表（和特征列表大小一致）
  vector<cv::Point2f> mvP2D;            // 2D像素坐标
  vector<float> mvSigma2;               // 比例因子的平方（地图点所在的层）
  vector<cv::Point3f> mvP3Dw;           // 3D世界坐标
  vector<size_t> mvKeyPointIndices;     // 地图点在特征列表的索引

  /// 单次RANSAC迭代的结果
  double mRi[3][3];              // 旋转矩阵
  double mti[3];                 // 平移向量
  cv::Mat mTcwi;                 // 变换矩阵
  vector<bool> mvbInliersi;      // 是否为内点
  int mnInliersi;                // 内点数

  /// 当前RANSAC迭代的状态
  int mnIterations;              // 累积迭代次数
  vector<bool> mvbBestInliers;   // 是否为内点（内点数最多）
  int mnBestInliers;             // 最多内点数
  cv::Mat mBestTcw;              // 最好的结果（内点数最多）

  // 对得到的结果进行提纯
  cv::Mat mRefinedTcw;            // 相机位姿
  vector<bool> mvbRefinedInliers; // 是否为内点
  int mnRefinedInliers;           // 内点数

  int N;                       // 匹配点的数量
  vector<size_t> mvAllIndices; // 地图点的绝对索引（用来随机选择）

  /// RANSAC迭代的参数
  double mRansacProb;       // 置信度
  int mRansacMinInliers;    // 最小内点数（单次迭代是否成功）
  int mRansacMaxIts;        // 最大累积迭代次数
  float mRansacEpsilon;     // 内点数：总点数（期望的）
  float mRansacTh;          // 未使用过？
  int mRansacMinSet;        // 单次迭代需要的匹配点数
  vector<float> mvMaxError; // 区分内点的阈值（金字塔）

};

} //namespace ORB_SLAM

#endif //PNPSOLVER_H
