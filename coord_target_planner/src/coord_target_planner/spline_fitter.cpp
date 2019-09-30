//
// Created by yonghui on 19-9-4.
//

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <Eigen/Dense>
#include "spline_fitter.h"

namespace coord_target_planner
{
    /**
     * @brief Newton迭代法求解三次方程
     * @param result  输出结果
     * @param coeff_matrix 三次方程系数矩阵
     * @param init 迭代初始值
     * @param max_iter 最大迭代次数
     * @return
     */
    bool cubicEquationSolve(double &result, const Eigen::Vector4d &coeff_matrix, double init, int max_iter)
    {
        double xi = init;
        double f_xi = coeff_matrix(0)*xi*xi*xi + coeff_matrix(1)*xi*xi + coeff_matrix(2)*xi + coeff_matrix(3);
        for (int i=0; i<max_iter; i++)
        {
            double df_xi = 3*coeff_matrix(0)*xi*xi + 2*coeff_matrix(1)*xi + coeff_matrix(2);
            double dx = -f_xi / (df_xi + 1e-6);
            xi += dx;
            f_xi = coeff_matrix(0)*xi*xi*xi + coeff_matrix(1)*xi*xi + coeff_matrix(2)*xi + coeff_matrix(3);
            printf("\033[31miter: %02d, f_xi: %06.4f, xi: %06.4f\n\033[0m", i, f_xi, xi);
            if (fabs(f_xi) < 1e-6)
            {
                result = xi;
                return true;
            }
        }
        result = init;
        return false;
    }


    /**
     * @brief 更新这个样条曲线的一些基本信息
     * @param pts
     */
    void Spline::updateCltPoints(const Eigen::Matrix<double, 4, 2> &pts)
    {
        clt_points = pts;
        Eigen::Matrix4d M;
        M << -1,  3, -3,  1,
              3, -6,  3,  0,
             -3,  3,  0,  0,
              1,  0,  0,  0;
        coeff_matrix_ = M*clt_points;
        double d01 = hypot(pts(1,0)-pts(0,0), pts(1,1)-pts(0,1));
        double d12 = hypot(pts(2,0)-pts(1,0), pts(2,1)-pts(1,1));
        double d23 = hypot(pts(3,0)-pts(2,0), pts(3,1)-pts(2,0));
        d_sum = d01 + d12 + d23;
        d_ij << d01, d12, d23;
    }


    /**
     * @brief 计算pt点同x轴上, 样条曲线上的点的y值
     * @param pt
     * @return
     */
    bool Spline::computerVerticalDistance(Eigen::Vector2d &pt, double &result)
    {
        assert(pt(0) > clt_points(0,0));
//        assert(pt(0) < clt_points(3,0));

        // buid cubic equation
        Eigen::Vector4d coeff_x = coeff_matrix_.block(0,0,4,1);
        coeff_x(3) -= pt(0);

        // compute initial t
        double d_0x = 0;
        int idx = 0;
        while (idx < 3 && pt(0) > clt_points(idx+1, 0))
            d_0x += d_ij(idx++);
        cout << "---" << endl;
        cout << "init d_0x: " << d_0x;
        d_0x = d_0x + hypot(pt(0)-clt_points(idx,0), pt(1)-clt_points(idx,1));
        double t_init = d_0x / d_sum;
        cout << ", t_init: " << t_init << ", coeff: " << coeff_x.transpose() << endl;
        cout << "control points: " << endl;
        cout << clt_points << endl;

        // solve t
        double t_final = t_init;
        bool solve_success = true;
        solve_success = cubicEquationSolve(t_final, coeff_x, t_init);
        Eigen::Vector4d T;
        T << t_final*t_final*t_final, t_final*t_final, t_final, 1;
        result = (T.transpose()*coeff_matrix_.block(0,1,4,1))(0);
        return solve_success;
    }


    SplineFitter::SplineFitter():
    degree_(3), sample_n_(0), max_iters_(10)
    {}


    void SplineFitter::addPoints(const vector<Eigen::Vector2d> &pt_vec)
    {
        fit_scatters_ = pt_vec;
    }


    void SplineFitter::addPoint(const Eigen::Vector2d &pt)
    {
        fit_scatters_.push_back(pt);
    }


    /**
     * @brief spline fit求解接口
     * @param sp
     */
    void SplineFitter::solve(Spline &sp)
    {
        // TODO: 目前没有使用RANSAC
        Eigen::MatrixX2d scatter_points(fit_scatters_.size(), 2);
        sample_n_ = fit_scatters_.size();
        for (int i=0; i<fit_scatters_.size(); i++)
            scatter_points.block(i,0,1,2) << fit_scatters_[i](0), fit_scatters_[i](1);
        fitSpline(scatter_points, sp);
    }


    /**
     * @brief 从待拟合点集合中随机采样
     * @param sample_pts
     */
    void SplineFitter::getRandomSample(Eigen::MatrixX2d &sample_pts)
    {
        if (fit_scatters_.size() <= sample_n_)
        {
            for (int i=0; i<fit_scatters_.size(); i++)
                sample_pts.block(i,0,1,2) << fit_scatters_[i](0), fit_scatters_[i](0);
            return;
        }

        // random sample
        int cnt = 0;
        vector<int> sample_idxs;
        while (cnt < sample_n_)
        {
            int rs = rand() % sample_n_;
            if (find(sample_idxs.begin(), sample_idxs.end(), rs) != sample_idxs.end())
                continue;
            sample_idxs.push_back(rs);
            cnt++;
        }

        // sort and build matrix
        sort(sample_idxs.begin(), sample_idxs.end());
        for (int i=0; i<sample_idxs.size(); i++)
            sample_pts.block(i,0,1,4) << fit_scatters_[i](0), fit_scatters_[i](1);
    }


    /**
     * @brief 使用给定点集合进行spline fit
     * @param sample_pts
     * @param sp
     * @return
     */
    bool SplineFitter::fitSpline(const Eigen::MatrixX2d &sample_pts, Spline &sp)
    {
        assert(sample_pts.rows() == sample_n_);
        assert(degree_ == 3);

        sp.degree = degree_;

        // start point
        Eigen::Vector2d st_pt;
        st_pt << sample_pts(0,0), sample_pts(0,1);

        // end point
        Eigen::Vector2d ed_pt;
        ed_pt << sample_pts(sample_n_-1,0), sample_pts(sample_n_-1,1);

        // T matrix
        Eigen::MatrixX4d T(sample_n_, 4);
        Eigen::VectorXd d_ij(sample_n_-1);
        T.block(0,0,1,4) = Eigen::Matrix<double, 1, 4>::Zero();
        T.block(sample_n_-1,0,1,4) = Eigen::Matrix<double, 1, 4>::Ones();

        // distance
        for (int i=1; i<sample_n_; i++)
            d_ij(i-1) = hypot( sample_pts(i,0)-sample_pts(i-1,0), sample_pts(i,1)-sample_pts(i-1,1) );
        double sum_t_all = d_ij.sum();
        double sum_t_i = 0;

        // build T matrix
        for (int i=1; i<sample_n_-1; i++)
        {
            sum_t_i += d_ij(i-1);
            double t = sum_t_i / sum_t_all;
            T.block(i,0,1,4) << t*t*t, t*t, t, 1.;
        }

        // Bezier Matrix
        Eigen::Matrix4d M;
        M << -1,  3, -3,  1,
              3, -6,  3,  0,
             -3,  3,  0,  0,
              1,  0,  0,  0;
        Eigen::MatrixX4d B = T*M;

        // solve control points
        Eigen::MatrixX2d P(4,2);
        P = B.colPivHouseholderQr().solve(sample_pts);

        // start and end point should not be changed
        P.block(0,0,1,2) = sample_pts.block(0,0,1,2);
        P.block(3,0,1,2) = sample_pts.block(sample_n_-1,0,1,2);
        sp.updateCltPoints(P);
    }


    /**
     * @brief 给拟合出来的spline进行打分
     * @param sp
     * @return
     */
    double SplineFitter::comupteScore(Spline &sp)
    {

    }
}