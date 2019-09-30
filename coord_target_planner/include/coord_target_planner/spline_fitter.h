//
// Created by yonghui on 19-9-4.
//

#ifndef COORD_MOVE_BASE_RANSACSPLINEFITTER_H
#define COORD_MOVE_BASE_RANSACSPLINEFITTER_H

#include <Eigen/Core>
#include <vector>
using namespace std;

namespace coord_target_planner
{
    bool cubicEquationSolve(double &result, const Eigen::Vector4d &coeff_matrix, double init=0, int max_iter=10);

    struct Spline
    {
        bool computerVerticalDistance(Eigen::Vector2d &pt, double &result);

        void updateCltPoints(const Eigen::Matrix<double, 4, 2> &pts);

        int degree;
        double score;
        Eigen::Matrix<double, 4, 2> clt_points;
        Eigen::Matrix<double, 4, 2> coeff_matrix_;
        Eigen::Vector3d d_ij;
        double d_sum;
    };


    class SplineFitter
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        SplineFitter();

        void addPoints(const vector<Eigen::Vector2d> &pt_vec);

        void addPoint(const Eigen::Vector2d &pt);

        void solve(Spline &sp);

    protected:
        void getRandomSample(Eigen::MatrixX2d &sample_pts);

        bool fitSpline(const Eigen::MatrixX2d &sample_pts, Spline &sp);

        double comupteScore(Spline &sp);

        void resetState();

        vector<Eigen::Vector2d> fit_scatters_;
        int degree_;
        int sample_n_;
        int max_iters_;
    };
}

#endif //COORD_MOVE_BASE_RANSACSPLINEFITTER_H
