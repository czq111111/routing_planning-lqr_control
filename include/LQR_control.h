#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>

#include "struct_common.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "Eigen/Core"

using Matrix = Eigen::MatrixXd;

class LQR_control
{
private:
    // the following parameters are vehicle physics related.
    // control time interval
    double ts_ = 0.0;
    // corner stiffness; front
    double cf_ = 0.0;
    // corner stiffness; rear
    double cr_ = 0.0;
    // distance between front and rear wheel center
    double wheelbase_ = 0.0;
    // mass of the vehicle
    double mass_ = 0.0;
    // distance from front wheel center to COM
    double lf_ = 0.0;
    // distance from rear wheel center to COM
    double lr_ = 0.0;
    // rotational inertia
    double iz_ = 0.0;
    // the ratio between the turn of the steering wheel and the turn of the wheels
    double steer_ratio_ = 0.0;
    // the maximum turn of steer
    double steer_single_direction_max_degree_ = 0.0;

    // parameters for lqr solver; number of iterations
    int lqr_max_iteration_ = 0;
    // parameters for lqr solver; threshold for computation
    double lqr_eps_ = 0.0;

    // number of states without previews, includes
    // lateral error, lateral error rate, heading error, heading error rate
    const int basic_state_size_ = 4;
    // vehicle state matrix
    Eigen::MatrixXd matrix_a_;
    // vehicle state matrix (discrete-time)
    Eigen::MatrixXd matrix_ad_;
    // control matrix
    Eigen::MatrixXd matrix_b_;
    // control matrix (discrete-time)
    Eigen::MatrixXd matrix_bd_;
    // gain matrix
    Eigen::MatrixXd matrix_k_;
    // control authority weighting matrix
    Eigen::MatrixXd matrix_r_;
    // state weighting matrix
    Eigen::MatrixXd matrix_q_;
    // updated state weighting matrix
    Eigen::MatrixXd matrix_q_updated_;
    // vehicle state matrix coefficients
    Eigen::MatrixXd matrix_a_coeff_;
    // 4 by 1 matrix; state matrix
    Eigen::MatrixXd matrix_state_;

    //最近的轨迹点上的曲率
    double ref_curv_;
    double ref_curv_front_;

    std::vector<planning_trajectory> planning_path;
    typedef std::shared_ptr<LateralControlError> LateralControlErrorPtr;

public:
    void LoadControlConf(); //加载车辆信息
    void Init();    //初始化状态矩阵
    double ComputeControlCommand(const location_info &VehicleState,
                                 const std::vector<planning_trajectory> &planning_published_trajectory,
                                 struct ControlCmd &cmd);   //计算控制大小
    
    planning_trajectory QueryNearestPointByPosition(const double x, const double y);
    void ComputeErrors(const double x,
                       const double y,
                       const double theta,
                       const double linear_v,
                       const double angular_v,
                       const double linear_a,
                       LateralControlErrorPtr &lat_con_error);  //计算误差
    void UpdataState(const location_info &VehicleState);
    void SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q,
                         const Matrix &R, const double tolerance,
                         const uint max_num_iteration, Matrix *ptr_K);  //计算K矩阵
    double ComputeFeedForward(const location_info &VehicleState,
                              double ref_curvature);  //计算前馈控制
};
