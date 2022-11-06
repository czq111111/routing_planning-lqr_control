#pragma once
#include <limits> 
#include <vector>

typedef std::numeric_limits<double> Info;

// 车辆信息
 struct location_info {
     double host_x;
     double host_y;
     double host_heading_xy;
     double host_yawrate;
     double host_speed;
     double host_vx;
     double host_vy;
     double host_acceleration;
     double host_ax;
     double host_ay;
 };

 struct xy_points_info{
    double x;
    double y;
 }; 

 // 障碍物信息
 struct obstacle_info {
     double obs_x;
     double obs_y;
     double obs_heading;
     double obs_velocity;
 };

 // 
 struct PathProfile_info{
      double headings;
      double accumulated_s;
      double kappas;
      double dkappas;
 };

 // 路径点的信息
 struct points_conf {
   double x;
   double y;
   double heading;
   double kappa;
 };

 // 轨迹规划的结果
 struct planning_trajectory
 {
  double trajectory_x;
  double trajectory_y;
  double trajectory_heading;
  double trajectory_kappa;
  double trajectory_speed;
  double trajectory_accel;
  double trajectory_time;
 };

//拼接轨迹结构体
 struct stitch_trajectory
{
    double stitch_x;
    double stitch_y;
    double stitch_heading;
    double stitch_kappa;
    double stitch_speed;
    double stitch_accel;
    double stitch_time;
};
struct plan_start_info
{
    double plan_start_x;
    double plan_start_y;
    double plan_start_vx;
    double plan_start_vy;
    double plan_start_heading;
    double plan_start_ax;
    double plan_start_ay;
    double plan_start_kappa;
    double plan_start_time;
};

//投影点的结构体
struct proj_points_info
{
    double proj_match_index;
    double proj_point_x;
    double proj_point_y;
    double proj_heading;
    double proj_kappa;
};

struct ControlCmd {
    double steer_target;
    double acc;
};

struct LateralControlError {
    double lateral_error;       // 横向误差
    double heading_error;       // 转向误差
    double lateral_error_rate;  // 横向误差速率
    double heading_error_rate;  // 转向误差速度
};