#pragma once
#include <Eigen/Eigen>
#include <vector>

#include "OsqpEigen/OsqpEigen.h"
#include "matplotlibcpp.h"


namespace plt = matplotlibcpp;
using namespace std;

class refercenceline_smooth {
public:
  refercenceline_smooth() = default;
  ~refercenceline_smooth() =default;

  void ReferenceLine(const std::vector<std::pair<double, double>> &xy_points);

  bool line_smooth(double w_cost_smooth, double w_cost_length,
                   double w_cost_ref, double x_lb, double x_ub, double y_lb,
                   double y_ub);

  //输入 referenceline x,y,init未平滑的参考线
  Eigen::MatrixXd referenceline_x_init = Eigen::MatrixXd::Zero(5, 5);
  Eigen::MatrixXd referenceline_y_init = Eigen::MatrixXd::Zero(5, 5);
  //平滑代价，紧凑代价，几何相似代价权重系数
  double w_cost_smooth = 0;
  double w_cost_length = 0;
  double w_cost_ref = 0;
  //允许x,y变化的上下界
  double x_lb = 0;
  double x_ub = 0;
  double y_lb = 0;
  double y_ub = 0;

  //输出
  std::vector<double> ref_x;
  std::vector<double> ref_y;

  Eigen::MatrixXd referenceline_x;
  Eigen::MatrixXd referenceline_y;
  Eigen::MatrixXd A1;
  Eigen::MatrixXd A2;
  Eigen::MatrixXd A3;
  Eigen::MatrixXd f;
  Eigen::MatrixXd lb;
  Eigen::MatrixXd ub;
  Eigen::MatrixXd pre_result_x;
  Eigen::MatrixXd pre_result_y;

private:
  std::vector<std::pair<double, double>> xy_points_;
};
