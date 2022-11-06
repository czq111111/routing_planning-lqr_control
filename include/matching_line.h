#pragma once
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <algorithm>

#include "matplotlibcpp.h"

using Eigen::MatrixXd;
using namespace std;
namespace plt = matplotlibcpp;
class MatchingToFrenet{
public:
    //导入参考线坐标和自身坐标
    void tofrenetreferenceLine(std::vector<std::pair<double, double>>const& xy_points,
                               std::vector<std::pair<double, double>>const& xy_set);

    //实现坐标转换
    void tofrenet(std::vector<double>* headings, std::vector<double>* kappas);


    //输出
    Eigen::MatrixXd match_point_index_set;
    Eigen::MatrixXd proj_x_set;
    Eigen::MatrixXd proj_y_set;
    Eigen::MatrixXd proj_heading_set;
    Eigen::MatrixXd proj_kappa_set;

    //Eigen::MatrixXd pre_match_point_index_set;
    std::vector<double>pre_match_point_index_set;



private:
    std::vector<std::pair<double, double>> xy_points_;
    std::vector<std::pair<double, double>> xy_set_;
};