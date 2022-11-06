#pragma once
#include "matching_line.h"

class ExtractPoint {
public:
    ExtractPoint() = default;
    ~ExtractPoint() = default;
    void axtractglobalpath(std::vector<std::pair<double, double>>const& xy_points);
    void axtractrefercenceline(int host_match_point_index);

    //输出
    std::vector<double> referenceline_x_init;
    std::vector<double> referenceline_y_init;
private:    
    std::vector<std::pair<double, double>> xy_points_;
};