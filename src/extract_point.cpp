#include "extract_point.h"

void ExtractPoint::axtractglobalpath(std::vector<std::pair<double, double>>const& xy_points)
{
    xy_points_ = xy_points;
}

void ExtractPoint::axtractrefercenceline(int host_match_point_index)
{
    int start_index = -1;
    //判断后面前面的点是否足够多
    if(host_match_point_index -30 <1)
    {
        //匹配点后面的点太少了，不够30个
        start_index = 0;
    }
    else if(host_match_point_index +150 > (int)xy_points_.size())
    {
        //匹配点前面的点太少了，不够150个
        start_index = xy_points_.size() - 180;
    }
    else{
        //匹配点正常情况
        start_index = host_match_point_index - 30;
    }
    //提取referenceline
    for(int i=start_index;i<start_index+180;i++)
    {
        referenceline_x_init.push_back(xy_points_.at(i).first);
        referenceline_y_init.push_back(xy_points_.at(i).second);
    }
    
}