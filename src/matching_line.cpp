#include "matching_line.h"


void MatchingToFrenet::tofrenetreferenceLine(
    std::vector<std::pair<double, double>> const &xy_points,
    std::vector<std::pair<double, double>> const &xy_set) {
  xy_points_ = xy_points;
  xy_set_ = xy_set;
}

void MatchingToFrenet::tofrenet(std::vector<double> *headings,
                                std::vector<double> *kappas) {
  // int n = xy_set_.size();
  int n = 128;
  //输出初始化
  match_point_index_set = Eigen::MatrixXd::Ones(n, 1) * NAN;
  proj_x_set = Eigen::MatrixXd::Ones(n, 1) * NAN;
  proj_y_set = Eigen::MatrixXd::Ones(n, 1) * NAN;
  proj_heading_set = Eigen::MatrixXd::Ones(n, 1) * NAN;
  proj_kappa_set = Eigen::MatrixXd::Ones(n, 1) * NAN;

  int is_first_run=0;
  std::vector<double> pre_frenet_path_x;
  std::vector<double> pre_frenet_path_y;
  std::vector<double> pre_frenet_path_heading;
  std::vector<double> pre_frenet_path_kappa;
  std::vector<double> pre_x_set;
  std::vector<double> pre_y_set;
  int size = xy_set_.size();
  if (is_first_run==0) {
    
    //cout<<"首次运行"<<endl;
    //该if分支表示函数首次运行
    is_first_run = 1;
    
    for (int i = 0; i < (int)xy_points_.size(); i++) {
      pre_frenet_path_x.push_back(xy_points_[i].first);
      pre_frenet_path_y.push_back(xy_points_[i].second);
      pre_frenet_path_heading.push_back(headings->at(i));
      pre_frenet_path_kappa.push_back(kappas->at(i));
    } 
    //pre_match_point_index_set = Eigen::MatrixXd::Ones(n, 1) * NAN;
    for (int i = 0; i < size; i++) {
      if (size == 0) {
        cout<<"没有数据点"<<endl;
        return;
      }
      //首次运行时，没有任何提示，只能从frenet path的第一个点开始找
      int start_search_index = 0;
      //int fp_size = xy_points_.size();
      //声明increase_count，用于表示在遍历时distance连续增加的个数
      int increase_count = 0;
      //开始遍历
      double min_distance = INFINITY;
      for (int j = start_search_index; j < (int)xy_points_.size(); j++) {
        double distance = (xy_set_[i].first - xy_points_[j].first) *
                              (xy_set_[i].first - xy_points_[j].first) +
                          (xy_set_[i].second - xy_points_[j].second) *
                              (xy_set_[i].second - xy_points_[j].second);
        // cout<<"distance: "<<distance<<endl;
        if (distance < min_distance) {
          min_distance = distance;
          match_point_index_set(i, 0) = j;
          increase_count = 0;
        } else {
          increase_count = increase_count + 1;
        }
        if (increase_count > 50) {
          break;
        }
      }
 
      //取出匹配点的编号
      int match_point_index = match_point_index_set(i, 0);
      //cout<<"match_point_index: "<<match_point_index<<endl;
      //取出匹配点的信息
      double match_point_x = xy_points_[match_point_index].first;
      double match_point_y = xy_points_[match_point_index].second;

      // match_point_heading.push_back((*headings)[i]);
      double match_point_heading = headings->at(match_point_index);
      double match_point_kappa = kappas->at(match_point_index);
      Eigen::MatrixXd vector_match_point;
      Eigen::MatrixXd vector_match_point_direction;
      Eigen::MatrixXd vector_r;
      vector_match_point = Eigen::MatrixXd::Zero(2, 1);
      vector_match_point_direction = Eigen::MatrixXd::Zero(2, 1);
      vector_r = Eigen::MatrixXd::Zero(2, 1);

      //计算匹配点的方向向量

      vector_match_point(0 , 0) = match_point_x;
      vector_match_point(1 , 0) = match_point_y;
      //声明待投影点的与法向量

      vector_match_point_direction(0, 0) = cos(match_point_heading);
      vector_match_point_direction(1, 0) = sin(match_point_heading);
      //声明待投影点的位矢

      vector_r(0, 0) = xy_set_[i].first;
      vector_r(1, 0) = xy_set_[i].second;

      //通过匹配点计算投影点
      Eigen::MatrixXd vector_d;
      Eigen::MatrixXd ds;
      Eigen::MatrixXd vector_proj_point;
      
      vector_d = Eigen::MatrixXd::Zero(2, 1);
      ds = Eigen::MatrixXd::Zero(1, 1);
      vector_proj_point = Eigen::MatrixXd::Zero(2, 1);
      
      vector_d = vector_r - vector_match_point;
      ds = vector_d.transpose() * vector_match_point_direction;

      vector_proj_point = vector_match_point + ds(0,0) * vector_match_point_direction;
      //cout<<"-------------------------- "<<endl;
      double proj_heading = match_point_heading + match_point_kappa * ds(0, 0);
      //cout<<"1111111111111111111111111111111 "<<endl;
      double proj_kappa = match_point_kappa;
      //计算结果输出
      proj_x_set(i, 0) = vector_proj_point(0, 0);
      proj_y_set(i, 0) = vector_proj_point(1, 0);
      proj_heading_set(i, 0) =proj_heading;
      proj_kappa_set(i, 0) = proj_kappa;
    }
    //匹配点的计算结果保存，供下一个周期使用
    
    for (int i = 0; i < (int)xy_set_.size(); i++) {
      pre_x_set.push_back(xy_set_[i].first);
      pre_y_set.push_back(xy_set_[i].second);
    }
    for(int i=0;i<(int)match_point_index_set.size();i++)
    {
      pre_match_point_index_set.push_back(match_point_index_set(i,0));
    }
    
    //std::cout << "pre_match_point_index_set.size"<< pre_match_point_index_set.size() << endl;
    //std::cout << "----------match_point_index_set: ----" << match_point_index_set << endl;
    // for (int i = 0; i < size; i++) 
    // {
    //   cout << "pre_x_set " << pre_x_set[i] << endl;
    //   cout << "pre_y_set " << pre_y_set[i] << endl;
    // }
    #if 0
    std::vector<double> x, y;
    x.resize(pre_x_set.size());
    y.resize(pre_x_set.size());
    for (int i = 0; i < 10; i++) {
      x.push_back(pre_x_set[i]);
      y.push_back(pre_y_set[i]);
    }
    plt::plot(x, y);
    // plt::xlim(0, 10 * 10);
    plt::title("line");
    plt::legend();
    plt::show();
    #endif
  } 
  else 
  {
    //此if分支表示不是首次运行
    //对每个x_set上的点做处理
    cout<<"不是首次运行"<<endl;
    for (int i = 0; i < size; i++) 
    {
      double start_search_index = pre_match_point_index_set.at(i); //上个周期匹配点的编号作为本周期搜索的起点
      //判断帧与帧之间的障碍物是否为同一个
      double square_dis = (xy_set_[i].first - pre_x_set[i]) *
                              (xy_set_[i].first - pre_x_set[i]) +
                          (xy_set_[i].second - pre_y_set[i]) *
                              (xy_set_[i].second - pre_y_set[i]);
      if (square_dis > 36) {
        //帧与帧之间的物体的位置距离过大，认为是两个障碍物
        start_search_index = -1;
      }
      /* 声明increase_count，用于表示在遍历时distance连续增加的个数
         %%%%%%%%
         对于障碍物检测而言，当感知第一次检测到障碍物时，算法并不是首次运行，此时
         pre_match_point_index_set的值是nan，如果还用上一时刻的结果作为本周期搜索的起点
         必然会出问题，所以要修改
      */
      int increase_count_limit = 5;

      if (start_search_index == -1) {
        //没有上个周期的结果，那就不能只检查5次了
        increase_count_limit = 50;
        //搜索起点也要设为1
        start_search_index = 0;
      }
      int increase_count = 0;
      //开始遍历
      //这里多一个步骤，判断遍历的方向
      //计算上个周期匹配点的位矢
      Eigen::MatrixXd vector_pre_match_point;
      Eigen::MatrixXd vector_pre_match_point_direction;
      vector_pre_match_point = Eigen::MatrixXd::Zero(2, 1);
      vector_pre_match_point_direction = Eigen::MatrixXd::Zero(2, 1);

      vector_pre_match_point(0, 0) = pre_frenet_path_x.at(start_search_index);
      vector_pre_match_point(1, 0) = pre_frenet_path_y.at(start_search_index);
      vector_pre_match_point_direction(0, 0) =
          cos(pre_frenet_path_heading.at(start_search_index));
      vector_pre_match_point_direction(1, 0) =
          sin(pre_frenet_path_heading.at(start_search_index));

      // vector< vector<double> > delta_xy(2, vector<double>(xy_points_.size(),
      // 0));
      Eigen::MatrixXd delta_xy;
      delta_xy = Eigen::MatrixXd::Zero(2, 1);
      delta_xy(0, 0) = xy_set_[i].first - vector_pre_match_point(0, 0);
      delta_xy(1, 0) = xy_set_[i].second - vector_pre_match_point(1, 0);
      Eigen::MatrixXd flag;
      flag = Eigen::MatrixXd::Zero(1, 1);
      flag = delta_xy.transpose() * vector_pre_match_point_direction;
      double min_distance = INFINITY;
      double flag_val = flag(0, 0);
      //判断遍历方向
      if (flag_val > 0.001) {
        for (int j = start_search_index; j < (int)xy_points_.size(); j++) {
          double distance = (xy_set_[i].first - xy_points_[j].first) *
                                (xy_set_[i].first - xy_points_[j].first) +
                            (xy_set_[i].second - xy_points_[j].second) *
                                (xy_set_[i].second - xy_points_[j].second);
          if (distance < min_distance) {
            min_distance = distance;
            match_point_index_set(i, 0) = j;
            increase_count = 0;
          } else {
            increase_count = increase_count + 1;
          }
          //如果distance连续增加increase_count_limit次就不要再遍历了，节省时间
          if (increase_count > increase_count_limit) {
            break;
          }
        }
      } 
      else if (flag_val < -0.001) 
      {
        for (int j = start_search_index; j > 1; j--) {
          double distance = (xy_set_[i].first - xy_points_[j].first) *
                                (xy_set_[i].first - xy_points_[j].first) +
                            (xy_set_[i].second - xy_points_[j].second) *
                                (xy_set_[i].second - xy_points_[j].second);
          if (distance < min_distance) {
            min_distance = distance;
            match_point_index_set(i, 0) = j;
            increase_count = 0;
          } else {
            increase_count = increase_count + 1;
          }
          //如果distance连续增加increase_count_limit次就不要再遍历了，节省时间
          if (increase_count > increase_count_limit) {
            break;
          }
        }

      } 
      else 
      {
        match_point_index_set(i, 0) = start_search_index;
      }
      //取出匹配点的编号
      int match_point_index = match_point_index_set(i, 0);
      cout<<"match_point_index: "<<match_point_index<<endl;
      //取出匹配点的信息
      double match_point_x = xy_points_[match_point_index].first;
      double match_point_y = xy_points_[match_point_index].second;

      // match_point_heading.push_back((*headings)[i]);
      double match_point_heading = headings->at(match_point_index);
      double match_point_kappa = kappas->at(match_point_index);
      Eigen::MatrixXd vector_match_point;
      Eigen::MatrixXd vector_match_point_direction;
      Eigen::MatrixXd vector_r;
      vector_match_point = Eigen::MatrixXd::Zero(2, 1);
      vector_match_point_direction = Eigen::MatrixXd::Zero(2, 1);
      vector_r = Eigen::MatrixXd::Zero(2, 1);

      //计算匹配点的方向向量

      vector_match_point(0 , 0) = match_point_x;
      vector_match_point(1 , 0) = match_point_y;
      //声明待投影点的与法向量

      vector_match_point_direction(0, 0) = cos(match_point_heading);
      vector_match_point_direction(1, 0) = sin(match_point_heading);
      //声明待投影点的位矢

      vector_r(0, 0) = xy_set_[i].first;
      vector_r(1, 0) = xy_set_[i].second;

      //通过匹配点计算投影点
      Eigen::MatrixXd vector_d;
      Eigen::MatrixXd ds;
      Eigen::MatrixXd vector_proj_point;

      vector_d = Eigen::MatrixXd::Zero(2, 1);
      ds = Eigen::MatrixXd::Zero(1, 1);
      vector_proj_point = Eigen::MatrixXd::Zero(2, 1);

      vector_d = vector_r - vector_match_point;
      ds = vector_d.transpose() * vector_match_point_direction;
      vector_proj_point =
          vector_match_point + ds * vector_match_point_direction;
      double proj_heading = match_point_heading + match_point_kappa * ds(0, 0);
      double proj_kappa = match_point_kappa;
      //计算结果输出
      proj_x_set(i, 0) = vector_proj_point(0, 0);
      proj_y_set(i, 0) = vector_proj_point(1, 0);
      proj_heading_set(i, 0) =proj_heading;
      proj_kappa_set(i, 0) = proj_kappa;
    }
    //匹配点的计算结果保存，供下一个周期使用
    for (int i = 0; i < (int)xy_set_.size(); i++) 
    {
      pre_x_set.push_back(xy_set_[i].first);
      pre_y_set.push_back(xy_set_[i].second);
    }
    //pre_match_point_index_set = match_point_index_set;
    for(int i=0;i<(int)match_point_index_set.size();i++)
    {
      pre_match_point_index_set.push_back(match_point_index_set(i,0));
    }
    std::cout << "match_point_index_set: " << match_point_index_set << endl;

    for (int i = 0; i < (int)xy_points_.size(); i++) 
    {
      pre_frenet_path_x.push_back(xy_points_[i].first);
      pre_frenet_path_y.push_back(xy_points_[i].second);
      pre_frenet_path_heading.push_back(headings->at(i));
      pre_frenet_path_kappa.push_back(kappas->at(i));
    }
    // cout << " pre_match_point_index_set " << match_point_index_set(0, 0)
    //      << endl;
    #if 0
    std::vector<double> x, y;
    x.resize(pre_x_set.size());
    y.resize(pre_x_set.size());
    for (int i = 0; i < 10; i++) {
      x.push_back(pre_x_set[i]);
      y.push_back(pre_y_set[i]);
    }
    plt::plot(x, y);
    // plt::xlim(0, 10 * 10);
    plt::title("line");
    plt::legend();
    plt::show();
    #endif
  }
} 

