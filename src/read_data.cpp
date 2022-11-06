#include "read_data.h"
#include <Eigen/Eigen>

// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;


bool readDate::readTxt(std::vector<std::pair<double, double>> &xy_points) {
  std::ifstream infile;
  infile.open(
      "/home/czq/refercence_line/data/cube_town_record_line.txt"); //将文件流对象与文件连接起来
  if (!infile.is_open()) {
    return false;
  }
  //   std::vector<std::pair<double, double>> xy_points;
  std::string s, x, y;
  while (getline(infile, s)) {
    std::stringstream word(s);
    word >> x;
    word >> y;
    double pt_x = std::atof(x.c_str());
    double pt_y = std::atof(y.c_str());
    xy_points.push_back(std::make_pair(pt_x, pt_y));
  }
  return true;
}

// void write_to_csv(std::vector<double> &r_x_, std::vector<double> &r_y_) {
//   std::ofstream outFile;
//   outFile.open("solve_path_points.csv", std::ios::out);
//   outFile << "x_value" << ',' << "y_value" << '\n';
//   for (size_t i = 0; i < r_x_.size(); ++i) {
//     outFile << r_x_.at(i) << ',' << r_y_.at(i) << '\n';
//   }
//   outFile.close();
// }