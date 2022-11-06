#pragma once
#include <math.h>
#include<algorithm>
#include <iostream>
#include <vector>

using namespace std;

class ReferenceLine {
 public:
  ReferenceLine(const std::vector<std::pair<double, double>>& xy_points);
  ~ReferenceLine() = default;

//   vector<vector<double>> transposeInPlace(vector<vector<double>>& m);

  void referenceLine(std::vector<std::pair<double, double>>const& xy_points);

  bool ComputePathProfile(std::vector<double>* headings,
                          std::vector<double>* accumulated_s,
                          std::vector<double>* kappas,
                          std::vector<double>* dkappas);
 private:
  std::vector<std::pair<double, double>> xy_points_;
};