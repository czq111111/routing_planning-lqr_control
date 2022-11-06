#include "refercenceline_smooth.h"

void refercenceline_smooth::ReferenceLine(
    const std::vector<std::pair<double, double>> &xy_points) {
  xy_points_ = xy_points;
}

bool refercenceline_smooth::line_smooth(double w_cost_smooth,
                                        double w_cost_length, double w_cost_ref,
                                        double x_lb, double x_ub, double y_lb,
                                        double y_ub) {
  //初始化
  // int n = xy_points_.size();
  int n=180;
  referenceline_x = Eigen::MatrixXd::Zero(n, 1);
  referenceline_y = Eigen::MatrixXd::Zero(n, 1);
  A1 = Eigen::MatrixXd::Zero(2 * n - 4, 2 * n);
  A2 = Eigen::MatrixXd::Zero(2 * n - 2, 2 * n);
  A3 = Eigen::MatrixXd::Zero(2 * n, 2 * n);
  f = Eigen::MatrixXd::Zero(2 * n, 1);
  //全局变量
  int is_first_run = 0;

  Eigen::MatrixXd pre_referenceline_x_init;
  Eigen::MatrixXd pre_referenceline_y_init;
  if (is_first_run == 0) {
    is_first_run = 1;
    for (int j = 0; j < 2 * n - 5; j++) {
      A1(j, j) = 1;
      A1(j, j + 2) = -2;
      A1(j, j + 4) = 1;
      A1(j + 1, j + 1) = 1;
      A1(j + 1, j + 3) = -2;
      A1(j + 1, j + 5) = 1;
    }
    for (int k = 0; k < 2 * n - 3; k++) {
      A2(k, k) = 1;
      A2(k, k + 2) = -1;
      A2(k + 1, k + 1) = 1;
      A2(k + 1, k + 3) = -1;
    }
    for (int i = 0; i < 2 * n; i++) {
      A3(i, i) = 1;
    }

    // cout << "A1: " << A1 << endl;
    // cout << "A2: " << A2 << endl;

    Eigen::MatrixXd H;
    //
    Eigen::SparseMatrix<double> H1_d;         // H
    Eigen::VectorXd f_d;                      // f
    Eigen::VectorXd lb_d;                     //下边界
    Eigen::VectorXd ub_d;                     //上边界
    Eigen::SparseMatrix<double> linearMatrix; // A
    H = 2 * (w_cost_smooth * (A1.transpose() * A1) +
             w_cost_length * (A2.transpose() * A2) + w_cost_ref * A3);

    //设置H矩阵
    H1_d.resize(H.rows(), H.cols());
    for (int i = 0; i < H.rows(); i++) {
      for (int j = 0; j < H.cols(); j++) {
        H1_d.insert(i, j) = H(i, j);
      }
    }
    //设置f矩阵
    f_d.resize(2 * n);
    for (int k = 0; k < n; k++) {
      f_d(2 * k) = -2 * w_cost_ref * xy_points_[k].first;
      f_d(2 * k + 1) = -2 * w_cost_ref * xy_points_[k].second;
    }
    //设置A矩阵
    linearMatrix.resize(2 * n, 2 * n);
    for (int i = 0; i < 2 * n; i++) {
      linearMatrix.insert(i, i) = 1;
    }
    //设置上下边界
    lb_d.resize(2 * n);
    ub_d.resize(2 * n);
    for (int i = 0; i < n; i++) {
      f(2 * i, 0) = xy_points_[i].first;
      f(2 * i + 1, 0) = xy_points_[i].second;
      lb_d(2 * i) = f(2 * i, 0) - x_lb;
      ub_d(2 * i) = f(2 * i, 0) + x_ub;
      lb_d(2 * i + 1) = f(2 * i + 1, 0) - y_lb;
      ub_d(2 * i + 1) = f(2 * i + 1, 0) + y_ub;
    }
    // std::cout << "hessian:" << std::endl << H1_d << std::endl;
    // std::cout << "f_d:" << std::endl << f_d << std::endl;
    // std::cout << "hessian:" << std::endl << f_d << std::endl;
    // std::cout << "hessian:" << std::endl << ub_d << std::endl;

    int NumBerOfVaribles = 2 * n;
    int NumberOfConstraints = 2 * n;
    //二次规划求解
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    // 设置QP解算器的初始数据
    //矩阵A为m*n矩阵
    solver.data()->setNumberOfVariables(NumBerOfVaribles);
    solver.data()->setNumberOfConstraints(NumberOfConstraints);
    if (!solver.data()->setHessianMatrix(H1_d))
      return 1; //设置P矩阵
    if (!solver.data()->setGradient(f_d))
      return 1; //设置q or f矩阵。当没有时设置为全0向量
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
      return 1;
    if (!solver.data()->setLowerBound(lb_d))
      return 1; //设置下边界
    if (!solver.data()->setUpperBound(ub_d))
      return 1; //设置上边界
    if (!solver.initSolver())
      return 1;
    Eigen::VectorXd QPSolution;
    if (!solver.solve())
      return 1;
    QPSolution = solver.getSolution();
    // std::cout << "QPSolution:" << std::endl << QPSolution << std::endl;
    // std::cout << "QPSolution_Size:" << std::endl << QPSolution.size() << std::endl;
    for (int i = 0; i < n; i++) {
      referenceline_x(i,0) = QPSolution(2 * i);
      referenceline_y(i,0) = QPSolution(2 * i + 1);
    }
    pre_result_x = referenceline_x;
    pre_result_y = referenceline_y;
    pre_referenceline_x_init = referenceline_x_init;
    pre_referenceline_y_init = referenceline_y_init;

    // ref_x.clear();
    // ref_y.clear();
    for (int i = 0; i < n; i++) {
      ref_x.push_back(pre_result_x(i,0));
      ref_y.push_back(pre_result_y(i,0));
    }
  } else {
    cout<<"不是首次运行"<<endl;
    if (pre_referenceline_x_init(0) == referenceline_x_init(0) &&
        pre_referenceline_y_init(0) == referenceline_y_init(0)) {
      //起点相同，可以认为本周期和上周期的referenceline_init
      //一模一样，就直接复用上个周期的结果
      referenceline_x = pre_result_x;
      referenceline_y = pre_result_y;
    } else {
      //起点不同。重复第一次运行的逻辑
      for (int j = 0; j < 2 * n - 5; j++) {
        A1(j, j) = 1;
        A1(j, j + 2) = -2;
        A1(j, j + 4) = 1;
        A1(j + 1, j + 1) = 1;
        A1(j + 1, j + 3) = -2;
        A1(j + 1, j + 5) = 1;
      }
      for (int k = 0; k < 2 * n - 3; k++) {
        A2(k, k) = 1;
        A2(k, k + 2) = -1;
        A2(k + 1, k + 1) = 1;
        A2(k + 1, k + 3) = -1;
      }
      for (int i = 0; i < 2 * n; i++) {
        A3(i, i) = 1;
      }

      // cout << "A1: " << A1 << endl;
      // cout << "A2: " << A2 << endl;

      Eigen::MatrixXd H;
      //
      Eigen::SparseMatrix<double> H1_d;         // H
      Eigen::VectorXd f_d;                      // f
      Eigen::VectorXd lb_d;                     //下边界
      Eigen::VectorXd ub_d;                     //上边界
      Eigen::SparseMatrix<double> linearMatrix; // A
      H = 2 * (w_cost_smooth * (A1.transpose() * A1) +
               w_cost_length * (A2.transpose() * A2) + w_cost_ref * A3);

      //设置H矩阵
      H1_d.resize(H.rows(), H.cols());
      for (int i = 0; i < H.rows(); i++) {
        for (int j = 0; j < H.cols(); j++) {
          H1_d.insert(i, j) = H(i, j);
        }
      }
      //设置f矩阵
      f_d.resize(2 * n);
      for (int k = 0; k < n; k++) {
        f_d(2 * k) = -2 * w_cost_ref * xy_points_[k].first;
        f_d(2 * k + 1) = -2 * w_cost_ref * xy_points_[k].second;
      }
      //设置A矩阵
      linearMatrix.resize(2 * n, 2 * n);
      for (int i = 0; i < 2 * n; i++) {
        linearMatrix.insert(i, i) = 1;
      }
      //设置上下边界
      lb_d.resize(2 * n);
      ub_d.resize(2 * n);
      for (int i = 0; i < n; i++) {
        f(2 * i, 0) = xy_points_[i].first;
        f(2 * i + 1, 0) = xy_points_[i].second;
        lb_d(2 * i) = f(2 * i, 0) - x_lb;
        ub_d(2 * i) = f(2 * i, 0) + x_ub;
        lb_d(2 * i + 1) = f(2 * i + 1, 0) - y_lb;
        ub_d(2 * i + 1) = f(2 * i + 1, 0) + y_ub;
      }
      // std::cout << "hessian:" << std::endl << H1_d << std::endl;
      // std::cout << "f_d:" << std::endl << f_d << std::endl;
      // std::cout << "hessian:" << std::endl << f_d << std::endl;
      // std::cout << "hessian:" << std::endl << ub_d << std::endl;

      int NumBerOfVaribles = 2 * n;
      int NumberOfConstraints = 2 * n;
      //二次规划求解
      OsqpEigen::Solver solver;
      solver.settings()->setVerbosity(false);
      solver.settings()->setWarmStart(true);
      // 设置QP解算器的初始数据
      //矩阵A为m*n矩阵
      solver.data()->setNumberOfVariables(NumBerOfVaribles);
      solver.data()->setNumberOfConstraints(NumberOfConstraints);
      if (!solver.data()->setHessianMatrix(H1_d))
        return 1; //设置P矩阵
      if (!solver.data()->setGradient(f_d))
        return 1; //设置q or f矩阵。当没有时设置为全0向量
      if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
        return 1;
      if (!solver.data()->setLowerBound(lb_d))
        return 1; //设置下边界
      if (!solver.data()->setUpperBound(ub_d))
        return 1; //设置上边界
      if (!solver.initSolver())
        return 1;
      Eigen::VectorXd QPSolution;
      if (!solver.solve())
        return 1;
      QPSolution = solver.getSolution();
      // std::cout << "QPSolution:" << std::endl << QPSolution << std::endl;
      for (int i = 0; i < n; i++) {
        referenceline_x(i,0) = QPSolution(2 * i);
        referenceline_y(i,0) = QPSolution(2 * i + 1);
      }
      pre_result_x = referenceline_x;
      pre_result_y = referenceline_y;
      pre_referenceline_x_init = referenceline_x_init;
      pre_referenceline_y_init = referenceline_y_init;

      // ref_x.clear();
      // ref_y.clear();
      for (int i = 0; i < n; i++) {
        ref_x.push_back(pre_result_x(i,0));
        ref_y.push_back(pre_result_y(i,0));
      }
      // cout << pre_result_x << endl;
      // cout << pre_result_y << endl;
    }
  }
  return 0;
}
