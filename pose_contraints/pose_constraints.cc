#include "Eigen/Cholesky"
#include "sophus/se3.hpp"

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

int main() {
  Eigen::Matrix4d target;
  target << -0.436986, 0.457569, -0.774386, -7.12922, -0.541741, 0.553378,
      0.632684, 27.9613, 0.718025, 0.695991, 0.00606516, -42.5424, 0, 0, 0, 1;
  Eigen::Matrix4d current;
  current << -0.374503, 0.372684, -0.849031, -6.97732, -0.386477, 0.769596,
      0.508288, 23.2421, 0.842842, 0.518487, -0.144182, -45.3681, 0, 0, 0, 1;
  std::cout << "target: \n" << target << std::endl;
  std::cout << "current: \n" << current << std::endl;

  Sophus::SE3d target_Twc(target);
  Sophus::SE3d target_Tcw = target_Twc.inverse();
  Sophus::SE3d current_Twc(current);
  Sophus::SE3d current_Tcw = current_Twc.inverse();
  /*********optimize************/
  Sophus::SE3d diff = current_Tcw * target_Twc;
  Vector6d b = diff.log();
  std::cout << "diff: " << b.transpose() << std::endl;

  Matrix6d J;

  J = target_Tcw.Adj();

  Matrix6d H;
  H = J.transpose() * J;

  Vector6d step = H.ldlt().solve(-J.transpose() * b);

  Sophus::SE3d result_Tcw = current_Tcw * Sophus::SE3d::exp(step);
  Sophus::SE3d result_Twc = result_Tcw.inverse();

  std::cout << "result_Twc: \n" << result_Twc.matrix3x4() << std::endl;
}