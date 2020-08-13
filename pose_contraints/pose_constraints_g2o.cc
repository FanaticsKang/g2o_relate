#include <iostream>
#include "Eigen/Core"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "sophus/se3.hpp"

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>> BlockSolverType;
typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>
    LinearSolverType;

int main() {
  Eigen::Matrix4d target;
  target << -0.436986, 0.457569, -0.774386, -7.12922, -0.541741, 0.553378,
      0.632684, 27.9613, 0.718025, 0.695991, 0.00606516, -42.5424, 0, 0, 0, 1;
  Eigen::Matrix4d current;
  current << -0.374503, 0.372684, -0.849031, -6.97732, -0.386477, 0.769596,
      0.508288, 23.2421, 0.842842, 0.518487, -0.144182, -45.3681, 0, 0, 0, 1;
  std::cout << "target: \n" << target << std::endl;
  std::cout << "current: \n" << current << std::endl;

  g2o::VertexSE3* v_target = new g2o::VertexSE3();
  v_target->setEstimate(Eigen::Isometry3d(target));
  v_target->setId(0);
  v_target->setFixed(true);

  g2o::VertexSE3* v_current = new g2o::VertexSE3();
  v_current->setEstimate(Eigen::Isometry3d(current));
  v_current->setId(1);

  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;  // 图模型
  optimizer.setAlgorithm(solver);  // 设置求解器
  optimizer.setVerbose(true);      // 打开调试输出

  optimizer.addVertex(v_target);
  optimizer.addVertex(v_current);

  g2o::EdgeSE3* edge = new g2o::EdgeSE3();
  edge->setVertex(0, optimizer.vertices()[0]);
  edge->setVertex(1, optimizer.vertices()[1]);
  edge->setMeasurement(Eigen::Isometry3d::Identity());
  edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
  optimizer.addEdge(edge);

  optimizer.save("test.g2o");
  optimizer.initializeOptimization();
  optimizer.optimize(30);

  Eigen::Isometry3d result = v_current->estimate();
  std::cout << "result:  \n" << result.matrix() << std::endl;
}