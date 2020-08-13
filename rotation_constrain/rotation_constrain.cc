#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include "g2o/types/slam3d/se3_ops.h"

namespace g2o {

class VertexRotation : public BaseVertex<3, Eigen::Matrix3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VertexRotation() = default;

  bool read(std::istream& /*is*/){};

  bool write(std::ostream& /*os*/) const {};

  virtual void setToOriginImpl() { _estimate = Eigen::Matrix3d::Identity(); }

  virtual void oplusImpl(const double* update_) {
    Eigen::Map<const Eigen::Vector3d> update(update_);
    setEstimate(exp(update) * estimate());
  }

  static Eigen::Matrix3d exp(const Eigen::Vector3d& omega) {
    double theta = omega.norm();
    Eigen::MatrixX3d Omega = skew(omega);

    Eigen::MatrixX3d R;
    if (theta < 0.00001) {
      R = (Eigen::Matrix3d::Identity() + Omega + 0.5 * Omega * Omega);
    } else {
      Eigen::Matrix3d Omega2 = Omega * Omega;

      R = (Matrix3D::Identity() + sin(theta) / theta * Omega +
           (1 - cos(theta)) / (theta * theta) * Omega2);
    }
    return R;
  }
};

class EdgeManhattanConstraint
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexRotation> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EdgeManhattanConstraint() = default;

  virtual bool read(std::istream& /*is*/) { return true; };

  virtual bool write(std::ostream& /*os*/) const { return true; };

  inline void computeError() {
    const g2o::VertexRotation* v1 =
        static_cast<const g2o::VertexRotation*>(_vertices[0]);
    Eigen::Matrix3d R = v1->estimate();
    Eigen::Matrix3d delta = R * target_R.inverse();
    _error = Log(delta);
  }

  Eigen::Vector3d Log(const Eigen::Matrix3d& R) {
    // double d = 0.5 * (R(0, 0) + R(1, 1) + R(2, 2) - 1);
    const double d = 0.5 * (R.trace() - 1);
    Eigen::Vector3d omega;
    Eigen::Vector3d dR = g2o::deltaR(R);
    if (d > 0.99999) {
      omega = 0.5 * dR;
    } else {
      double theta = acos(d);
      omega = theta / (2 * sqrt(1 - d * d)) * dR;
    }
    return omega;
  }

  virtual void linearizeOplus() { _jacobianOplusXi = target_R; };

  Eigen::Matrix3d target_R;
};
}

int main() {
  Eigen::Matrix3d R_init = Eigen::Matrix3d::Identity();
  Eigen::AngleAxisd rotation_vector(0.1, Eigen::Vector3d(0.5, 0.5, 1));
  Eigen::Matrix3d R_target = rotation_vector.toRotationMatrix();

  std::cout << "R_target: \n" << R_target << std::endl;

  g2o::SparseOptimizer optimizer;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1> > Block;
  Block::LinearSolverType* linearSolver =
      new g2o::LinearSolverDense<Block::PoseMatrixType>();  // 线性方程求解器
  Block* solver_ptr = new Block(linearSolver);  // 矩阵块求解器
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  auto* v = new g2o::VertexRotation();
  v->setToOriginImpl();
  std::cout << "Original Rotation: \n" << v->estimate() << std::endl;
  v->setId(0);
  optimizer.addVertex(v);

  g2o::EdgeManhattanConstraint* edge = new g2o::EdgeManhattanConstraint();
  edge->target_R = R_target;
  edge->setId(0);
  edge->setVertex(0, v);
  edge->setInformation(Eigen::Matrix3d::Identity());
  optimizer.addEdge(edge);

  optimizer.initializeOptimization();
  // optimizer.setVerbose(true);
  optimizer.optimize(100);
  std::cout << "Optimization Rotation: \n" << v->estimate() << std::endl;
}