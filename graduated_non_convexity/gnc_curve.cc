#include <stdlib.h>
#include <algorithm>
#include <iostream>
#include <vector>

#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include "toc.h"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

class CurveVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual void setToOriginImpl() { _estimate << 0, 0, 0; }

  virtual void oplusImpl(const double* update) {
    _estimate += Eigen::Vector3d(update);
  }

  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& out) const {}
};

class CurveEdge : public g2o::BaseUnaryEdge<1, double, CurveVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CurveEdge(double x) : BaseUnaryEdge(), x_(x) {}

  void computeError() {
    auto* v = static_cast<CurveVertex*>(_vertices[0]);
    const Eigen::Vector3d abc = v->estimate();
    _error(0) = _measurement - function(abc, x_);
  }

  double GetError() {
    computeError();
    return _error(0);
  }
  static double function(const Eigen::Vector3d& abc, const double x) {
    const double x2 = x * x;
    return std::exp(abc(0) * x2 + abc(1) * x + abc(2));
  }

  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& out) const {}

 private:
  double x_;
};

Eigen::Vector3d GNC(const std::vector<double>& x, const std::vector<double>& y,
                    const Eigen::Vector3d& abc, const size_t inlier_size) {
  // build optimizer
  g2o::SparseOptimizer optimizer;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1> > Block;
  Block::LinearSolverType* linearSolver =
      new g2o::LinearSolverDense<Block::PoseMatrixType>();  // 线性方程求解器
  Block* solver_ptr = new Block(linearSolver);  // 矩阵块求解器
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  // The only vertex, the parameter of function
  auto* v = new CurveVertex();
  // Initial values are one
  v->setEstimate(Eigen::Vector3d::Ones());
  v->setId(0);
  optimizer.addVertex(v);
  // Initial weight are one
  std::vector<double> weight(x.size(), 1);

  std::vector<CurveEdge*> edges;
  double max_error = 0;
  for (size_t i = 0; i < x.size(); ++i) {
    CurveEdge* edge = new CurveEdge(x[i]);
    edge->setId(i);
    edge->setVertex(0, v);
    edge->setMeasurement(y[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * weight[i]);
    optimizer.addEdge(edge);
    edges.emplace_back(edge);
    double tmp_error = fabs(edge->GetError());
    if (tmp_error > max_error) {
      max_error = tmp_error;
    }
  }
  // std::cout << "Max_error: " << max_error << std::endl;
  // c^2
  const double c2 = 1;
  double mu = 2 * max_error * max_error / c2;
  // std::cout << "Inital Mu: " << mu << std::endl;
  int iter_index = 0;
  auto abc_estimate = v->estimate();
  optimizer.initializeOptimization();
  mu *= 1.4;
  do {
    mu /= 1.4;
    // std::cout << "Iter : " << std::setprecision(3) << iter_index;
    const double tmp_mu = ((mu > 1) ? mu : 1);
    // std::cout << ", Mu: " << tmp_mu;
    const double uc2 = tmp_mu * c2;
    // std::cout << ", uc2: " << uc2;
    double inlier_weight = 0;
    double inlier_error = 0;
    double outlier_weight = 0;
    double outlier_error = 0;
    for (int i = 0; i < edges.size(); ++i) {
      auto* edge = edges[i];
      const double error = edge->GetError();
      const double e2 = error * error;
      double w = (uc2) / (e2 + uc2);
      w = w * w;
      edge->setInformation(w * Eigen::Matrix<double, 1, 1>::Identity());
      if (i < inlier_size) {
        inlier_weight += w;
        inlier_error += e2;
      } else {
        outlier_weight += w;
        outlier_error += e2;
      }
    }
    // std::cout << ", Inlier/Outlier weight: " << inlier_weight << " / "
    //           << outlier_weight;
    // std::cout << ", Inlier/Outlier error: " << inlier_error << " / "
    //           << outlier_error;
    optimizer.optimize(5);
    abc_estimate = v->estimate();
    Eigen::Vector3d delta = abc_estimate - abc;
    // std::cout << ", sum: " << delta.cwiseAbs().sum() << std::endl;
    iter_index++;
  } while (mu > 1);

  return abc_estimate;
}

Eigen::Vector3d Robust(const std::vector<double>& x,
                       const std::vector<double>& y) {
  // build optimizer
  g2o::SparseOptimizer optimizer;
  typedef g2o::BlockSolver<g2o::BlockSolverTraits<3, 1> > Block;
  Block::LinearSolverType* linearSolver =
      new g2o::LinearSolverDense<Block::PoseMatrixType>();  // 线性方程求解器
  Block* solver_ptr = new Block(linearSolver);  // 矩阵块求解器
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);
  optimizer.setVerbose(false);

  // The only vertex, the parameter of function
  auto* v = new CurveVertex();
  // Initial values are one
  v->setEstimate(Eigen::Vector3d::Ones());
  v->setId(0);
  optimizer.addVertex(v);

  std::vector<CurveEdge*> edges;
  for (size_t i = 0; i < x.size(); ++i) {
    CurveEdge* edge = new CurveEdge(x[i]);
    edge->setId(i);
    edge->setVertex(0, v);
    edge->setMeasurement(y[i]);
    edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    // robust kernel
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    edge->setRobustKernel(rk);
    rk->setDelta(0.5);
    optimizer.addEdge(edge);
    edges.emplace_back(edge);
  }
  optimizer.initializeOptimization();
  optimizer.optimize(20);
  return v->estimate();
}

int main(int argc, char** argv) {
  // estimate value
  Eigen::Vector3d abc(1.0, 2.0, 5.0);
  // data's edge and step
  double edge = 1;
  double step = 0.05;
  // noise generator
  std::default_random_engine generator;
  std::normal_distribution<double> dist(0.0, 0.5);

  // measurements
  std::vector<double> x, y;
  double y_max = 0;
  // create inlier
  for (double i = -edge; i < edge; i += step) {
    x.emplace_back(i);
    double noise = dist(generator);
    double tmp_y = CurveEdge::function(abc, i);
    y.emplace_back(tmp_y + noise);
    if (tmp_y > y_max) {
      y_max = tmp_y;
    }
  }
  // plot inlier
  plt::plot(x, y, "g+");
  const int inlier_size = x.size();
  std::cout << "y_max: " << y_max << std::endl;
  std::cout << "Inlier size: " << x.size() << std::endl;

  std::normal_distribution<double> dist2(0.0, 0.5 * y_max);
  // create outlier
  {
    std::vector<double> x_out, y_out;
    for (double i = -edge; i < edge; i += 0.5 * step) {
      x_out.emplace_back(i);
      double noise = dist2(generator);
      y_out.emplace_back(noise);
    }
    // plot outlier
    plt::plot(x_out, y_out, "b*");
    x.insert(x.end(), x_out.begin(), x_out.end());
    y.insert(y.end(), y_out.begin(), y_out.end());
    std::cout << "Outlier size: " << x_out.size() << std::endl;
  }
  std::cout << "Total size: " << x.size() << std::endl;

  TicToc tic;
  Eigen::Vector3d abc_estimate = GNC(x, y, abc, inlier_size);
  std::cout << "GNC time cost: " << tic.toc() << std::endl;
  tic.tic();
  Eigen::Vector3d abc_estimate2 = Robust(x, y);
  std::cout << "Robust time cost: " << tic.toc() << std::endl;
  std::vector<double> x_estimate, y_estimate;
  for (double i = -edge; i < edge; i += step) {
    x_estimate.emplace_back(i);
    y_estimate.emplace_back(CurveEdge::function(abc_estimate, i));
  }

  plt::plot(x_estimate, y_estimate, "r-");
  x_estimate.clear();
  y_estimate.clear();
  for (double i = -edge; i < edge; i += step) {
    x_estimate.emplace_back(i);
    y_estimate.emplace_back(CurveEdge::function(abc_estimate2, i));
  }

  plt::plot(x_estimate, y_estimate, "g-");
  // Without pause, the figure may fail.
  plt::pause(2);
  plt::show();
}