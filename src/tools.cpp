#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  Done:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // Accumulate the error
  std::vector<VectorXd>::const_iterator i1,i2;
  for (i1 = estimations.begin(), i2 = ground_truth.begin();
       i1 < estimations.end() && i2 < ground_truth.end();
       ++i1, ++i2) {
    VectorXd residual = *i1-*i2;
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // square root of average
  rmse /= estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  Done:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);

  // put state in friendly named variables
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // intermediate computations
  float rho_2 = px*px + py*py;
  float rho = sqrt(rho_2);
  float rho_32 = rho_2*rho;

  // Check for div by 0
  if (fabs(rho_2) < 1e-4) {
    std::cout << "CalculateJacobian() - Error: Division by Zero" << std::endl;
    return Hj;
  }

  // compute the matrix
  Hj << px/rho,                  py/rho,                  0.0,    0.0,
        -py/rho_2,               px/rho_2,                0.0,    0.0,
        py*(vx*py-vy*px)/rho_32, px*(vy*px-vx*py)/rho_32, px/rho, py/rho;

  return Hj;
}

