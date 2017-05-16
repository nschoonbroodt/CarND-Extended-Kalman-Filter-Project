#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  Done:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::UpdateStep(const VectorXd &y) {
  MatrixXd P_Ht = P_ * H_.transpose();
  MatrixXd S = H_ * P_Ht + R_;
  MatrixXd K = P_Ht * S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

  x_ = x_ + K * y;
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  Done:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateStep(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  Done:
    * update the state by using Extended Kalman Filter equations
  */
  // Compute z_pred using non linear function
  // for convenience put state into meaningfull variables
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rho = sqrt(px*px + py*py);

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, atan2(py,px), (px*vx+py*vy)/rho;

  // makes sure that the angle is in [-pi, pi]
  VectorXd y = z - z_pred;
  if (y(1) > M_PI) {
    y(1) -= 2*M_PI;
  } else if (y(1) < -M_PI) {
    y(1) += 2*M_PI;
  }
  
  UpdateStep(y);
}
