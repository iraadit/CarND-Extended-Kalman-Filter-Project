#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

const float DoublePI = 2 * M_PI;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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

  long x_size = x_.size();
  I_ = MatrixXd::Identity(x_size, x_size);
}

void KalmanFilter::Predict() {
  /**
    * predict the state
  */

  x_ = F_ * x_; // + u;

  MatrixXd F_transpose = F_.transpose();
  P_ = F_ * P_ * F_transpose + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations (lidar)
  */

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  KF(y);
}

VectorXd KalmanFilter::ConvertCartesianToPolar() {
  /**
    * convert from cartesian to polar coordinates for the radar
  */

  // Conversion from polar to cartesian coordinates
  double px, py, vx, vy;
  px = x_(0);
  py = x_(1);
  vx = x_(2);
  vy = x_(3);

  double rho = sqrt(px * px + py * py);
  double phi = atan2(py, px); // returns values between -PI and PI
  // check division by zero
  if (rho < 0.0001) {
    rho = 0.0001;
  }
  double rho_dot = (px * vx + py * vy) / rho;

  VectorXd z_pred = VectorXd(3); // h(x_)
  z_pred << rho, phi, rho_dot;

  return z_pred;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
    * update the state by using Extended Kalman Filter equations (radar)
  */

  VectorXd z_pred = ConvertCartesianToPolar();
  VectorXd y = z - z_pred;

  // normalize the angle between -PI to PI
  while(y(1) > M_PI){
    y(1) -= DoublePI;
  }

  while(y(1) < -M_PI){
    y(1) += DoublePI;
  }

  KF(y);
}

void KalmanFilter::KF(const VectorXd &y) {
  /**
    * Common part for the Update of Kalman Filter and of the Extended Kalman Filter
  */

  MatrixXd H_transpose = H_.transpose();
  MatrixXd P_H_transpose = P_ * H_transpose;

  MatrixXd S = H_ * P_H_transpose + R_;
  MatrixXd S_inverse = S.inverse();

  MatrixXd K =  P_H_transpose * S_inverse;

  // new estimate
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_) * P_;
}
