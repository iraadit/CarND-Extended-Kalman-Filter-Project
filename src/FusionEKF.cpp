#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0,      0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0,      0,
              0,    0.0009, 0,
              0,    0,      0.09;

  /**
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  VectorXd x = VectorXd(4);

  MatrixXd F =  MatrixXd::Identity(4, 4);

  MatrixXd P =  MatrixXd(4, 4);
  P <<  1,    0,    0,    0,
        0,    1,    0,    0,
        0,    0,    1000, 0,
        0,    0,    0,    1000;

  MatrixXd Q = MatrixXd(4, 4);

  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement

    float px;
    float py;
    float vx;
    float vy;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];

      px = rho * cos(phi);
      py = rho * sin(phi);

      /**
       * From lesson, part Tips and Tricks:
       * Although radar gives velocity data in the form of the range rate rho_dot, a radar measurement does
       * not contain enough information to determine the state variable velocities vx and vy
       */
      vx = rho_dot * cos(phi); // 0;
      vy = rho_dot * sin(phi); // 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */

      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
      vx = 0;
      vy = 0;
    }

    ekf_.x_ << px, py, vx, vy;

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / (double)CLOCKS_PER_SEC;

  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  float dt2 = dt   * dt; // dt^2
  float dt3 = dt2  * dt; // dt^3
  float dt4 = dt3  * dt; // d^4

  float dt4_4 = dt4 / 4; // dt^4/4
  float dt3_2 = dt3 / 2; // dt^3/2

  ekf_.Q_ <<  dt4_4 * noise_ax,   0,                dt3_2 * noise_ax, 0,
              0,                  dt4_4 * noise_ay, 0,                dt3_2 * noise_ay,
              dt3_2 * noise_ax,   0,                dt2 * noise_ax,   0,
              0,                  dt3_2 * noise_ay, 0,                dt2 * noise_ay;

  ekf_.Predict();


  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    if (ekf_.x_[0] == 0 && ekf_.x_[1] == 0){
      return;
    }

    ekf_.R_ = R_radar_;

    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;

    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;

    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
