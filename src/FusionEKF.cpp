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
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  Done:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  
  //measurement matrix for Laser
  H_laser_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;

  //state transition matrix (dt elements on 0,2 and 1,3 to be set later when we get timestamps
  MatrixXd F = MatrixXd(4,4);
  F << 1,0,0,0,
       0,1,0,0,
       0,0,1,0,
       0,0,0,1;
  ekf_.F_ = F;

  //process covariance matrix
  MatrixXd Q = MatrixXd(4,4);
  Q << 0,0,0,0,
       0,0,0,0,
       0,0,0,0,
       0,0,0,0; // will be modified with time stamps differences
  ekf_.Q_ = Q;


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
    Done:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    //empty state vector
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0,0,0,0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);
      // initialise vx,vy as if there was only radial velocity
      ekf_.x_ << rho*cos(phi), rho*sin(phi), rho_dot*cos(phi), rho_dot*sin(phi);

      // state covariance initialisation: small uncertainty on px,py, larger on vx,vy as we only measure radial velocity
      MatrixXd P = MatrixXd(4,4);
      P << 3,0,0,  0,
           0,3,0,  0,
           0,0,10, 0,
           0,0,0,  10;
      ekf_.P_ = P;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      // keep vx,vy at 0 as we don't have any info about them with laser measurment
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);

      // state covariance initialisation: small uncertainty on px,py, but extremely large on vx,vy as lidar doesn't measure speed at all
      MatrixXd P = MatrixXd(4,4);
      P << 1,0,0,    0,
           0,1,0,    0,
           0,0,1000, 0,
           0,0,0,    1000;
      ekf_.P_ = P;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   Done:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  // update F
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.;
  
  // only do the predict state if dt is larger than 1ms
  // always the case in the original udacity data, but could be different if lidar/radar
  // data were not synchronized, could get data very close to each other sometimes
  if (dt > 1e-3) {
    previous_timestamp_ = measurement_pack.timestamp_;
  
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;

    // update Q
    float dt2 = dt*dt;
    float dt3 = dt*dt2;
    float dt4 = dt*dt3;
    float noise_ax = 9, noise_ay = 9;
    ekf_.Q_ << dt4/4*noise_ax, 0,              dt3/2*noise_ax, 0,
               0,              dt4/4*noise_ay, 0,              dt3/2*noise_ay,
               dt3/2*noise_ax, 0,              dt2*noise_ax,   0,
               0,              dt3/2*noise_ay, 0,              dt2*noise_ay;
  
    // predict state
    ekf_.Predict();
  }

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   Done:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Jacobian matrix at current state
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    // update H and R
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    // update state
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    // update H and R
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    // update state
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
