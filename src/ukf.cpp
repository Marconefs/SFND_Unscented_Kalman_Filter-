#include "ukf.h"
#include "Eigen/Dense"
#include "iostream"
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // State vector: [px, py, long. velocity, yaw, yaw rate]
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (!is_initialized_) 
  {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
      P_.fill(0.0);
	    P_(0, 0) = std_laspx_*std_laspx_;
	    P_(1, 1) = std_laspy_*std_laspy_;
	    P_(2, 2) = 0;
      P_(3, 3) = 0;
      P_(4, 4) = 0;
    }
    else // RADAR
    {
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];

      x_ << rho*cos(phi), rho*sin(phi), rho_dot, phi, 0;

      P_.fill(0.0);
	    P_(0, 0) = std_radr_*std_radr_;
	    P_(1, 1) = std_radr_*std_radr_;
	    P_(2, 2) = std_radrd_*std_radrd_;
      P_(3, 3) = 0;
      P_(4, 4) = 0;
    }
    
    is_initialized_ = true;
    time_us_ = meas_package.timestamp_;
  }

  double delta_t = (meas_package.timestamp_-time_us_)/1e6;
  time_us_ = meas_package.timestamp_;
  
  Prediction(delta_t);
  
  if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);
  }
  
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  // Augmentation
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);

  // Augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // Augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // Calculate square root of P_aug
  MatrixXd A = P_aug.llt().matrixL();
  
  // Sigma point matrix
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
  }

  // # predict sigma points

  for (int i=0; i<2*n_aug_+1; ++i)
  {
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double  v  = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yaw_dot = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yaw_dotdot = Xsig_aug(6,i);
    
    // predicted state values
    double px_pred, py_pred;

    // avoid division by zero
    if (fabs(yaw_dot) > 0.001)
    {
      px_pred = p_x + v/yaw_dot * (sin(yaw+yaw_dot*delta_t) - sin(yaw));
      py_pred = p_y - v/yaw_dot * (cos(yaw+yaw_dot*delta_t) - cos(yaw));
    }
    else 
    {
      px_pred = p_x + v*cos(yaw)*delta_t;
      py_pred = p_y + v*sin(yaw)*delta_t;
    }

    double v_pred = v;
    double yaw_pred = yaw + yaw_dot*delta_t;
    double yaw_dot_pred = yaw_dot;

    //add noise
    px_pred += 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_pred += 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_pred += nu_a*delta_t;
    yaw_pred += 0.5*nu_yaw_dotdot*delta_t*delta_t;
    yaw_dot_pred += nu_yaw_dotdot*delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_pred;
    Xsig_pred_(1,i) = py_pred;
    Xsig_pred_(2,i) = v_pred;
    Xsig_pred_(3,i) = yaw_pred;
    Xsig_pred_(4,i) = yaw_dot_pred;    
  } 
  
  // # Predict Mean and Covariance
  
  // set weights, used to invert the spreading (lambda_) of the sigma points
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  
  // predicted state mean
  x_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; ++i)
  {
    x_ += weights_(i)*Xsig_pred_.col(i);
  }
  
  // Predicted state covariance matrix
  P_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; ++i)
  {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i)*x_diff*x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // Measurement data
  VectorXd z_ = meas_package.raw_measurements_;

  int n_z_ = 2;

  MatrixXd Zsig = MatrixXd(n_z_, 2*n_aug_+1); // sigma points in measurement space
  MatrixXd z_pred = VectorXd(n_z_); // mean predicted measurement
  z_pred.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; i++) // 2n+1 simga points
  {
    Zsig(0, i) = Xsig_pred_(0, i); // p_x
    Zsig(1, i) = Xsig_pred_(1, i); // p_y

    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  MatrixXd Tc = MatrixXd(n_x_,n_z_); // Cross-correlation Matrix Tc
  MatrixXd S = MatrixXd(n_z_,n_z_); // innovation covariance matrix S
  Tc.fill(0.0);
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i) // 2n+1 simga points
  { 
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
  
  // Measurement noise
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R.fill(0.0);
	R(0, 0) = std_laspx_ * std_laspx_;
  R(1, 1) = std_laspy_ * std_laspy_;
  S = S + R;
  
  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  // residual
  VectorXd z_diff = z_ - z_pred;  
  // Update State
  Eigen::VectorXd x_predicted = x_;
  x_ = x_ + K*z_diff;

  // Covariance Matrix Update
  P_ = P_ - K*S*K.transpose();

  // compute normalized innovation squared(NIS)
  double nis_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
  //std::cout << to_string(nis_lidar_)+","+to_string(x_predicted[2])+","+to_string(x_predicted[4])+","+to_string(x_[2])+","+to_string(x_[4]) << std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  // Measurement data
  VectorXd z_ = meas_package.raw_measurements_;

  int n_z_ = 3;

  MatrixXd Zsig = MatrixXd(n_z_, 2*n_aug_+1); // sigma points in measurement space
  MatrixXd z_pred = VectorXd(n_z_); // mean predicted measurement
  z_pred.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; i++) // 2n+1 simga points
  {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v   = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y); //r
    Zsig(1, i) = atan2(p_y, p_x); // phi
    Zsig(2, i) = (p_x * cos(yaw)*v + p_y * sin(yaw)*v) / sqrt(p_x * p_x + p_y * p_y); //r_dot

    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }
  
  MatrixXd Tc = MatrixXd(n_x_,n_z_); // Cross-correlation Matrix Tc
  MatrixXd S = MatrixXd(n_z_,n_z_); // innovation covariance matrix S
  Tc.fill(0.0);
  S.fill(0.0);
  for (int i = 0; i < 2*n_aug_+1; ++i) // 2n+1 simga points
  { 
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Radar measurement noise
  MatrixXd R = MatrixXd(n_z_, n_z_);
  R.fill(0.0);
	R(0, 0) = std_radr_ * std_radr_;
	R(1, 1) = std_radphi_ * std_radphi_;
	R(2, 2) = std_radrd_ * std_radrd_;

  S = S + R;
  
  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = z_ - z_pred;  

  // Update State
  Eigen::VectorXd x_predicted = x_;
  x_ = x_ + K*z_diff;

  // Covariance Matrix Update
  P_ = P_ - K*S*K.transpose();

  // compute normalized innovation squared(NIS)
  double nis_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  //std::cout << to_string(nis_radar_)+","+to_string(x_predicted[2])+","+to_string(x_predicted[4])+","+to_string(x_[2])+","+to_string(x_[4]) << std::endl;
}