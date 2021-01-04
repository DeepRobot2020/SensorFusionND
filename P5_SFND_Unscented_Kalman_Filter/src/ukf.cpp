#include <iostream>
#include <math.h>
#include <limits.h>
#include "Eigen/Dense"
#include "ukf.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//normalize an input angle and return it
static double normalize_angle(double angle)
{
  //angle normalization
  while (angle > M_PI)
    angle -= 2. * M_PI;
  while (angle < -M_PI)
    angle += 2. * M_PI;
  return angle;
}


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // We need initialize the UKF on receiving the first measurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_.fill(0);

  // State dimension
  n_x_ = 5;
  
  // Augmented state dimension
  n_aug_ = n_x_ + 2;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_.fill(0.0);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.9;
  
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

  // Laser measurement noise covariance
  R_lidar_ = MatrixXd(n_l_, n_l_);
  R_lidar_ << std_laspx_ * std_laspx_, 0,
      0, std_laspy_ * std_laspy_;

  R_radar_ = MatrixXd(n_z_, n_z_);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_ * std_radphi_, 0,
      0, 0, std_radrd_ * std_radrd_;


  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  // Initialize the weights for sigmna points
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++)
  {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }


  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug_.fill(0.0);


  P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);
}

UKF::~UKF() {}

double UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  double senor_nis = 0.0;
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_)
  {    
      // first measurement
      x_ << 0, 0, 0, 0, 0;
      
      // init covariance matrix
      P_ << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

    // first measurement
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
    {
      // Convert radar from polar to cartesian coordinates and initialize state.
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);   
      double vx = meas_package.raw_measurements_[2] * cos(phi);
      double vy = meas_package.raw_measurements_[2] * sin(phi);  
      x_(2) = sqrt(vx * vx + vy * vy);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
    {
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return senor_nis;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  if (use_laser_ || use_radar_)
  {
    float dt = (meas_package.timestamp_ - time_us_) / 1e6; //dt - expressed in seconds
    time_us_ = meas_package.timestamp_;
    Prediction(dt);
  } else {
    std::cout << "UKF: both radar and laser are not used" << std::endl;
    return senor_nis;
  }

  /*****************************************************************************
   *  Measurement Update
   ****************************************************************************/
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_)
  {
    // Radar update
    senor_nis = UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)
  {
    // Radar update
    senor_nis = UpdateRadar(meas_package);
  }

  /*****************************************************************************
   *  Print current state vector and covariance
  //  ****************************************************************************/
  // cout << "x_ = " << x_ << endl;
  // cout << "P_ = " << P_ << endl;
  // cout << "senor_nis = " << senor_nis << endl;
  return senor_nis;
}

/**
 * Calculate augmented sigma points state and convariance matrix
 */
void UKF::CalculateAugmentSigmaPoints()
{
  //reset to 0 
  Xsig_aug_.fill(0.0);
  P_aug_.fill(0.0);

  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  // augmented state covariance
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_(5, 5) = std_a_ * std_a_;
  P_aug_(6, 6) = std_yawdd_ * std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug_.llt().matrixL();

  //create augmented sigma points
  float sigma_scale = sqrt(lambda_ + n_aug_);
  Xsig_aug_.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug_.col(i + 1) = x_aug + sigma_scale * L.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) = x_aug - sigma_scale * L.col(i);
  }

}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  /*******************************************************************************
   * Re-generate sigma points: Xsig_aug
   ******************************************************************************/
  CalculateAugmentSigmaPoints();

  /*******************************************************************************
   * Predict sigma points: Xsig_pred_
   ******************************************************************************/
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //extract values for better readability
    double p_x      = Xsig_aug_(0, i);
    double p_y      = Xsig_aug_(1, i);
    double v        = Xsig_aug_(2, i);
    double yaw      = Xsig_aug_(3, i);
    double yawd     = Xsig_aug_(4, i);
    double nu_a     = Xsig_aug_(5, i);
    double nu_yawdd = Xsig_aug_(6, i);

    //predicted state values
    double px_p = 0.0, py_p = 0.0;

    //avoid division by zero
    if (fabs(yawd) > 0.001)
    {
      px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
    }
    else
    {
      px_p = p_x + v * delta_t * cos(yaw);
      py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p    = v;
    double yaw_p  = yaw + yawd * delta_t;
    double yawd_p = yawd;

    //add noise
    double delta_t_square = delta_t * delta_t;
    px_p = px_p + 0.5 * nu_a * delta_t_square * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t_square * sin(yaw);
    v_p  = v_p + nu_a * delta_t;

    yaw_p  = yaw_p + 0.5 * nu_yawdd * delta_t_square;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = yaw_p;
    Xsig_pred_(4, i) = yawd_p;
  } //for (int i = 0; i < 2 * n_aug_ + 1; i++)

  /*******************************************************************************
   * Predict state mean and covariance: x_ and P_
   ******************************************************************************/
  //predicted state mean
  x_.fill(0.0); // Why this is necessary???
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0); // Why this is necessary???
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //iterate over sigma points
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    x_diff(3) = normalize_angle(x_diff(3));
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  } // for (int i = 0; i < 2 * n_aug_ + 1; i++)
} // void UKF::Prediction(double delta_t)


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 * @return {double} ladar_nis. The NIS value of the lidar sensor.
 */
double UKF::UpdateLidar( MeasurementPackage meas_package)
{
  /** Use lidar data to update the belief about the object's
    * position. Update state vector, x_, and covariance, P_ and  lidar NIS.
  */
  MatrixXd Zsig = MatrixXd(n_l_, 2 * n_aug_ + 1);
  Zsig.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);

    // measurement model
    Zsig(0, i) = p_x; //px
    Zsig(1, i) = p_y; //py
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_l_);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_l_, n_l_);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_lidar_;
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_l_);
  VectorXd z = meas_package.raw_measurements_;

  if(fabs(z(0)) < 0.001)
    z(0) = 0.001;
  if(fabs(z(1)) < 0.001)
    z(1) = 0.001;
    
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  //calculate nis
  lidar_nis_ = z_diff.transpose() * S.inverse() * z_diff;

  return lidar_nis_;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 * @return {double} radar_nis. The NIS value of the radar sensor.
 * 
 */
double UKF::UpdateRadar( MeasurementPackage meas_package)
{
  /*******************************************************************************
   * Map sigma points from from state space to radar measurement space
   ******************************************************************************/

  //transform sigma points into measurement space
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);
  MatrixXd Zsig_diff = MatrixXd(n_z_, 2 * n_aug_ + 1);
  
  Zsig.fill(0.0);
  Zsig_diff.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //2n+1 simga points
    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v   = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double v1  = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    if (fabs(p_x) < 0.001)
      p_x = 0.001;
    if (fabs(p_y) < 0.001)
      p_y = 0.001;

    // measurement model
    double rho = sqrt(p_x * p_x + p_y * p_y); //r
    double phi = atan2(p_y, p_x);
    double rho_dot = (p_x * v1 + p_y * v2) / rho; //r_dot

    Zsig(0, i) = rho;     //r
    Zsig(1, i) = phi;     //phi
    Zsig(2, i) = rho_dot; //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z_, n_z_);
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z_);
  VectorXd z = meas_package.raw_measurements_;

  Tc.fill(0.0);
  S.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  { //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    z_diff(1) = normalize_angle(z_diff(1));
    x_diff(3) = normalize_angle(x_diff(3));

    S = S + weights_(i) * z_diff * z_diff.transpose();
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_radar_;

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = normalize_angle(z_diff(1));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();


  
  //calculate nis
  radar_nis_ = z_diff.transpose() * S.inverse() * z_diff;

  return radar_nis_;
}
