#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.75;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/8;

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
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  ///* initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;
  
  ///* time when the state is true, in us
  time_us_ = 0.0;
  
  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_x_;
  
  ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  
  ///* predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  
  ///* current NIS evaluation for laser
  NIS_laser_ = 0.0;
  
  ///* Current NIS evaluation for radar
  NIS_radar_ = 0.0;  
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  ///* ensure at least one type of laser and radar date is/are used
  if((meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_)||
      (meas_package.sensor_type_ == MeasurementPackage:: LASER && use_laser_)){
 
    ///***********************A. Initializaiton in UKF ****************************
    ///**********************************************************************
    if (! is_initialized_){
    ///* here the state vector x_ and covariance matrix P_ are initialized. And x_ is initialized 
    ///* with the first measurement,
    x_ << 1,1,1,1,1;
    P_ << 0.15,0, 0, 0, 0,
               0, 0.15, 0, 0, 0,
               0, 0, 1, 0, 0,
               0, 0, 0, 1, 0,
               0, 0, 0, 0, 1;
    
     time_us_ = meas_package.timestamp_; // initialize the timestamp
     
     if(meas_package.sensor_type_ == MeasurementPackage:: LASER && use_laser_){
        
        x_(0) = meas_package.raw_measurements_(0);
        x_(1) = meas_package.raw_measurements_(1);     
     }
     else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
        
        float rho = meas_package.raw_measurements_(0);
        float phi = meas_package.raw_measurements_(1);
        float rhodot = meas_package.raw_measurements_(2);
        
        x_(0) = rho * cos(phi);
        x_(1) = rho * sin(phi);    
     }
    
    is_initialized_ = true;
    
    return;   
    }  
  
    ///* convert the time difference into seconds
    float dt = (meas_package.timestamp_ - time_us_)/1000000.0;
    time_us_ = meas_package.timestamp_;
    cout<<"the delta_t is: "<<dt<<endl; 
    ///*********************** B . UKF Prediction *******************************
    ///**********************************************************************
    Prediction(dt);
    
    ///*********************** C. UKF Update ***********************************
    ///**********************************************************************
    if (meas_package.sensor_type_ == MeasurementPackage:: LASER){
      UpdateLidar(meas_package);
    } 
    else if (meas_package.sensor_type_ == MeasurementPackage:: RADAR){
      UpdateRadar(meas_package);
    }
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  ///* ********************* 1. generate sigma points***************************
  ///**********************************************************************
  MatrixXd Xsig = MatrixXd(n_x_,2*n_x_+1);
  MatrixXd A = P_.llt().matrixL();
  
  lambda_ = 3 - n_x_;
  ///* set sigma points matrix
  Xsig.col(0) = x_;
  for(int i=0; i<n_x_; i++){ 
    Xsig.col(i+1) = x_ + sqrt(lambda_+ n_x_)*A.col(i);
    Xsig.col(i+1+n_x_) = x_ - sqrt(lambda_+n_x_)*A.col(i);
  }
  
  ///***********************************2.  Augment the sigma points***********
  ///**********************************************************************
  /// create the predicted mean state vector and covariance matrix
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  lambda_ = 3 - n_aug_;
  
  /// set  the predicted  state mean vector and covariance matrix 
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;
  
  MatrixXd L = P_aug.llt().matrixL();
  
  ///* set the augmented sigma points matrix
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i< n_aug_;i++){
    Xsig_aug.col(i+1) = x_aug + sqrt(lambda_+n_aug_)* L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  
  ///****************************3. predict sigma points**************************
  ///*************************************************************************
  for (int i = 0; i< 2*n_aug_+1;i++){
    double px = Xsig_aug(0,i);
    double py = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);
  
  /// *predited state values
    double px_pred;
    double py_pred;
    
    if(fabs(yawd) > 0.0001){
      px_pred = px +v/yawd* (sin(yaw + yawd*delta_t) - sin(yaw));
      py_pred = py + v/yawd*(-cos(yaw + yawd*delta_t) + cos(yaw));    
    }
    else{
      px_pred = px + v*cos(yaw)*delta_t;
      py_pred = py + v*sin(yaw)*delta_t;
    }
    
    double v_pred = v;
    double yaw_pred = yaw+ yawd*delta_t;
    double yawd_pred = yawd;
    
    /// *add noise to predicted state
    px_pred = px_pred + 0.5*delta_t*delta_t*cos(yaw)*nu_a;
    py_pred = py_pred + 0.5*delta_t*delta_t*sin(yaw)*nu_a;
    v_pred = v_pred + delta_t*nu_a;
    yaw_pred = yaw_pred + 0.5* delta_t* delta_t*nu_yawdd;
    yawd_pred = yawd_pred + delta_t*nu_yawdd;
    
    /// * insert these predicted values into augmented simga points matrix
    Xsig_pred_(0,i) = px_pred; 
    Xsig_pred_(1,i) = py_pred;
    Xsig_pred_(2,i) = v_pred;
    Xsig_pred_(3,i) = yaw_pred;
    Xsig_pred_(4,i) = yawd_pred;    
  }
  
  ///********************4. predict state mean and covariance matrix***************
  ///***********************************************************************
  double weight_0 = lambda_/(lambda_+ n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i< 2*n_aug_+1;i++){
    double weight = 0.5/(lambda_ + n_aug_);
    weights_(i) = weight;
  }
  
  x_.fill(0.0);
  for(int i = 0; i< 2*n_aug_ +1;i++){
    x_ = x_ + weights_(i)* Xsig_pred_.col(i);
  }
  
  P_.fill(0.0);
  for(int i = 0; i< 2*n_aug_+1; i++){
  
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while(x_diff(3) > M_PI)  x_diff(3) -= 2.* M_PI;
    while(x_diff(3) < - M_PI)  x_diff(3) += 2.*M_PI;
    
    P_ = P_ + weights_(i)*x_diff*x_diff.transpose();
  }
   cout<< "predicted state x_ = "<< x_<<endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  ///********************* 5. measurement prediction***************************
  ///***********************************************************************
  ///* gain the new measurement z
  VectorXd z =  meas_package.raw_measurements_;
  
  int n_z = 2; 
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);  // sigma points matrix in measurement space
  for (int i = 0; i < 2*n_aug_+1; i++){
    
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    
    Zsig(0,i) = px;
    Zsig(1,i) = py;
  }
  
  ///* create predicted measurement mean vector and covariance matrix
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i = 0; i< 2*n_aug_+1;i++){
    z_pred = z_pred + weights_(i)*Zsig.col(i);
  }
  
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for(int i = 0; i< 2* n_aug_ +1; i++){
  
    VectorXd z_diff = Zsig.col(i) - z_pred; // residual
    S = S + weights_(i)*z_diff*z_diff.transpose();
  }
  
  ///* add measurement  noise
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_laspx_*std_laspx_;
  R(1,1) = std_laspy_*std_laspy_;
  
  S = S+R;
  
  ///***************************6. update UKF state for laser*********************
  ///***********************************************************************
  MatrixXd Tc = MatrixXd(n_x_,n_z);   // declare the cross-correlation matrix
  Tc.fill(0.0);
  for(int i = 0; i<2*n_aug_ + 1; i++){
    
    VectorXd z_diff = Zsig.col(i) - z_pred;
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    Tc = Tc + weights_(i)* x_diff*z_diff.transpose();
 }   
  ///* calculate kalman gain K
  MatrixXd K = Tc * S.inverse();
    
  ///* new residual
  VectorXd z_diff = z - z_pred;
    
  /// * update state mean vector and covariance matrix
  x_ = x_ + K* z_diff;
  P_ = P_ - K*S*K.transpose();
   
  ///*************************** 7. calculate the NIS for laser********************
  ///**********************************************************************
  NIS_laser_ = z_diff.transpose()*S.inverse()*z_diff;
  cout<<"NIS for laser is: "<<NIS_laser_<<endl;
  cout<<"after laser update state x_ = "<<x_<<endl;
}
/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  ///***********************5. measurement prediction**************************
  ///***********************************************************************
   VectorXd z = meas_package.raw_measurements_;
   
   int n_z = 3;
   MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
   
   for(int i = 0; i< 2*n_aug_+1; i++){
     double px =  Xsig_pred_(0,i); 
     double py =  Xsig_pred_(1,i);
     double v = Xsig_pred_(2,i);
     double yaw = Xsig_pred_(3,i);
     
     double v1 = cos(yaw) * v;
     double v2 = sin(yaw) * v;
     
     double rho = sqrt(px*px + py*py);
     double phi = 0.0;
     double rhodot = 0.0;
     
     if(fabs(px) < 0.0001){
         px = 0.0001;
     }
     phi = atan2(py,px);
     
     if(fabs(rho)<0.0001){
         rho = 0.0001;
     }
     rhodot = (px*v1 + py*v2)/rho;
     
     Zsig(0,i) = rho;
     Zsig(1,i) = phi;
     Zsig(2,i) = rhodot;   
}

/// predict measurement mean vector and covariance matrix
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i = 0; i< 2*n_aug_+1; i++){
    z_pred = z_pred + weights_(i)*Zsig.col(i);      
  }

  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for(int i = 0; i< 2*n_aug_+1; i++){
  
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while (z_diff(1) > M_PI) z_diff(1)-= 2.*M_PI;
    while (z_diff(1) < - M_PI) z_diff(1)+= 2.* M_PI;
    
    S = S + weights_(i)* z_diff* z_diff.transpose();
  }
  
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_radr_* std_radr_;
  R(1,1) = std_radphi_* std_radphi_;
  R(2,2) = std_radrd_* std_radrd_;
  
  S = S + R;
  
  ///******************* 6. UKF update state for radar ***************************
  ///***********************************************************************
  ///* declare the cross-correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for(int i = 0; i< 2* n_aug_+1; i++){
    
    VectorXd x_diff =  Xsig_pred_.col(i) - x_;
    while(x_diff(3)> M_PI) x_diff(3)-= 2.*M_PI;
    while(x_diff(3)< -M_PI) x_diff(3)+= 2.*M_PI;
    
    VectorXd z_diff = Zsig.col(i) - z_pred;
    while(z_diff(1) > M_PI)  z_diff(1)-= 2.*M_PI;
    while(z_diff(1) < -M_PI) z_diff(1)+= 2.*M_PI;
    
    Tc = Tc + weights_(i)*x_diff*z_diff.transpose();
  }
  
  /// * kalman gain K
  MatrixXd K = Tc * S.inverse();
  
  /// * update  state mean vector and covariance matrix
  ///* residual when t  = k+ 1
  VectorXd z_diff = z - z_pred;
  while(z_diff(1) > M_PI)  z_diff(1)-= 2.*M_PI;
  while(z_diff(1) < -M_PI) z_diff(1)+= 2.*M_PI;
  
  x_ = x_ + K*z_diff;
  P_ = P_ - K*S*K.transpose();
  
  ///*****************7. calculate the NIS for radar******************************
  ///***********************************************************************
  NIS_radar_ = z_diff.transpose()* S.inverse()*z_diff;  
  cout<<"NIS for radar is: "<<NIS_radar_<<endl;
  cout<<"after radar update state x_= "<<x_<<endl;
  } 
