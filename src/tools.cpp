#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd RMSE(4);
  RMSE<<0,0,0,0;
  
  if(estimations.size()!= ground_truth.size() || estimations.size()==0){
      cout<<"Invalid estimation or ground truth data!"<<endl;
      return RMSE;
  }
  
  for(int i = 0; i< estimations.size(); i++){
      VectorXd residual = estimations[i] - ground_truth[i];
      residual = residual.array()*residual.array();
      RMSE += residual;
  }
  
  RMSE = RMSE/estimations.size();
  RMSE = RMSE.array().sqrt();
  
  return RMSE;
 
}
