#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    //Checking the validity of the following inputs
    //Estimation vector size should not be zero
    // Estimation vector size should equal ground_truth
    
    if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
        cout << "Error occured from validity, James you wrote this" << endl;

    }
    
    // find the some of the squared residuals
    for(unsigned int i = 0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];
        
        residual = residual.array() * residual.array();
        rmse += residual;
        
    }
    // Finding the mean
    rmse = rmse / estimations.size();
    // Square Root
    rmse = sqrt(rmse.array());
    
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3, 4);
    
    // Recover state parameters
    float px = x_state[0];
    float py = x_state[1];
    float vx = x_state[2];
    float vy = x_state[3];
    
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = vx * py;
    float c4 = vy * px;
    
    if (px == 0 || py == 0) {
        cout << "Error for 0 value, James you wrote this" << endl;
    }
    
    //Calculating the jacobian matrix
    Hj << px / c2, py / c2, 0, 0,
          -py / c1, px / c1, 0, 0,
    (py * (c3 - c4)) / pow(c1 , (3/2)), (px * (c4 - c3)) / pow(c1, (3/2)), px / c2, py / c2;
    
    return Hj;
  
}
