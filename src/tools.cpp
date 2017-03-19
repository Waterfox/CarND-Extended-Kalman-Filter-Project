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
  VectorXd rmse(4);
	VectorXd se(4);
	rmse << 0,0,0,0;
	se << 0,0,0,0;

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
  }

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        VectorXd e =(estimations[i] - ground_truth[i]);
        VectorXd ee = e.array()*e.array();
        se += ee;

	}

	//calculate the mean
	VectorXd mean = 1.0/estimations.size() * se.array();

	//calculate the squared root
	// ... your code here
    rmse = mean.array().sqrt();
	//return the result
	return rmse;
}


MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //TODO: YOUR CODE HERE

  //check division by zero and return zeros if True
  float rsq = pow(px,2) + pow(py,2);
  if (rsq == 0){
      cout << "Division by zero error!";
      Hj << 0, 0 , 0, 0,
          0, 0 , 0, 0,
          0, 0, 0, 0;
          //Hj.setZero();
      return Hj;
  }

  //compute the Jacobian matrix
  //equation Hj from L16
  Hj << px/sqrt(rsq), py/sqrt(rsq) , 0, 0,
          -py/rsq, px/rsq , 0, 0,
          py*(vx*py-vy*px)/pow(rsq,1.5), px*(vy*px-vx*py)/pow(rsq,1.5), px/sqrt(rsq),  py/sqrt(rsq);


  return Hj;
}
