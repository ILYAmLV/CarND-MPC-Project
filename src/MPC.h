#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

// *********************** beginning \\// **************************
 
// adapted from Udacity CarND MPC Quizzes
// https://github.com/earlbread/CarND-MPC-Project
 
  vector<double> mpc_x_vals;
  vector<double> mpc_y_vals;
// *************************** end //\\ ****************************

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
