#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 10;  // tried 5, 7, 9, 12, 15
double dt = 0.2;// tried 0.01, 0.1, 0.15

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// ******************************* beginning \\// **************************************

double ref_v = 100 * 0.44704; // convert from MPH to m/s

// adapted from Udacity CarND MPC Quizzes
// https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp

double x_start = 0;
double y_start = x_start + N;
double psi_start = y_start + N;
double v_start = psi_start + N;
double cte_start = v_start + N;
double epsi_start = cte_start + N;
double delta_start = epsi_start + N;
double a_start = delta_start + N - 1;

// ************************************* end //\\ *************************************

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

// ************************************* beginning \\// ****************************

	  // adapted from lesson 9. Solution: Mind The Line
	  // The cost is stored is the first element of `fg`.
	  // Any additions to the cost should be added to `fg[0]`.
	  fg[0] = 0;

	  // The part of the cost based on the reference state.
	  for (int t = 0; t < N; t++) {
		  fg[0] += 4000 * CppAD::pow(vars[cte_start + t], 2); // CTE (yellow line)
		  fg[0] += 3000 * CppAD::pow(vars[epsi_start + t], 2);// EPSI (green line)
		  fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);  // Speed
	  }

	  // Minimize the use of actuators.
	  for (int t = 0; t < N - 1; t++) {
		  fg[0] += CppAD::pow(vars[delta_start + t], 2);// Steering
		  fg[0] += CppAD::pow(vars[a_start + t], 2);	// Throttle
	  }

	  // Minimize the value gap between sequential actuations.
	  for (int t = 0; t < N - 2; t++) {
		  fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);// Steering
		  fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);		// Throttle
	  }

	  // Initial constraints
	  //
	  // We add 1 to each of the starting indices due to cost being located at
	  // index 0 of `fg`.
	  // This bumps up the position of all the other values.
	  fg[1 + x_start] = vars[x_start];
	  fg[1 + y_start] = vars[y_start];
	  fg[1 + psi_start] = vars[psi_start];
	  fg[1 + v_start] = vars[v_start];
	  fg[1 + cte_start] = vars[cte_start];
	  fg[1 + epsi_start] = vars[epsi_start];

	  // The rest of the constraints
	  for (int t = 1; t < N; t++) {
		  // The state at time t+1 .
		  AD<double> x1 = vars[x_start + t];
		  AD<double> y1 = vars[y_start + t];
		  AD<double> psi1 = vars[psi_start + t];
		  AD<double> v1 = vars[v_start + t];
		  AD<double> cte1 = vars[cte_start + t];
		  AD<double> epsi1 = vars[epsi_start + t];

		  // The state at time t.
		  AD<double> x0 = vars[x_start + t - 1];
		  AD<double> y0 = vars[y_start + t - 1];
		  AD<double> psi0 = vars[psi_start + t - 1];
		  AD<double> v0 = vars[v_start + t - 1];
		  AD<double> cte0 = vars[cte_start + t - 1];
		  AD<double> epsi0 = vars[epsi_start + t - 1];

		  // Only consider the actuation at time t.
		  AD<double> delta0 = vars[delta_start + t - 1];
		  AD<double> a0 = vars[a_start + t - 1];

		  AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
		  AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

		  // Here's `x` to get you started.
		  // The idea here is to constraint this value to be 0.
		  //
		  // Recall the equations for the model:
		  // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
		  // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
		  // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
		  // v_[t+1] = v[t] + a[t] * dt
		  // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
		  // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
		  fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
		  fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
		  fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
		  fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
		  fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
		  fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
	  }
  }
};

// ***************************************** end //\\ ********************************************************

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

	bool ok = true;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	// ******************************* beginning \\// **************************

	// adapted from Udacity CarND MPC Quizzes
	// https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp

	double x = state[0];	 // position x
	double y = state[1];	 // position y
	double psi = state[2]; // bearing angle (yaw)
	double v = state[3];	 // velocity
	double cte = state[4]; // cross track error
	double epsi = state[5];// orientation error

	// ********************************** end //\\ *******************************

	// TODO: Set the number of model variables (includes both states and inputs).
	size_t n_vars = N * 6 + (N - 1) * 2; // Number of independent variables

	// TODO: Set the number of constraints
	size_t n_constraints = N * 6; // Number of constraints

	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	Dvector vars(n_vars);
	for (int i = 0; i < n_vars; i++) {
		vars[i] = 0;
	}

	// *************************************** beginning \\// ********************************

	// adapted from Udacity CarND MPC Quizzes
	// https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp

	// Set the initial variable values
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;

	// ******************************************** end //\\ **********************************

	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);
	// TODO: Set lower and upper limits for variables.

	// *************************************** beginning \\// *******************************

	// adapted from Udacity CarND MPC Quizzes
	// https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp

	for (int i = 0; i < delta_start; i++) {
		vars_lowerbound[i] = -1.0e19;			// Set all non-actuators upper and lowerlimits
		vars_upperbound[i] = 1.0e19;			// to the max negative and positive values.
	}

	for (int i = delta_start; i < a_start; i++) {
		vars_lowerbound[i] = -0.436332;		// The upper and lower limits of delta are set to -25 and 25
		vars_upperbound[i] = 0.436332;		// degrees (values in radians).
	}

	for (int i = a_start; i < n_vars; i++) {
		vars_lowerbound[i] = -1.0;			// Acceleration/decceleration upper and lower limits.
		vars_upperbound[i] = 1.0;
	}
	// ******************************************** end //\\ ******************************

	// Lower and upper limits for the constraints
	// Should be 0 besides initial state.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (int i = 0; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}

	// ************************************* beginning \\// **************************

	// adapted from Udacity CarND MPC Quizzes
	// https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp

	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;

	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
	constraints_upperbound[epsi_start] = epsi;

	// ****************************************** end //\\ *****************************

	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	// NOTE: You don't have to worry about these options
	//
	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";

	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
		constraints_upperbound, fg_eval, solution);

	// Check some of the solution values
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Cost
	auto cost = solution.obj_value;
	std::cout << "Cost " << cost << std::endl;

	// TODO: Return the first actuator values. The variables can be accessed with
	// `solution.x[i]`.

	// ******************************** beginning \\//**********************

	// adapted from Udacity CarND MPC Quizzes
	// https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp

	mpc_x_vals = {};
	mpc_y_vals = {};

	for (int i = 0; i < N - 1; i++)
	{
	    mpc_x_vals.push_back(solution.x[x_start + i + 1]);
	    mpc_y_vals.push_back(solution.x[y_start + i + 1]);
	}
    return{ solution.x[delta_start], solution.x[a_start] };
  }

// ********************************* end //\\ ***************************