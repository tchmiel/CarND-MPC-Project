# Model Predictive Control
## Self-Driving Car Engineer Nanodegree Program
## Term 2 - Project 5

### Tom Chmielenski
#### March 2018

----

In this project, we implement Model Predictive Control to drive a vehicle around the Udacity Term 2 Simulator track.

The goals of this project were to:
    
    * Describe the state, actuators, and update equations using in our Model Predictive Contol (MPC).
    * Choose approriate a N (timestep length) and dt (elapsed duration between timestamp) values.
    * Preprocess waypoints with by fitting a polynomial
    * Handle a 100 millisecond latency with the MPC.

---

###  Model Predictive Contol (MPC)

*TODO:  Student describes their model in detail. This includes the state, actuators and update equations.*


We need to shift waypoints to same coordinates as the vehicles to simplify the calculations.  

simplification calculations.
Substract all our points from our current position
so x, y coordinates are at 0,0.
psi zero as well, rotate all of our points.

// Lf, given for the simulator
  double Lf = 2.67;

// Objective
const ref_cte = 0;  // reference of cte
const ref_epsi = 0; // reference of error psi
const ref_v = 100;  // reference of 100 mph

// vector offsets for 1D vector
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = psi_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = delta_start + N;
size_t a_start = delta_start + N;

// cost function 
    fg[0] = 0;

    // Reference State Cost
    // tune by setting  coefficients to how much attention to pay attention to attributes
    for (int i = 0; i < N; i ++){
        //fg[0] += 2000 * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
        //fg[0] += 2000 * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
        fg[0] += CppAD::pow(vars[cte_start + i] - ref_cte, 2);
        fg[0] += CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
        fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    for (int i = 0; i < N - 1; i ++){
        //fg[0] += 5 * CppAD::pow(vars[delta_start + i], 2);
        //fg[0] += 5 * CppAD::pow(vars[a_start + i], 2);
        fg[0] += CppAD::pow(vars[delta_start + i], 2);
        fg[0] += CppAD::pow(vars[a_start + i], 2);
    }

    for (int i = 0; i < N - 2; i ++){
        //fg[0] += 200 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
        //fg[0] += 10 * CppAD::pow(vars[a_start + i + 1] -  vars[a_start + i], 2);
        fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
        fg[0] += CppAD::pow(vars[a_start + i + 1] -  vars[a_start + i], 2);
    }
   // Initial Constraints
    // 
    // We add 1 to each of the starting indices due to cost being location at index 0 of fg;
    // This bumps up the position of the all the other values
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of constraints
    for (int i = 0; i < N - 1; i ++) {
        AD<double> x1 = vars[x_start + i + 1];
        AD<double> y1 = vars[y_start + i + 1];
        AD<double> psi1 = vars[psi_start + i + 1];
        AD<double> v1 = vars[v_start + i + 1];
        AD<double> cte1 = vars[cte_start + i + 1];
        AD<double> epsi1 = vars[epsi_start + i + 1];

        AD<double> x0 = vars[x_start + i];
        AD<double> y0 = vars[y_start + i];
        AD<double> psi0 = vars[psi_start + i];
        AD<double> v0 = vars[v_start + i];
        AD<double> cte0 = vars[cte_start + i];
        AD<double> epsi0 = vars[epsi_start + i];

        AD<double> delta0 = vars[delta_start + i];
        AD<double> a0 = vars[a_start + i];

        AD<double> f0 = coeffs[0] + 
                        (coeffs[1] * x0) + 
                        (coeffs[2] * x0 * x0) + 
                        (coeffs[3] * x0 * x0 * x0);
        AD<double> psides0 = CPPAd::atan((3*coeffs[3] * x0 * x0 +
                                         (2*coeffs[2] * x0) +
                                         coeffs[1]);

      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + i[] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }


### solve
  double x = state[x];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  //size_t n_vars = 0;
  size_t n_vars = N * 6 + (N - 1) * 2;

  // TODO: Set the number of constraints
  //size_t n_constraints = 0;
  size_t n_constraints = N * 6;


  for (int i = 0; i < delta_start) {
      vars_lowerbound[i] = -1.01e19;
      vars_upperbound[i] = 1.01e19;

  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // 25 degrees to random == 0.436332 * Lf;
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // set the iniital upper and lower constaints to initial state value
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





### vector layout


---

###  Choose Timestamp Length and Elapsed Duration

*TODO:  Preprocess waypoints with by fitting a polynomial
Initially set based on Project Review
    size_t N = 10;
    double dt = 0.1;

---

###  Preprocess waypoints

*TODO: Preprocess waypoints with by fitting a polynomial


---

###  Latency

*TODO:  Handle a 100 millisecond latency with the MPC.




![PID Formula](PIDFormula.png)
Source: [Wikipedia](https://en.wikipedia.org/wiki/PID_controller)



----
Notes:



polynominal fit - same orientation as our line, and mostly horiztonal





Two Objectives
    Speed - How fast will the car go - cost function.
    Follow the line - insert waypoints, 6 polynominal - close to this line.fit     polynominal thru 6 waypoints.
