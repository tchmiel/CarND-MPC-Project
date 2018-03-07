//#pragma once

//https://gist.github.com/svdamani/47b045c82cf9cfb8a392
//Svdamani

//export DISPLAY=:0

//From <https://github.com/udacity/CarND-MPC-Quizzes/blob/master/install_Ipopt_CppAD.md> 
//namespace plt = matplotlibcpp;
//using CppAD::AD;


//#include "MPC.h"
//#include <math.h>
//#include <cppad/cppad.hpp>
//#include <cppad/ipopt/solve.hpp>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "matplotlibcpp.h"

#include "DebugMPC.h"
//#include <math.h>
//#include <cppad/cppad.hpp>
//#include <cppad/ipopt/solve.hpp>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "matplotlibcpp.h"


namespace plt = matplotlibcpp;

DebugMPC::DebugMPC () {}

DebugMPC::~DebugMPC () {}

   int  DebugMPC::GetPushCount() {
       return cte_vals.size();
   }

  void DebugMPC::PushValues(double cte, double delta, double v ) {
    cte_vals.push_back(cte);
    delta_vals.push_back(delta);
    v_vals.push_back(v);
  } 
  
  void DebugMPC::PushValues(double x, double y, double psi, double v, double cte, double epsi, double delta, double a) {
    x_vals.push_back(x);
    y_vals.push_back(y);
    psi_vals.push_back(psi);
    v_vals.push_back(v);
    cte_vals.push_back(cte);
    epsi_vals.push_back(epsi);
    delta_vals.push_back(delta);
    a_vals.push_back(a);
  }

  void DebugMPC::Plot3() {
        cout << "AAAA" << cte_vals.size() << " "<< delta_vals.size() << " "<< v_vals.size() << " " << endl;
    cout << "plot3" << endl;
    for (unsigned int j = 0;  j < cte_vals.size(); j++) {
    cout << j << " " << cte_vals[j] << " "<< delta_vals[j] << " "<< v_vals[j] <<  endl;
  }
  cout << endl;
//return;
        plt::subplot(3, 1, 1);
	    plt::title("CTE");
	    plt::plot(cte_vals);
	    plt::subplot(3, 1, 2);
	    plt::title("Delta (Radians)");
	    plt::plot(delta_vals);
	    plt::subplot(3, 1, 3);
	    plt::title("Velocity");
	    plt::plot(v_vals);	  
        //plt::savefig("output.png");
	    plt::show();

  }
  
  void DebugMPC::PlotAll() {
    plt::subplot(8, 1, 1);
    plt::title("CTE");
    plt::plot(cte_vals);
    plt::subplot(8, 1, 2);
    plt::title("Delta (Radians)");
    plt::plot(delta_vals);
    plt::subplot(8, 1, 3);
    plt::title("Velocity");
    plt::plot(v_vals);
    plt::subplot(8, 1, 4);
    plt::title("x");
    plt::plot(x_vals);
    plt::subplot(8, 1, 5);
    plt::title("y");
    plt::plot(y_vals);
    plt::subplot(8, 1, 6);
    plt::title("psi");
    plt::plot(psi_vals);
    plt::subplot(8, 1, 7);
    plt::title("epsi");
    plt::plot(epsi_vals);
    plt::subplot(8, 1, 8);
    plt::title("a");
    plt::plot(a_vals);
    plt::show();
    }
