#ifndef DebugMPC_H
#define DebugMPC_H



using namespace std;

class DebugMPC {
    std::vector<double> x_vals = {};
    std::vector<double> y_vals = {};
    std::vector<double> psi_vals = {};
    std::vector<double> v_vals = {};
    std::vector<double> cte_vals = {};
    std::vector<double> epsi_vals = {};
    std::vector<double> delta_vals = {};
    std::vector<double> a_vals = {};

 public:
  DebugMPC();

  virtual ~DebugMPC();

  int  GetPushCount();
  void PushValues(double cte, double delta, double v ); 
  void PushValues(double x, double y, double psi, double v, double cte, double epsi, double delta, double a);

  void Plot3();
  void PlotAll();

    
};

#endif /* DebugMPC_H */
