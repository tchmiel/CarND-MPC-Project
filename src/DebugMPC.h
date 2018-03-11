#ifndef DebugMPC_H
#define DebugMPC_H

#include <string>
#include <vector>

using namespace std;

class DebugMPC {
    std::vector<double> x_vals_ = {};
    std::vector<double> y_vals_ = {};
    std::vector<double> psi_vals_ = {};
    std::vector<double> v_vals_ = {};
    std::vector<double> cte_vals_ = {};
    std::vector<double> epsi_vals_ = {};
    std::vector<double> delta_vals_ = {};
    std::vector<double> a_vals_ = {};

    bool plotIterations_ = false;
    int num_iterations_;
    std::string plot_filename_;

public:
    DebugMPC();

    virtual ~DebugMPC();

    bool IsPlottingIterations();

    int GetNumIterations();
    void SetNumIterations(int num_iterations);

    void SetPlotFilename(std::string filename);

    int GetPushCount();
    void PushValues(double cte, double delta, double v);
    void PushValues(double x, double y, double psi, double v, double cte, double epsi, double delta, double a);

    void Plot3();
    void PlotAll();
};

#endif /* DebugMPC_H */
