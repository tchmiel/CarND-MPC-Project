
#include "DebugMPC.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

DebugMPC::DebugMPC() {
  plotIterations_ = false;
  num_iterations_ = 0;
}

DebugMPC::~DebugMPC() {}

bool DebugMPC::IsPlottingIterations() {
    return plotIterations_;
}

int DebugMPC::GetNumIterations()
{
    return num_iterations_;
}

void DebugMPC::SetNumIterations(int num_iterations)
{
    plotIterations_ = true;
    num_iterations_ = num_iterations;
}

void DebugMPC::SetPlotFilename(std::string filename)
{
    plot_filename_ = filename;
}

int DebugMPC::GetPushCount()
{
    return cte_vals_.size();
}

void DebugMPC::PushValues(double cte, double delta, double v)
{
    cte_vals_.push_back(cte);
    delta_vals_.push_back(delta);
    v_vals_.push_back(v);
}

void DebugMPC::PushValues(double x, double y, double psi, double v, double cte, double epsi, double delta, double a)
{
    x_vals_.push_back(x);
    y_vals_.push_back(y);
    psi_vals_.push_back(psi);
    v_vals_.push_back(v);
    cte_vals_.push_back(cte);
    epsi_vals_.push_back(epsi);
    delta_vals_.push_back(delta);
    a_vals_.push_back(a);
}

void DebugMPC::Plot3()
{
    if (!plotIterations_)
        return;

    plt::subplot(3, 1, 1);
    plt::title("CTE");
    plt::plot(cte_vals_);
    plt::subplot(3, 1, 2);
    plt::title("Delta (Radians)");
    plt::plot(delta_vals_);
    plt::subplot(3, 1, 3);
    plt::title("Velocity");
    plt::plot(v_vals_);
    cout << "Plotting output to: " << plot_filename_ << "!" << endl;
    if (!plot_filename_.empty()) {
        plt::save(plot_filename_);
    }
    plt::show();
}

void DebugMPC::PlotAll()
{
    if (!plotIterations_)
        return;

    plt::subplot(8, 1, 1);
    plt::title("CTE");
    plt::plot(cte_vals_);
    plt::subplot(8, 1, 2);
    plt::title("Delta (Radians)");
    plt::plot(delta_vals_);
    plt::subplot(8, 1, 3);
    plt::title("Velocity");
    plt::plot(v_vals_);
    plt::subplot(8, 1, 4);
    plt::title("x");
    plt::plot(x_vals_);
    plt::subplot(8, 1, 5);
    plt::title("y");
    plt::plot(y_vals_);
    plt::subplot(8, 1, 6);
    plt::title("psi");
    plt::plot(psi_vals_);
    plt::subplot(8, 1, 7);
    plt::title("epsi");
    plt::plot(epsi_vals_);
    plt::subplot(8, 1, 8);
    plt::title("a");
    plt::plot(a_vals_);
    if (!plot_filename_.empty()) {
        plt::save(plot_filename_);
    }
    plt::show();
}
