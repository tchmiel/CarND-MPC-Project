#include "DebugMPC.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
    int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (unsigned int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

static void show_usage(std::string name)
{
    std::cerr << "Usage: " << name << " <option(s)> SOURCES"
              << "Options:\n"
              << "\t-h,--help\t\t\tShow this help message\n"
              << "\t-p,--plot NUM_ITERATIONS\tSpecify number of iterations to run before plotting\n"
              << "\t-f,--filename FILENAME\t\tSpecify filename to save plot to\n"
              << std::endl;
}

int main(int argc, char* argv[])
{
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;
    DebugMPC debugMPC;

    if (argc > 6) {
        show_usage(argv[0]);
        return 1;
    }

    std::string savePlotFilename;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) {
            show_usage(argv[0]);
            return 0;
        } else if ((arg == "-p") || (arg == "--plot")) {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                int num_its = strtol(argv[++i], NULL, 10);
                debugMPC.SetNumIterations(num_its); // Increment 'i' so we don't get the argument as the next argv[i].
                cout << "Running simulator for " << debugMPC.GetNumIterations() << "!" << endl;
            } else { // Uh-oh, there was no argument to the plot option.
                std::cerr << "--plot option requires one argument." << std::endl;
                return 1;
            }
        } else if ((arg == "-f") || (arg == "--filename")) {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                debugMPC.SetPlotFilename(argv[++i]); // Increment 'i' so we don't get the argument as the next argv[i].
            } else { // Uh-oh, there was no argument to the saveplot destination option.
                std::cerr << "--filename option requires one argument." << std::endl;
                return 1;
            }
        }
    }

    h.onMessage([&mpc, &debugMPC](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
                    uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        cout << sdata << endl;
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];

                    // Simplify calculation by shifting to cars reference angle
                    for (unsigned int i = 0; i < ptsx.size(); i++) {
                        // shift car reference angle to 90 degrees
                        double shift_x = ptsx[i] - px;
                        double shift_y = ptsy[i] - py;

                        ptsx[i] = (shift_x * cos(0 - psi) - shift_y * sin(0 - psi));
                        ptsy[i] = (shift_x * sin(0 - psi) + shift_y * cos(0 - psi));
                    }

                    // Convert from vector<double> to Eigen::VectorXd
                    double* ptrx = &ptsx[0];
                    Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
                    double* ptry = &ptsy[0];
                    Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

                    auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

                    // Calculate Cross Track Error (cte) and error psi (epsi)
                    //  not truly the cte, it is the horiztonal
                    double cte = polyeval(coeffs, 0);

                    //since the initial point is 0,0, and 0, we can simplify this equation.
                    //psi and px = 0;
                    //double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 coeffs[3] * pow (px, 2));
                    double epsi = -atan(coeffs[1]);

                    // Lf, given for the simulator
                    // Distance between front wheels to CoG (in meters)
                   double Lf = 2.67;       

                    Eigen::VectorXd state(6);
                   
                    // Latency of 100ms
                    bool includeLatency = true;
                    if (includeLatency) {

                      //use steering_angle and throttle, use for delay.
                      //throttle is not the same as acceleration
                      // for low delays, it will work as an estimator.
                      double steer_value = j[1]["steering_angle"];
                      double throttle_value = j[1]["throttle"];
                      
                      const double latency = 0.1;    // 100 ms 
                      
                      double lat_px = v * latency;
                      double lat_py = 0.0;
                      double lat_psi = v * -(steer_value / Lf) * latency;
                      double lat_v = v + throttle_value * latency;
                      double lat_cte = cte * v * sin(epsi) * latency;
                      double lat_epsi = epsi + lat_psi;

                      state << lat_px, lat_py, lat_psi, lat_v, lat_cte, lat_epsi;  
                    } else {
                      state << 0, 0, 0, v, cte, epsi;  // no latency
                    }
                    
                    auto vars = mpc.Solve(state, coeffs);

                    //Display the MPC predicted trajectory
                    vector<double> mpc_x_vals;
                    vector<double> mpc_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Green line
                    for (unsigned int i = 2; i < vars.size(); i++) {
                        if (i % 2 == 0) {
                            mpc_x_vals.push_back(vars[i]);
                        } else {
                            mpc_y_vals.push_back(vars[i]);
                        }
                    }

                    //Display the waypoints/reference line
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;

                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a Yellow line
                    double poly_inc = 2.5;
                    int num_points = 25;
                    for (int i = 1; i < num_points; i++) {
                        next_x_vals.push_back(poly_inc * i);
                        next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
                    }


                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = -vars[0] / (deg2rad(25) * Lf);
                    msgJson["throttle"] = vars[1];

                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;

                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    std::cout << msg << std::endl;
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(100));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    double cte2 = vars[4];
                    double delta2 = vars[6];
                    double v2 = vars[7];

                    debugMPC.PushValues(cte2, delta2, v2);
                    if (debugMPC.IsPlottingIterations()) {
                      if (debugMPC.GetPushCount() % debugMPC.GetNumIterations() == 0) {
                          debugMPC.Plot3();
                          exit(1);
                      }
                    }
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse* res, uWS::HttpRequest req, char* data,
                        size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                          char* message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
