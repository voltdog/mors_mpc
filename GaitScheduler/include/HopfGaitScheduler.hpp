#ifndef HOPFGAIT_SCHEDULER_HPP
#define HOPFGAIT_SCHEDULER_HPP

#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <cmath>
#include <vector>

using namespace std;
using namespace Eigen;
// using namespace YAML;

class HopfGaitScheduler {
public:
    HopfGaitScheduler();

    void set_static_params(double ampl = 1.0, double alpha = 100.0, double lamb = 1.0, double a = 1.0);
                        //    double x_offset = 0.084, double y_offset = 0.0);

    void set_gait_params(double& w_sw, double& w_st, vector<double>& gait_template);// int phi = 1); // Default gait: TROT

    void init_integrator(double dt);

    std::vector<double> step();

private:
    // State variables
    std::vector<double> r;
    std::vector<double> w;
    std::vector<std::vector<double>> sum_con;
    std::vector<double> x;
    std::vector<double> y;

    // Gait type parameters
    std::vector<double> phi_walk;
    std::vector<double> phi_trot;
    std::vector<double> phi_pace;
    std::vector<double> phi_gallop;
    std::vector<double> phi_pronking;

    std::vector<double> xy_init;

    // Static parameters
    double mu;
    double alpha;
    double lamb;
    double a;
    // double x_offset;
    // double y_offset;

    // Gait parameters
    double w_sw;
    double w_st;
    std::vector<double> phi;

    // Integrator
    typedef std::vector<double> state_type;
    boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;
    state_type current_state;
    double t0;
    double dt;

    // Internal methods
    std::vector<double> hopf_osc(const state_type& xy);
};

#endif // HOPFGAIT_SCHEDULER_HPP