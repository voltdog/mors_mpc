#include "HopfGaitScheduler.hpp"
#include <Eigen/Dense>
#include <cmath>

HopfGaitScheduler::HopfGaitScheduler()
    : r(4, 0.0), w(4, 0.0), sum_con(4, std::vector<double>(2, 0.0)), x(4, 0.0), y(4, 0.0),
      phi_walk({0, 0.5 * M_PI, M_PI, 1.5 * M_PI}),
      phi_trot({M_PI, 0, 0, M_PI}),
      phi_pace({M_PI, 0, M_PI, 0}),
      phi_gallop({0, 0, M_PI, M_PI}),
      phi_pronking({0, 0, 0, 0}),
      xy_init({0.12, -0.01, 0.0001, 0.0001, 0.0001, 0.0001, 0.0, 0.0}),
      t0(0.0), dt(0.01) {
        this->phi = {M_PI, 0.0, 0.0, M_PI};
      }

void HopfGaitScheduler::set_static_params(double ampl, double alpha, double lamb, double a)
                                        //   double x_offset, double y_offset) {
{
    this->mu = std::sqrt(ampl);
    this->alpha = alpha;
    this->lamb = lamb;
    this->a = a;
    // this->x_offset = x_offset;
    // this->y_offset = y_offset;
}

void HopfGaitScheduler::set_gait_params(double& w_sw, double& w_st, vector<double>& gait_template) 
{
    this->w_sw = w_sw;
    this->w_st = w_st;

    // switch (phi_type) {
    //     case 0: // WALK
    //         this->phi = phi_walk;
    //         break;
    //     case 1: // TROT
    //         this->phi = phi_trot;
    //         break;
    //     case 2: // PACE
    //         this->phi = phi_pace;
    //         break;
    //     case 3: // GALLOP
    //         this->phi = phi_gallop;
    //         break;
    //     case 4: // PRONKING
    //         this->phi = phi_pronking;
    //         break;
    //     default:
    //         this->phi = phi_trot; // Default to TROT
    //         break;
    // }
    // cout << "1" << endl;
    // cout << gait_temp[0] << endl;
    for (int i = 0; i < 4; i++)
    {
        this->phi[i] = gait_template[i];
        // cout << i << endl;
    }
}

void HopfGaitScheduler::init_integrator(double dt) {
    this->dt = dt;
    this->current_state = xy_init;
}

std::vector<double> HopfGaitScheduler::step() {
    // Perform one integration step
    boost::numeric::odeint::integrate_const(stepper, [this](const state_type& xy, state_type& dxydt, double t) {
        dxydt = hopf_osc(xy);
    }, current_state, t0, t0 + dt, dt);

    // Update phase outputs
    std::vector<double> phi_out(4);
    phi_out[0] = std::atan2(current_state[1], current_state[0]) / M_PI;
    phi_out[1] = std::atan2(current_state[3], current_state[2]) / M_PI;
    phi_out[2] = std::atan2(current_state[5], current_state[4]) / M_PI;
    phi_out[3] = std::atan2(current_state[7], current_state[6]) / M_PI;

    return phi_out;
}

std::vector<double> HopfGaitScheduler::hopf_osc(const state_type& xy) {
    // Unpack state variables
    x[0] = xy[0];
    y[0] = xy[1];
    x[1] = xy[2];
    y[1] = xy[3];
    x[2] = xy[4];
    y[2] = xy[5];
    x[3] = xy[6];
    y[3] = xy[7];

    // Compute dynamics
    std::vector<double> dxydt(8);
    for (int i = 0; i < 4; ++i) {
        r[i] = std::sqrt(x[i] * x[i] + y[i] * y[i]);
        w[i] = (w_sw / (std::exp(-a * y[i]) + 1)) + (w_st / (std::exp(a * y[i]) + 1));
        sum_con[i][0] = 0.0;
        sum_con[i][1] = 0.0;

        for (int j = 0; j < 4; ++j) {
            sum_con[i][0] += lamb * (std::cos(phi[i] - phi[j]) * x[j] - std::sin(phi[i] - phi[j]) * y[j]);
            sum_con[i][1] += lamb * (std::sin(phi[i] - phi[j]) * x[j] + std::cos(phi[i] - phi[j]) * y[j]);
        }

        dxydt[2 * i] = alpha * (mu - r[i] * r[i]) * x[i] - w[i] * y[i] + sum_con[i][0];
        dxydt[2 * i + 1] = alpha * (mu - r[i] * r[i]) * y[i] + w[i] * x[i] + sum_con[i][1];
    }

    return dxydt;
}